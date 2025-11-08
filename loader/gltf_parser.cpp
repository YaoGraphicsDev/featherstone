#include "gltf_parser.h"

#define TINYGLTF_IMPLEMENTATION
#include "tiny_gltf.h"
#include "json.hpp"
#include "gltf_traits.h"

#include <iostream>

namespace tg = tinygltf;
namespace nl = nlohmann;
using namespace Eigen;

tg::Model model;
tg::TinyGLTF loader;
std::string err;
std::string warn;

std::vector<ImplicitShape> g_shapes;
std::vector<PhysicsMaterial> g_materials;
std::map<int, Collider> g_colliders;

enum class NodePhysicsLabel {
	NonPhysical = 0,
	ConvexHullColliderShapeData,

	// Static-Dynamic / CompoundParent-CompoundChild-Trivial / Implicit-ConvexHull
	StaticCompoundParent,
	DynamicCompoundParent,
	CompoundChildImplicit,
	CompoundChildConvexHull,
	StaticTrivialImplicit,
	StaticTrivialConvexHull,
	DynamicTrivialImplicit,
	DynamicTrivialConvexHull,
};
std::vector<NodePhysicsLabel> g_phy_labels;

const std::string KHR_implicit_shapes = "KHR_implicit_shapes";
const std::string KHR_physics_rigid_bodies = "KHR_physics_rigid_bodies";
const std::string khr_physics_extra_props = "khr_physics_extra_props";

template<typename T>
static bool load_accessor(int accessor_id, std::vector<T>& buffer) {
	tg::Accessor& acc = model.accessors[accessor_id];
	tg::BufferView& bv = model.bufferViews[acc.bufferView];
	tg::Buffer& buf = model.buffers[bv.buffer];

	// type check
	if (GltfElementTraits<T>::gltf_type != acc.type ||
		GltfElementTraits<T>::gltf_component_type != acc.componentType) {
		std::cout << "data type mismatch. accessor.type = " << acc.type
			<< ", accessor.componentType = " << acc.componentType
			<< ", target type = " << TypeReflect<T>::name << std::endl;
		return false;
	}

	const unsigned char* ptr = buf.data.data() + bv.byteOffset + acc.byteOffset;
	size_t count = acc.count;

	size_t element_size = sizeof(T);
	size_t stride = bv.byteStride ? bv.byteStride : element_size;

	buffer.resize(count);
	for (size_t i = 0; i < count; ++i) {
		const void* srcPtr = ptr + stride * i;
		std::memcpy(&buffer[i], srcPtr, sizeof(T));
	}

	return true;
}

template<typename T>
static bool load_attribute(
	const tinygltf::Primitive& prim,
	const std::string& name,
	std::vector<T>& buffer,
	int mesh_id,
	int prim_id)
{
	auto iter = prim.attributes.find(name);
	if (iter != prim.attributes.end()) {
		if (!load_accessor(iter->second, buffer)) {
			std::cout << "error parsing attribute " << name << std::endl;
			return false;
		}
	}
	return true;
}


// a gltf mesh primitive corresponds to a renderable
static bool load_primitive(int mesh_id, int prim_id, Renderable& renderable) {
	const tg::Primitive& prim = model.meshes[mesh_id].primitives[prim_id];

	// load geometry
	renderable.mesh = std::make_shared<MeshData>();

	if (prim.mode != TINYGLTF_MODE_TRIANGLES) {
		std::cout << "cannot process primitive mode = " << prim.mode << std::endl;
		return false;
	}

	if (!load_accessor(prim.indices, renderable.mesh->indices)) {
		std::cout << "error parsing indices. indices_id = " << prim.indices << std::endl;
		return false;
	}

	bool ret = true;
	ret &= load_attribute(prim, "POSITION", renderable.mesh->positions, mesh_id, prim_id);
	ret &= load_attribute(prim, "NORMAL", renderable.mesh->normals, mesh_id, prim_id);
	ret &= load_attribute(prim, "TEXCOORD_0", renderable.mesh->uv0, mesh_id, prim_id);

	if (!ret) {
		return false;
	}
	
	return true;
}

static nl::json extension_property(const tg::Node& node, const std::string& extension_name) {
	if (node.extensions_json_string.empty()) {
		// no extension at all
		return nl::json();
	}

	nl::json ext = nl::json::parse(node.extensions_json_string);

	if (!ext.contains(extension_name)) {
		// cant find specific extension
		return nl::json();
	}

	nl::json& khr = ext[extension_name];
	return khr;
}

static nl::json extra_property(const tg::Node& node, const std::string& extra_name) {
	if (node.extras_json_string.empty()) {
		// no extras at all
		return nl::json();
	}

	nl::json ext = nl::json::parse(node.extras_json_string);

	if (!ext.contains(extra_name)) {
		// cant find specific extension
		return nl::json();
	}

	nl::json& khr = ext[extra_name];
	return khr;
}

static bool label_physical_node(int node_id, bool as_child) {
	tg::Node& node = model.nodes[node_id];
	nl::json rb_props = std::move(extension_property(node, KHR_physics_rigid_bodies));
	nl::json extra_props = std::move(extra_property(node, khr_physics_extra_props));

	bool has_valid_mass = false;
	if (!rb_props.is_null() && rb_props.contains("motion") && rb_props["motion"].contains("mass")) {
		has_valid_mass = true;
	}
	bool non_renderable = false;
	if (!extra_props.is_null() && extra_props.contains("non_renderable") && extra_props["non_renderable"] == 1) {
		non_renderable = true;
	}
	bool implicit = false;
	int mesh_data_node_id = -1;
	if (!rb_props.is_null() && rb_props.contains("collider") && rb_props["collider"].contains("geometry")) {
		nl::json& geo_props = rb_props["collider"]["geometry"];
		if (geo_props.contains("shape")) {
			implicit = true;
		}
		// if collider is not implicit, the shape of it is stored as a mesh. It has to be a convex hull
		else {
			if (!geo_props.contains("convexHull") || !(bool)geo_props["convexHull"]) {
				std::cout << "unsupported collider.geometry" << std::endl;
				return false;
			}
			if (!geo_props.contains("node")) {
				std::cout << "collider.geometry.node field not found" << std::endl;
				return false;
			}
			mesh_data_node_id = geo_props["node"];
		}
	}


	for (int c_id : node.children) {
		// label children first
		if (!label_physical_node(c_id, true)) {
			std::cout << "error labeling node. node_id = " << c_id << std::endl;
			return false;
		}
	}

	auto add_label = [&](NodePhysicsLabel label, int id) {
		size_t size = g_phy_labels.size();
		g_phy_labels.resize(std::max((size_t)(id + 1), size));
		g_phy_labels[id] = label;
	};



	// label the node itself
	if (node.children.empty()) {
		if (!non_renderable && rb_props.is_null()) add_label(NodePhysicsLabel::NonPhysical, node_id);
		else if (!non_renderable && !rb_props.is_null() && !as_child && has_valid_mass && implicit)								add_label(NodePhysicsLabel::DynamicTrivialImplicit, node_id);
		else if (!non_renderable && !rb_props.is_null() && !as_child && has_valid_mass && !implicit && mesh_data_node_id >= 0)	add_label(NodePhysicsLabel::DynamicTrivialConvexHull, node_id);
		else if (!non_renderable && !rb_props.is_null() && !as_child && !has_valid_mass && implicit)							add_label(NodePhysicsLabel::StaticTrivialImplicit, node_id);
		else if (!non_renderable && !rb_props.is_null() && !as_child && !has_valid_mass && !implicit && mesh_data_node_id >= 0)	add_label(NodePhysicsLabel::StaticTrivialConvexHull, node_id);
		else if (non_renderable && !rb_props.is_null()&& as_child && !has_valid_mass && implicit)								add_label(NodePhysicsLabel::CompoundChildImplicit, node_id);
		else if (non_renderable && !rb_props.is_null()&& as_child && !has_valid_mass && !implicit && mesh_data_node_id >= 0)	add_label(NodePhysicsLabel::CompoundChildConvexHull, node_id);
		else if (non_renderable && !rb_props.is_null() && !as_child) {
			std::cout << "Trivial object has to be renderable" << std::endl;
			return false;
		}
		else {
			std::cout << "cannot find suitable label for node. rb_props.is_null() = " << rb_props.is_null() << 
				", non_renderable = " << non_renderable << 
				", has_valid_mass = " << has_valid_mass <<
				", as_child = " << as_child << 
				", implicit = " << implicit << 
				", mesh_data_node_id = " << mesh_data_node_id << std::endl;
			return false;
		}

		if (g_phy_labels[node_id] == NodePhysicsLabel::CompoundChildConvexHull ||
			g_phy_labels[node_id] == NodePhysicsLabel::DynamicTrivialConvexHull ||
			g_phy_labels[node_id] == NodePhysicsLabel::StaticTrivialConvexHull) {
			if (model.nodes[mesh_data_node_id].mesh < 0) {
				std::cout << "A node that is supposed to store convex hull mesh data does not have mesh. node_id = " << mesh_data_node_id << std::endl;
				return false;
			}
			add_label(NodePhysicsLabel::ConvexHullColliderShapeData, mesh_data_node_id);
		}
		return true;
	}

	// has children. NonPhysical / StaticCompoundParent / DynamicCompoundParent,
	NodePhysicsLabel child_label = g_phy_labels[node.children[0]];
	bool children_legal = std::all_of(node.children.begin(), node.children.end(), [&](int c_id) {
		if (child_label == NodePhysicsLabel::NonPhysical) {
			return g_phy_labels[c_id] == child_label;
		}
		else if (child_label == NodePhysicsLabel::CompoundChildImplicit || child_label == NodePhysicsLabel::CompoundChildConvexHull) {
			return g_phy_labels[c_id] == NodePhysicsLabel::CompoundChildImplicit ||
				g_phy_labels[c_id] == NodePhysicsLabel::CompoundChildConvexHull;
		}
		else {
			return false;
		}
	});
	if (!children_legal) {
		std::cout << "Illegal children combination. Children's labels are: ";
		std::for_each(node.children.begin(), node.children.end(), [&](int c_id) {
			std::cout << (int)g_phy_labels[c_id] << ", ";
		});
		std::cout << std::endl;
		return false;
	}

	if (child_label == NodePhysicsLabel::NonPhysical) {
		if (!rb_props.is_null()) {
			std::cout << "If all children are non-physical, parent cannot be physical" << std::endl;
			return false;
		}
		add_label(NodePhysicsLabel::NonPhysical, node_id);
	}
	else if (child_label == NodePhysicsLabel::CompoundChildImplicit || child_label == NodePhysicsLabel::CompoundChildConvexHull) {
		if (non_renderable) {
			std::cout << "A physical parent has to be renderable" << std::endl;
			return false;
		}
		if (has_valid_mass) add_label(NodePhysicsLabel::DynamicCompoundParent, node_id);
		else add_label(NodePhysicsLabel::StaticCompoundParent, node_id);
	}
	else {
		// Should never happen
		assert(false);
	}

	// check if physics material exists
	if (g_phy_labels[node_id] == NodePhysicsLabel::CompoundChildImplicit ||
		g_phy_labels[node_id] == NodePhysicsLabel::CompoundChildConvexHull ||
		g_phy_labels[node_id] == NodePhysicsLabel::StaticTrivialImplicit ||
		g_phy_labels[node_id] == NodePhysicsLabel::StaticTrivialConvexHull ||
		g_phy_labels[node_id] == NodePhysicsLabel::DynamicTrivialImplicit ||
		g_phy_labels[node_id] == NodePhysicsLabel::DynamicTrivialConvexHull) {
		if (!rb_props["collider"].contains("physicsMaterial")) {
			std::cout << "Cannot find physics material. Label = " << (int)g_phy_labels[node_id] << std::endl;
			return false;
		}
	}

	return true;
}

static bool label_all_physical_nodes() {
	for (int node_id : model.scenes[0].nodes) {
		if(!label_physical_node(node_id, false)) {
			std::cout << "error labeling node. node_id = " << node_id << std::endl;
			return false;
		}
	}

	return true;
}

static bool parse_rigid_transform(const tg::Node& node, Vector3f& trans, Quaternionf& rot) {
	if (!node.matrix.empty()) {
		std::cout << "error parsing rigid transform. Matrix not supported." << std::endl;
		return false;
	}

	trans = Vector3f::Zero();
	rot = Quaternionf::Identity();

	if (!node.translation.empty()) {
		trans.x() = (float)node.translation[0];
		trans.y() = (float)node.translation[1];
		trans.z() = (float)node.translation[2];
	}

	if (!node.rotation.empty()) {
		rot.x() = (float)node.rotation[0];
		rot.y() = (float)node.rotation[1];
		rot.z() = (float)node.rotation[2];
		rot.w() = (float)node.rotation[3];
		rot.normalize();
	}

	if (!node.scale.empty()) {
		std::cout << "scaling factor will mess up rigidbody simulations" << std::endl;
		return false;
	}

	return true;
}

static std::shared_ptr<Collider> build_implicit_collider(int node_id) {
	std::shared_ptr<Collider> c = std::make_shared<Collider>();
	tg::Node& node = model.nodes[node_id];
	c->name = node.name;
	nl::json rb_props = std::move(extension_property(node, KHR_physics_rigid_bodies));
	assert(!rb_props.is_null()); // checked when labelling. This should never happen
	int shape_id = rb_props["collider"]["geometry"]["shape"];
	c->implicit_shape = std::make_shared<ImplicitShape>(g_shapes[shape_id]);
	if (g_phy_labels[node_id] == NodePhysicsLabel::CompoundChildImplicit) {
		Vector3f translation;
		Quaternionf rotation;
		if (!parse_rigid_transform(node, translation, rotation)) {
			std::cout << "error parsing mesh transforms." << std::endl;
			return nullptr;
		}
		c->implicit_shape_translation = translation;
		c->implicit_shape_rotation = rotation;
	}
	else {
		assert(g_phy_labels[node_id] == NodePhysicsLabel::DynamicTrivialImplicit || 
			g_phy_labels[node_id] == NodePhysicsLabel::StaticTrivialImplicit);
	}
	int material_id = rb_props["collider"]["physicsMaterial"];
	c->material = std::make_shared<PhysicsMaterial>(g_materials[material_id]);
	return c;
}

// "physicsMeshDataNode"
static std::shared_ptr<MeshData> build_convex_hull_shape(int node_id, Vector3f parent_translation, Quaternionf parent_rotation) {
	const tg::Node& node = model.nodes[node_id];
	int mesh_id = node.mesh;
	const tg::Primitive& prim = model.meshes[mesh_id].primitives[0];

	// load geometry
	std::shared_ptr<MeshData> m = std::make_shared<MeshData>();

	if (prim.mode != TINYGLTF_MODE_TRIANGLES) {
		std::cout << "cannot process primitive mode = " << prim.mode << ", mesh_id = " << mesh_id << std::endl;
		return false;
	}

	if (!load_accessor(prim.indices, m->indices)) {
		std::cout << "error parsing indices. mesh_id = " << mesh_id << ", indices_id = " << prim.indices << std::endl;
		return false;
	}

	if (!load_attribute(prim, "POSITION", m->positions, mesh_id, 0)) {
		std::cout << "cannot load POSITION attribute. mesh_id = " << mesh_id << std::endl;
		return nullptr;
	}

	Vector3f node_translation = Vector3f::Zero();
	Quaternionf node_rotation = Quaternionf::Identity();
	if (!parse_rigid_transform(node, node_translation, node_rotation)) {
		std::cout << "error parsing mesh transforms." << std::endl;
		return nullptr;
	}

	Quaternionf total_rotation = (parent_rotation * node_rotation).normalized();
	Vector3f total_translation = parent_rotation * node_translation + parent_translation;
	// apply transforms to vertices
	for (auto& p : m->positions) {
		p = PV3(total_rotation * EV3(p) + total_translation);
	}

	return m;
}

// 
static std::shared_ptr<Collider> build_convex_hull_collider(int node_id) {
	std::shared_ptr<Collider> c = std::make_shared<Collider>();
	tg::Node& node = model.nodes[node_id];
	c->name = node.name;
	nl::json rb_props = std::move(extension_property(node, KHR_physics_rigid_bodies));
	assert(!rb_props.is_null()); // checked when labelling. This should never happen
	int convex_hull_node_id = rb_props["collider"]["geometry"]["node"];

	Vector3f translation = Vector3f::Zero();
	Quaternionf rotation = Quaternionf::Identity();
	if (g_phy_labels[node_id] == NodePhysicsLabel::CompoundChildConvexHull) {
		if (!parse_rigid_transform(node, translation, rotation)) {
			std::cout << "error parsing convex hull transforms." << std::endl;
			return nullptr;
		}
	}
	else {
		assert(g_phy_labels[node_id] == NodePhysicsLabel::StaticTrivialConvexHull || 
			g_phy_labels[node_id] == NodePhysicsLabel::DynamicTrivialConvexHull);
	}
	c->convex_hull = build_convex_hull_shape(convex_hull_node_id, translation, rotation);

	int material_id = rb_props["collider"]["physicsMaterial"];
	c->material = std::make_shared<PhysicsMaterial>(g_materials[material_id]);
	return c;
}

static std::vector<Collider> build_compound_collider(int node_id) {
	std::vector<Collider> compound_collider;
	tg::Node& node = model.nodes[node_id];
	for (int c_id : node.children) {
		if (g_phy_labels[c_id] == NodePhysicsLabel::CompoundChildImplicit) {
			std::shared_ptr<Collider> c = build_implicit_collider(c_id);
			assert(c);
			compound_collider.push_back(*c);
		}
		else if (g_phy_labels[c_id] == NodePhysicsLabel::CompoundChildConvexHull) {
			std::shared_ptr<Collider> c = build_convex_hull_collider(c_id);
			assert(c);
			compound_collider.push_back(*c);
		}
		else {
			assert(false); // should never happen
		}
	}
	return compound_collider;
}

static bool load_node(int node_id, int parent_id, SceneGraph& scene) {
	const tg::Node& node = model.nodes[node_id];

	scene.emplace_back();
	SceneNode& scene_node = scene.back();
	int this_id = scene.size() - 1;

	scene_node.parent = parent_id;
	scene_node.name = node.name;

	Vector3f translation;
	Quaternionf rotation;
	if (!parse_rigid_transform(node, translation, rotation)) {
		std::cout << "error parsing rigid transforms. node_id = " << node_id << std::endl;
		return nullptr;
	}
	scene_node.local_translation = translation;
	scene_node.local_rotation = rotation;

	if (parent_id == -1) {
		scene_node.world_translation = translation;
		scene_node.world_rotation = rotation;
	}
	else {
		scene_node.world_rotation = (scene[parent_id].world_rotation * rotation).normalized();
		scene_node.world_translation = scene[parent_id].world_rotation * translation + scene[parent_id].world_translation;
	}

	// parse primitives
	if (node.mesh != -1) {
		for (int prim_id = 0; prim_id < model.meshes[node.mesh].primitives.size(); ++prim_id) {
			scene_node.renderables.emplace_back();
			bool ret = load_primitive(node.mesh, prim_id, scene_node.renderables.back());
			if (!ret) {
				std::cout << "error loading primitive. mesh_id = " << node.mesh << ", prim_id = " << prim_id << std::endl;
				return false;
			}
		}
	}

	// physical
	if (g_phy_labels[node_id] == NodePhysicsLabel::NonPhysical) {
		// recursively parse children
		for (int node_id : node.children) {
			bool ret = load_node(node_id, this_id, scene);
			if (!ret) {
				std::cout << "error loading node. node_id = " << node_id << std::endl;
				return false;
			}
		}
	}
	else if (g_phy_labels[node_id] == NodePhysicsLabel::StaticCompoundParent) {
		scene_node.physical.reset(new Physical);
		scene_node.physical->dyn_type = Physical::DynamicType::Static;
		scene_node.physical->mass = 0.0f;
		scene_node.physical->colliders = std::move(build_compound_collider(node_id));
	}
	else if (g_phy_labels[node_id] == NodePhysicsLabel::DynamicCompoundParent) {
		scene_node.physical.reset(new Physical);
		scene_node.physical->dyn_type = Physical::DynamicType::Dynamic;
		scene_node.physical->mass = (float)extension_property(node, KHR_physics_rigid_bodies)["motion"]["mass"];
		scene_node.physical->colliders = std::move(build_compound_collider(node_id));
	}
	else if (g_phy_labels[node_id] == NodePhysicsLabel::StaticTrivialImplicit) {
		scene_node.physical.reset(new Physical);
		scene_node.physical->dyn_type = Physical::DynamicType::Static;
		scene_node.physical->mass = 0.0f;
		scene_node.physical->colliders.push_back(*build_implicit_collider(node_id));
	}
	else if (g_phy_labels[node_id] == NodePhysicsLabel::StaticTrivialConvexHull) {
		scene_node.physical.reset(new Physical);
		scene_node.physical->dyn_type = Physical::DynamicType::Static;
		scene_node.physical->mass = 0.0f;
		scene_node.physical->colliders.push_back(*build_convex_hull_collider(node_id));
	}
	else if (g_phy_labels[node_id] == NodePhysicsLabel::DynamicTrivialImplicit) {
		scene_node.physical.reset(new Physical);
		scene_node.physical->dyn_type = Physical::DynamicType::Dynamic;
		scene_node.physical->mass = (float)extension_property(node, KHR_physics_rigid_bodies)["motion"]["mass"];
		scene_node.physical->colliders.push_back(*build_implicit_collider(node_id));
	}
	else if (g_phy_labels[node_id] == NodePhysicsLabel::DynamicTrivialConvexHull) {
		scene_node.physical.reset(new Physical);
		scene_node.physical->dyn_type = Physical::DynamicType::Dynamic;
		scene_node.physical->mass = (float)extension_property(node, KHR_physics_rigid_bodies)["motion"]["mass"];
		scene_node.physical->colliders.push_back(*build_convex_hull_collider(node_id));
	}
	else if (g_phy_labels[node_id] == NodePhysicsLabel::CompoundChildImplicit ||
		g_phy_labels[node_id] == NodePhysicsLabel::CompoundChildConvexHull) {
		// do nothing
	}
	else { assert(false); }


	return true;
}

static bool load_all_implicit_shapes(nlohmann::json& ext) {
	if (!ext.contains(KHR_implicit_shapes)) {
		std::cout << "KHR_implicit_shapes not present" << std::endl;
		return true;
	}

	nl::json& khr = ext[KHR_implicit_shapes];
	if (!khr.contains("shapes")) {
		std::cout << "shapes field not present in KHR_implicit_shapes" << std::endl;
		return true;
	}

	nl::json& shapes = khr["shapes"];
	for (size_t i = 0; i < shapes.size(); ++i) {
		nl::json& shape = shapes[i];
		ImplicitShape imp_shape;
		// Parse type
		if (!shape.contains("type")) {
			std::cout << "type field of KHR_implicit_shapes.shapes[" << i << "] not found" << std::endl;
			return false;
		}

		std::string type = shape["type"];
		if (type == "box") imp_shape.type = ImplicitShape::Type::Box;
		else if (type == "sphere") imp_shape.type = ImplicitShape::Type::Sphere;
		else if (type == "cylinder") imp_shape.type = ImplicitShape::Type::Cylinder;
		else if (type == "capsule") imp_shape.type = ImplicitShape::Type::Capsule;
		else { std::cout << "unrecognized shape type " << type << std::endl; return false; }

		// Parse implicit shape size
		if (imp_shape.type == ImplicitShape::Type::Box) {
			if (!shape.contains("box")) {
				std::cout << "Cannot find box parameters" << std::endl;
				return false;
			}

			nl::json& size = shape["box"]["size"];
			imp_shape.half_dims = Eigen::Vector3f((float)size[0], (float)size[1], (float)size[2]) * 0.5f;
		}
		else if (imp_shape.type == ImplicitShape::Type::Cylinder) {
			if (!shape.contains("cylinder")) {
				std::cout << "Cannot find cylinder parameters" << std::endl;
				return false;
			}

			nl::json& size = shape["cylinder"];
			imp_shape.half_dims = Eigen::Vector3f((float)size["radiusBottom"], (float)size["height"] * 0.5f, (float)size["radiusBottom"]);
		}
		else if (imp_shape.type == ImplicitShape::Type::Sphere) {
			if (!shape.contains("sphere")) {
				std::cout << "Cannot find sphere parameters" << std::endl;
				return false;
			}

			nl::json& size = shape["sphere"];
			imp_shape.half_dims = Eigen::Vector3f::Constant((float)size["radius"]);
		}
		else {
			std::cout << "Implicit shape not supported. Shape = " << (int)imp_shape.type << std::endl;
			assert(false);
			return false;
		}
		g_shapes.push_back(imp_shape);
	}

	return true;
}

static bool load_all_physics_materials(nlohmann::json& ext) {
	if (!ext.contains(KHR_physics_rigid_bodies)) {
		std::cout << "KHR_physics_rigid_bodies not present" << std::endl;
		return true;
	}

	nl::json& khr = ext[KHR_physics_rigid_bodies];
	if (!khr.contains("physicsMaterials")) {
		std::cout << "physicsMaterials field not present in KHR_physics_rigid_bodies" << std::endl;
		return true;
	}

	nl::json& materials = khr["physicsMaterials"];
	for (size_t i = 0; i < materials.size(); ++i) {
		nlohmann::json& material = materials[i];
		PhysicsMaterial phy_material;
		if (!material.contains("staticFriction")) {
			std::cout << "staticFriction field of physicsMaterials[" << i << "] not found" << std::endl;
			return false;
		}
		phy_material.friction = (float)material["staticFriction"];

		if (!material.contains("restitution")) {
			std::cout << "restitution field of physicsMaterials[" << i << "] not found" << std::endl;
			return false;
		}
		phy_material.restitution = (float)material["restitution"];

		g_materials.push_back(phy_material);
	}

	return true;
}

static bool parse_extensions() {
	nl::json ext = nl::json::parse(model.extensions_json_string);

	if (!load_all_implicit_shapes(ext)) {
		std::cout << "error loading implicit shapes" << std::endl;
		return false;
	}

	if (!load_all_physics_materials(ext)) {
		std::cout << "error loading physicsMaterials" << std::endl;
		return false;
	}

	return true;
}

static bool load_gltf(const std::string& filename, SceneGraph& scene) {
	bool ret = loader.LoadASCIIFromFile(&model, &err, &warn, filename);
	if (!err.empty()) {
		std::cout << "Error loading file " << filename << ": " << err << std::endl;
		return false;
	}
	if (!warn.empty()) {
		std::cout << "Warning loading file " << filename << ": " << warn << std::endl;
	}

	if (model.scenes.size() != 1) {
		std::cout << "Parse gltf file with 1 scene only" << std::endl;
		return false;
	}

	if (!parse_extensions()) {
		std::cout << "error parsing extensions" << std::endl;
		return false;
	}

	if (!label_all_physical_nodes()) {
		std::cout << "error labeling physical nodes" << std::endl;
		return false;
	}

	for (int node_id : model.scenes[0].nodes) {
		bool ret = load_node(node_id, -1, scene);
		if (!ret) {
			std::cout << "error loading node. node_id = " << node_id << std::endl;
			return false;
		}
	}

 	model = {};
	loader = {};
	err.clear();
	warn.clear();

	return true;
}

bool load_gltf(
	const std::string& filename,
	SceneGraph& graph,
	SceneGraphFlatRefs& graph_refs) {

	loader.SetStoreOriginalJSONForExtrasAndExtensions(true);

	if (!load_gltf(filename, graph)) {
		std::cout << "error loading gltf file " << filename << std::endl;
		g_shapes.clear();
		g_materials.clear();
		g_colliders.clear();
		g_phy_labels.clear();
		return false;
	}

	g_shapes.clear();
	g_materials.clear();
	g_colliders.clear();
	g_phy_labels.clear();

	for (uint32_t node_id = 0; node_id < graph.size(); ++node_id) {
		SceneNode& node = graph[node_id];
		for (uint32_t renderable_id = 0; renderable_id < node.renderables.size(); ++renderable_id) {
			if (graph[node_id].renderables[renderable_id].mesh) {
				graph_refs.push_back({ node_id, renderable_id });
			}
		}
	}

	return true;
}