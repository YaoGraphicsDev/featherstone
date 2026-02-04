#include "gltf_parser.h"

#define TINYGLTF_IMPLEMENTATION
#include "tiny_gltf.h"
#include "json.hpp"
#include "gltf_traits.h"

#include <iostream>
#include <unordered_set>
#include <queue>
#include <set>

namespace tg = tinygltf;
namespace nl = nlohmann;
using namespace Eigen;

tg::Model model;
tg::TinyGLTF loader;
std::string err;
std::string warn;
bool blender_export;

std::vector<ImplicitShape> g_shapes;
std::vector<PhysicsMaterial> g_materials;
std::vector<ArticulationLinkage::Joint> g_joints;
std::map<int, Collider> g_colliders;
std::map<int, ArticulationLinkage> g_id_link_map; // jointSpaceB id -- linkage

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

	JointProps,
	JointSpaceA,
	JointSpaceB,
};
std::vector<NodePhysicsLabel> g_phy_labels;

const std::string KHR_implicit_shapes = "KHR_implicit_shapes";
const std::string KHR_physics_rigid_bodies = "KHR_physics_rigid_bodies";
const std::string khr_physics_extra_props = "khr_physics_extra_props";
const std::string khr_physics_extra_constraint_props = "khr_physics_extra_constraint_props";

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

static void add_label(NodePhysicsLabel label, int id) {
	size_t size = g_phy_labels.size();
	g_phy_labels.resize(std::max((size_t)(id + 1), size));
	g_phy_labels[id] = label;
};

static bool label_physical_node(int node_id, bool as_child) {
	tg::Node& node = model.nodes[node_id];
	nl::json rb_props = std::move(extension_property(node, KHR_physics_rigid_bodies));
	nl::json extra_props = std::move(extra_property(node, khr_physics_extra_props));
	nl::json extra_constraint_props = std::move(extra_property(node, khr_physics_extra_constraint_props));

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

	// these names are generated by blender physics export add-on. There is no way user could modify these
	// so probably a reliable trait to distinguish node types
	bool named_joint_space_A = node.name == "jointSpaceA";
	bool named_joint_space_B = node.name == "jointSpaceB";
	bool has_space_A_fields = false;
	if (!rb_props.is_null() && rb_props.contains("joint") && rb_props["joint"].contains("connectedNode") && rb_props["joint"].contains("joint")) {
		has_space_A_fields = true;
	}

	for (int c_id : node.children) {
		// label children first
		if (!label_physical_node(c_id, true)) {
			std::cout << "error labeling node. node_id = " << c_id << std::endl;
			return false;
		}
	}

	// pick out joint-related children
	std::vector<NodePhysicsLabel> body_children;
	std::vector<NodePhysicsLabel> joint_children;
	std::for_each(node.children.begin(), node.children.end(), [&](int c) {
		if (g_phy_labels[c] == NodePhysicsLabel::JointProps ||
			g_phy_labels[c] == NodePhysicsLabel::JointSpaceA ||
			g_phy_labels[c] == NodePhysicsLabel::JointSpaceB) {
			joint_children.push_back(g_phy_labels[c]);
		}
		else {
			body_children.push_back(g_phy_labels[c]);
		}
	});

	auto check_articulation = [&]() {
		if (!joint_children.empty()) {
			if (g_phy_labels[node_id] == NodePhysicsLabel::StaticCompoundParent ||
				g_phy_labels[node_id] == NodePhysicsLabel::DynamicCompoundParent ||
				g_phy_labels[node_id] == NodePhysicsLabel::StaticTrivialImplicit ||
				g_phy_labels[node_id] == NodePhysicsLabel::StaticTrivialConvexHull ||
				g_phy_labels[node_id] == NodePhysicsLabel::DynamicTrivialImplicit ||
				g_phy_labels[node_id] == NodePhysicsLabel::DynamicTrivialConvexHull) {
				return true;
			}
			else {
				return false;
			}
		}
		else {
			return true;
		}
	};
	
	// label the node itself
	if (body_children.empty()) {
		if (!extra_constraint_props.is_null() && as_child)				add_label(NodePhysicsLabel::JointProps, node_id);
		else if (named_joint_space_A && has_space_A_fields && as_child) add_label(NodePhysicsLabel::JointSpaceA, node_id);
		else if (named_joint_space_B && as_child)						add_label(NodePhysicsLabel::JointSpaceB, node_id);
		else if (!non_renderable && rb_props.is_null())	add_label(NodePhysicsLabel::NonPhysical, node_id);
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
			std::cout << "cannot find suitable label for node. No children body. rb_props.is_null() = " << rb_props.is_null() << 
				", non_renderable = " << non_renderable << 
				", has_valid_mass = " << has_valid_mass <<
				", as_child = " << as_child << 
				", implicit = " << implicit << 
				", mesh_data_node_id = " << mesh_data_node_id <<
				", has contraint_props = " << !extra_constraint_props.is_null() <<
				", jointSpaceA = " << named_joint_space_A << 
				", jointSpaceB = " << named_joint_space_B <<
				", has_space_A_fields = " << has_space_A_fields <<
				std::endl;
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

		// check if this node serves as a valid articulated body
		if (!check_articulation()) {
			std::cout << "This type of node should not be linked into an articulated body, label = " << (int)g_phy_labels[node_id] << std::endl;
			return false;
		}

		return true;
	}

	// has children. NonPhysical / StaticCompoundParent / DynamicCompoundParent,
	NodePhysicsLabel first_label = body_children[0];
	bool children_legal = std::all_of(body_children.begin(), body_children.end(), [&](NodePhysicsLabel label) {
		if (first_label == NodePhysicsLabel::NonPhysical) {
			return label == first_label;
		}
		else if (first_label == NodePhysicsLabel::CompoundChildImplicit || first_label == NodePhysicsLabel::CompoundChildConvexHull) {
			return label == NodePhysicsLabel::CompoundChildImplicit ||
				label == NodePhysicsLabel::CompoundChildConvexHull;
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

	if (first_label == NodePhysicsLabel::NonPhysical) {
		if (!rb_props.is_null()) {
			std::cout << "If all children are non-physical, parent cannot be physical" << std::endl;
			return false;
		}
		
		add_label(NodePhysicsLabel::NonPhysical, node_id);
	}
	else if (first_label == NodePhysicsLabel::CompoundChildImplicit || first_label == NodePhysicsLabel::CompoundChildConvexHull) {
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

	// check if this node serves as a valid articulated body
	if (!check_articulation()) {
		std::cout << "This type of node should not be linked to into an articulated body, label = " << (int)g_phy_labels[node_id] << std::endl;
		return false;
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
		else if (g_phy_labels[c_id] == NodePhysicsLabel::JointProps ||
			g_phy_labels[c_id] == NodePhysicsLabel::JointSpaceA || 
			g_phy_labels[c_id] == NodePhysicsLabel::JointSpaceB) {
			// do nothing
		}
		else {
			assert(false); // should never happen
		}
	}
	return compound_collider;
}

static bool collect_articulation_links(const tg::Node& node, int this_id) {
	auto find_prop_node = [&](const tg::Node& node, Vector3f spaceA_translation, Quaternionf spaceA_rotation) {
		for (int c_id : node.children) {
			const tg::Node& child = model.nodes[c_id];
			if (g_phy_labels[c_id] != NodePhysicsLabel::JointProps) continue;
			Vector3f translation;
			Quaternionf rotation;
			if (!parse_rigid_transform(child, translation, rotation)) {
				std::cout << "error parsing joint props transforms. id = " << c_id << std::endl;
				return -1;
			}
			float ang_dist = spaceA_rotation.angularDistance(rotation);
			float lin_dist = (spaceA_translation - translation).norm();
			if (ang_dist < 1E-4 && lin_dist < 1E-4) {
				return c_id;
			}
		}
		assert(false);
		return 0;
	};

	// sort out articulation
	for (int c_id : node.children) {
		const tg::Node& child = model.nodes[c_id];
		if (g_phy_labels[c_id] == NodePhysicsLabel::JointSpaceA) {
			int spaceB_id = (int)extension_property(child, KHR_physics_rigid_bodies)["joint"]["connectedNode"];
			ArticulationLinkage& link = g_id_link_map[spaceB_id];
			link.bodyA_id = this_id;
			Vector3f translation;
			Quaternionf rotation;
			if (!parse_rigid_transform(child, translation, rotation)) {
				std::cout << "error parsing jointSpaceA transforms. id = " << c_id << std::endl;
				return nullptr;
			}
			link.bodyA_translation = translation;
			link.bodyA_rotation = rotation;

			int prop_id = find_prop_node(node, translation, rotation);
			if (prop_id < 0) {
				std::cout << "Cannot find a joint property node that is close enough to jointSpaceA. id = " << c_id << std::endl;
				return false;
			}
			link.name = model.nodes[prop_id].name;

			int joint_id = (int)extension_property(child, KHR_physics_rigid_bodies)["joint"]["joint"];
			assert(joint_id < g_joints.size());
			link.joint = g_joints[joint_id];

			if (link.joint.type == ArticulationLinkage::Joint::Revolute) {
				Matrix3f yz_correction;
				yz_correction <<
					1, 0, 0,
					0, 0, -1,
					0, 1, 0;
				link.bodyA_rotation = rotation * Quaternionf(yz_correction);
			}
			else if (link.joint.type == ArticulationLinkage::Joint::Prismatic || 
				link.joint.type == ArticulationLinkage::Joint::Cylindrical) {
				Matrix3f xz_correction;
				xz_correction <<
					0, 0, -1,
					0, 1, 0,
					1, 0, 0;
				link.bodyA_rotation = rotation * Quaternionf(xz_correction);
			}
			else if (link.joint.type == ArticulationLinkage::Joint::Spherical) {
				link.bodyA_rotation = rotation;
			}
			else {
				std::cout << "joint type not supported. joint_id = " << joint_id << ", id = " << c_id << std::endl;
				assert(false);
				return false;
			}
			
			if (extension_property(child, KHR_physics_rigid_bodies)["joint"].contains("enableCollision")) {
				link.enable_collision = (bool)extension_property(child, KHR_physics_rigid_bodies)["joint"]["enableCollision"];
			}
			else {
				link.enable_collision = false;
			}
		}
		if (g_phy_labels[c_id] == NodePhysicsLabel::JointSpaceB) {
			g_id_link_map[c_id].bodyB_id = this_id;
			// TODO: check translation and rotation from space B's perspective
		}
	}
	return true;
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
	else {
		// should never happen. jointSpaceA jointSpaceB and jointProp only appears in children nodes
		assert(false);
	}
	
	if (!collect_articulation_links(node, this_id)) {
		std::cout << "error collecting articulation links. id = " << node_id << std::endl;
		return false;
	}

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

static bool parse_joint_dofs(nlohmann::json& joint, int& dof_flags, ArticulationLinkage::Joint::DofLimits& dof_limits) {
	if (!joint.contains("limits")) {
		// This is a 6 dof joint
		dof_flags = 0b111111; // all dofs free
		return true;
	}

	nlohmann::json& limits = joint["limits"];
	dof_flags = 0b111111; // all dofs free
	for (int i = 0; i < limits.size(); ++i) {
		// traverse limits, mark which axes are fixed
		nlohmann::json& limit = limits[i];
		if (!limit.contains("min") || !limit.contains("max")) {
			std::cout << "min/max field missing" << std::endl;
			return false;
		}

		float max = (float)limit["max"];
		float min = (float)limit["min"];
		if (std::abs(max) < std::numeric_limits<float>::epsilon() &&
			std::abs(min) < std::numeric_limits<float>::epsilon()) {
			// locked axis
			if (limit.contains("linearAxes")) {
				for (int i : limit["linearAxes"]) {
					dof_flags &= ~(1u << (2 - i));
				}
			}
			if (limit.contains("angularAxes")) {
				for (int i : limit["angularAxes"]) {
					dof_flags &= ~(1u << (5 - i));
				}
			}
		}
		else {
			// free axis with limits
			std::cout << "Cannot process free axes with non-zero limits[" << i << "]" << std::endl;
			return false;
		}
	}

	return true;
}

static bool parse_joint_spring_configs(
	nlohmann::json& joint,
	ArticulationLinkage::Joint::Type type,
	ArticulationLinkage::Joint::DofSprings& dof_springs) {

	if (!joint.contains("drives")) {
		return true;
	}
	dof_springs.clear();

	// parse spring configurations if applicable
	nlohmann::json& drives = joint["drives"];
	if (!drives.is_array()) {
		std::cout << "drives field should be an array" << std::endl;
	}
	int drives_size = drives.size();

	const int x_axis = 0;
	const int y_axis = 1;
	const int z_axis = 2;
	const std::string linear_axis = "linear";
	const std::string angular_axis = "angular";
	auto load_dof_spring = [&](int drive_id, const std::string& drive_type, int axis) {
		nlohmann::json drive = drives[drive_id];
		if (!drive.contains("type") || drive["type"] != drive_type) {
			std::cout << "illegal drive type = " << drive["type"] << std::endl;
			return false;
		}
		if (!drive.contains("axis") || axis != int(drive["axis"])) {
			std::cout << "illegal drive axis = " << int(drive["axis"]) << std::endl;
			return false;
		}
		if (!drive.contains("stiffness")) {
			std::cout << "drive does not contain stiffness field" << std::endl;
			return false;
		}
		if (!drive.contains("damping")) {
			std::cout << "drive does not contain damping field" << std::endl;
			return false;
		}
		dof_springs.push_back({ (float)drive["stiffness"], (float)drive["damping"] });
		return true;
	};

	if (type == ArticulationLinkage::Joint::Prismatic) {
		if (drives_size != 1) {
			std::cout << "illegal drive size = " << drives_size << std::endl;
			return false;
		}
		if (!load_dof_spring(0, linear_axis, x_axis)) {
			std::cout << "load drive 0 fail, joint type = Prismatic" << std::endl;
			return false;
		}
	}
	else if (type == ArticulationLinkage::Joint::Revolute) {
		if (drives_size != 1) {
			std::cout << "illegal drive size = " << drives_size << std::endl;
			return false;
		}
		if (!load_dof_spring(0, angular_axis, y_axis)) {
			std::cout << "load drive 0 fail, joint type = Revolute" << std::endl;
			return false;
		}
	}
	else {
		std::cout << "spring configuration does not support this type of joint" << std::endl;
		assert(false);
		return false;
	}

	return true;
}

static bool load_all_physics_joints(nlohmann::json& ext) {
	if (!ext.contains(KHR_physics_rigid_bodies)) {
		std::cout << "KHR_physics_rigid_bodies not present" << std::endl;
		return true;
	}
	
	nl::json& khr = ext[KHR_physics_rigid_bodies];
	if (!khr.contains("physicsJoints")) {
		std::cout << "physicsJoints field not present in KHR_physics_rigid_bodies" << std::endl;
		return true;
	}

	nl::json& joints = khr["physicsJoints"];
	for (int i = 0; i < joints.size(); ++i) {
		nlohmann::json& joint = joints[i];

		int dof_flags;
		ArticulationLinkage::Joint::DofLimits dof_limits;
		if (!parse_joint_dofs(joint, dof_flags, dof_limits)) {
			std::cout << "failed to parse dof data of physicsJoints[" << i << "]" << std::endl;
			return false;
		}

		ArticulationLinkage::Joint phy_joint;
		// blender convention
		const int ang_x_free = 0b100000;
		const int linear_x_free = 0b000100;
		const int ang_y_free = 0b010000;
		const int linear_free = 0b000111;
		const int ang_free = 0b111000;
		if (dof_flags == linear_x_free) {
			phy_joint.type = ArticulationLinkage::Joint::Prismatic;
		}
		else if (dof_flags == ang_y_free) {
			phy_joint.type = ArticulationLinkage::Joint::Revolute;
		}
		else if (dof_flags == (ang_x_free | linear_x_free)) {
			phy_joint.type = ArticulationLinkage::Joint::Cylindrical;
		}
		else if (dof_flags == (ang_free)) {
			phy_joint.type = ArticulationLinkage::Joint::Spherical;
		}
		else {
			std::cout << "invalid dof flag = " << dof_flags << ", physicsJoints[" << i << "]" << std::endl;
			assert(false);
			return false;
		}
		phy_joint.dof_limits = dof_limits;

		if (!parse_joint_spring_configs(joint, phy_joint.type, phy_joint.dof_springs)) {
			std::cout << "failed to parse drive of physicsJoints[" << i << "]" << std::endl;
			return false;
		}

		g_joints.push_back(phy_joint);
	}
	return true;
}

static bool parse_extensions() {
	if (model.extensions_json_string.empty()) {
		return true;
	}

	nl::json ext = nl::json::parse(model.extensions_json_string);

	if (!load_all_implicit_shapes(ext)) {
		std::cout << "error loading implicit shapes" << std::endl;
		return false;
	}

	if (!load_all_physics_materials(ext)) {
		std::cout << "error loading physicsMaterials" << std::endl;
		return false;
	}

	if (!load_all_physics_joints(ext)) {
		std::cout << "error loading physicsJoints" << std::endl;
	}

	return true;
}

static bool build_and_flatten_articulation_links(ArticulationForest& forest)
{
	std::vector<ArticulationLinkage> links;
	for (auto& p : g_id_link_map) {
		links.push_back(p.second);
	}

	forest.clear();
	if (links.empty()) {
		return true;
	}

	// parent body -> list of edge indices from that parent
	std::unordered_map<int, std::vector<int>> adj;
	adj.reserve(links.size());

	std::unordered_set<int> parents, children;
	parents.reserve(links.size());
	children.reserve(links.size());

	for (int ei = 0; ei < (int)links.size(); ++ei) {
		const auto& e = links[ei];
		if (e.bodyA_id < 0 || e.bodyB_id < 0) {
			std::cout << "invalid body AB id in ArticulationLinkage. bodyA_id = " << e.bodyA_id << ", bodyB_id = " << e.bodyB_id << std::endl;
			return false;
		}
		assert(e.bodyA_id >= 0 && e.bodyB_id >= 0);
		adj[e.bodyA_id].push_back(ei);
		parents.insert(e.bodyA_id);
		children.insert(e.bodyB_id);
	}

	// Roots: parent but never child.
	std::vector<int> roots;
	roots.reserve(parents.size());
	for (int p : parents) {
		if (!children.count(p)) roots.push_back(p);
	}

	// Track which edges have been emitted.
	std::vector<uint8_t> edge_used(links.size(), 0);

	auto bfs_from_start = [&](int start_body) -> std::vector<ArticulationLinkage> {
		std::vector<ArticulationLinkage> flat;
		std::queue<int> q;
		std::unordered_set<int> visited_bodies; // per-component
		visited_bodies.reserve(64);

		visited_bodies.insert(start_body);
		q.push(start_body);

		while (!q.empty()) {
			int u = q.front();
			q.pop();

			auto it = adj.find(u);
			if (it == adj.end()) continue;

			for (int ei : it->second) {
				if (edge_used[ei]) continue;

				edge_used[ei] = 1;
				flat.push_back(links[ei]); // copy into output

				int v = links[ei].bodyB_id;
				if (!visited_bodies.count(v)) {
					visited_bodies.insert(v);
					q.push(v);
				}
			}
		}
		return flat;
	};

	// BFS all rooted components first.
	for (int r : roots) {
		auto comp = bfs_from_start(r);
		if (!comp.empty())
			forest.push_back(std::move(comp));
	}

	// Group any remaining edges (cycles / components with no root).
	for (;;) {
		int start = -1;
		for (const auto& [p, edges] : adj) {
			for (int ei : edges) {
				if (!edge_used[ei]) { start = p; break; }
			}
			if (start != -1) break;
		}
		if (start == -1) break;

		auto comp = bfs_from_start(start);
		if (!comp.empty())
			forest.push_back(std::move(comp));
	}

	// Collect anything still unused (should normally be none).
	for (int ei = 0; ei < (int)links.size(); ++ei) {
		if (!edge_used[ei]) {
			// should never be unused links. As links are build upon bodies in the first place
			assert(false);
		}
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

bool load_gltf(const std::string& filename, Scene& scene, GLTFParseOption opt) {
	loader.SetStoreOriginalJSONForExtrasAndExtensions(true);
	if (opt == GLTFParseOption::BlenderExport) {
		blender_export = true;
	}
	else {
		blender_export = false;
	}

	if (!load_gltf(filename, scene.graph)) {
		std::cout << "error loading gltf file " << filename << std::endl;
		g_shapes.clear();
		g_materials.clear();
		g_joints.clear();
		g_colliders.clear();
		g_id_link_map.clear();
		g_phy_labels.clear();
		return false;
	}

	// build the tree of articulation
	scene.art_forest.clear();
	if (!build_and_flatten_articulation_links(scene.art_forest)) {
		std::cout << "Error bulding articulation forest" << std::endl;
		return false;
	}

	// group up articulted bodies
	std::set<int> articulated_set;
	for (auto& tree : scene.art_forest) {
		scene.art_groups.emplace_back();
		for (auto& link : tree) {
			if (articulated_set.insert(link.bodyA_id).second) {
				scene.art_groups.back().push_back(link.bodyA_id);
			}
			if (articulated_set.insert(link.bodyB_id).second) {
				scene.art_groups.back().push_back(link.bodyB_id);
			}
		}
	}

	// scene.art_forest point to scene.graph at this point
	// redirect it so that they point to the indices scene.art_groups
	for (int t = 0; t < scene.art_forest.size(); ++t) {
		ArticulationTree& tree = scene.art_forest[t];
		const NodeGroup& group = scene.art_groups[t];
		for (auto& link : tree) {
			ArticulationLinkage redirected_link = link;
			for (auto iter = group.begin(); iter < group.end(); ++iter) {
				if (*iter == link.bodyA_id) {
					redirected_link.bodyA_id = iter - group.begin();
				}
				if (*iter == link.bodyB_id) {
					redirected_link.bodyB_id = iter - group.begin();
				}
			}
			link = redirected_link;
		}
	}

	// group up individual rigidbodies and non physicals
	for (int id = 0; id < scene.graph.size(); ++id) {
		if (articulated_set.find(id) != articulated_set.end()) {
			// articulated
			continue;
		}

		if (!scene.graph[id].physical) {
			// non physical
			scene.non_physical_group.push_back(id);
		}
		else {
			scene.rigidbody_group.push_back(id);
		}
	}

	g_shapes.clear();
	g_materials.clear();
	g_joints.clear();
	g_colliders.clear();
	g_id_link_map.clear();
	g_phy_labels.clear();


	return true;
}