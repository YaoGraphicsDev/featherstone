#include "gltf_physics_body_parser.h"

#define TINYGLTF_IMPLEMENTATION
#include "tiny_gltf.h"
#include "json.hpp"
#include "gltf_traits.h"
#include "spshapes.hpp"

#include <iostream>

namespace tg = tinygltf;
namespace nl = nlohmann;

tg::Model model;
tg::TinyGLTF loader;
std::string err;
std::string warn;

std::vector<ImplicitShape> g_shapes;
std::vector<PhysicsMaterial> g_materials;
std::vector<std::shared_ptr<MeshData>> g_meshes;

// TODO: collision filters

const std::string KHR_implicit_shapes = "KHR_implicit_shapes";
const std::string KHR_physics_rigid_bodies = "KHR_physics_rigid_bodies";

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

static bool parse_rigid_transform(const tg::Node& node, Eigen::Vector3f& trans, Eigen::Quaternionf& rot, Eigen::Vector3f& scale) {
	if (!node.matrix.empty()) {
		std::cout << "error parsing rigid transform. Matrix not supported." << std::endl;
		return false;
	}

	trans = Eigen::Vector3f::Zero();
	rot = Eigen::Quaternionf::Identity();
	scale = Eigen::Vector3f::Constant(1.0f);

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
	}

	if (!node.scale.empty()) {
		scale.x() = (float)node.scale[0];
		scale.y() = (float)node.scale[1];
		scale.z() = (float)node.scale[2];
	}

	return true;
}

static bool load_all_implicit_shapes(nlohmann::json& ext) {
	if (!ext.contains(KHR_implicit_shapes)) {
		std::cout << "KHR_implicit_shapes not present" << std::endl;
		return true;
	}

	nlohmann::json& khr = ext[KHR_implicit_shapes];
	if (!khr.contains("shapes")) {
		std::cout << "shapes field not present in KHR_implicit_shapes" << std::endl;
		return true;
	}

	nlohmann::json& shapes = khr["shapes"];
	for (size_t i = 0; i < shapes.size(); ++i) {
		nlohmann::json& shape = shapes[i];
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

		// Parse box size
		if (imp_shape.type == ImplicitShape::Type::Box) {
			if (!shape.contains("box")) {
				std::cout << "Cannot find box parameters" << std::endl;
				return false;
			}

			nlohmann::json& box = shape["box"];
			if (!box.contains("size")) {
				std::cout << "Cannot find box size" << std::endl;
				return false;
			}

			nlohmann::json& size = box["size"];
			if (size.size() != 3) {
				std::cout << "Box size should have x,y,z values" << std::endl;
				return false;
			}

			float x = (float)size[0];
			float y = (float)size[1];
			float z = (float)size[2];
			imp_shape.half_dims = Eigen::Vector3f(x, y, z) * 0.5f;
		}
		else {
			std::cout << "Any other implicit shape other than box is not supported yet" << std::endl;
			assert(false);
			return false;
		}
		g_shapes.push_back(imp_shape);
	}

	return true;
}

static bool load_all_materials(nlohmann::json& ext) {
	if (!ext.contains(KHR_physics_rigid_bodies)) {
		std::cout << "KHR_physics_rigid_bodies not present" << std::endl;
		return true;
	}

	nlohmann::json& khr = ext[KHR_physics_rigid_bodies];
	if (!khr.contains("physicsMaterials")) {
		std::cout << "physicsMaterials field not present in KHR_physics_rigid_bodies" << std::endl;
		return true;
	}

	nlohmann::json& materials = khr["physicsMaterials"];
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
	nlohmann::json ext = nlohmann::json::parse(model.extensions_json_string);

	if (!load_all_implicit_shapes(ext)) {
		std::cout << "error loading implicit shapes" << std::endl;
		return false;
	}

	if (!load_all_materials(ext)) {
		std::cout << "error loading physicsMaterials" << std::endl;
		return false;
	}

	return true;
}

struct NodePhysicsType {
	enum class NodeType {
		NonPhysical = 0,
		RigidBody,
		// TODO: differentiate joint nodes
		// JointSpaceA,
		// JointSpaceB,
	};
	NodeType node_type = NodeType::NonPhysical;
	RigidBodyDescriptor::DynamicType dyn_type = RigidBodyDescriptor::DynamicType::Static;
};
static NodePhysicsType node_physics_type(const tg::Node& node) {
	if (node.extensions_json_string.empty()) {
		// TODO:
		assert(false);
	}

	nl::json ext = nl::json::parse(node.extensions_json_string);
	
	if (!ext.contains(KHR_physics_rigid_bodies)) {
		return { NodePhysicsType::NodeType::NonPhysical };
	}

	nl::json& khr = ext[KHR_physics_rigid_bodies];
	if (khr.contains("collider")) {
		if (khr.contains("motion") && khr["motion"].contains("mass")) {
			return { NodePhysicsType::NodeType::RigidBody, RigidBodyDescriptor::DynamicType::Dynamic };
		}
		else {
			return { NodePhysicsType::NodeType::RigidBody, RigidBodyDescriptor::DynamicType::Static };
		}
	} 
	else {
		// TODO
		assert(false);
		return { NodePhysicsType::NodeType::NonPhysical };
	}
}

static std::shared_ptr<RigidBodyDescriptor> build_rigid_body(const tg::Node& node, RigidBodyDescriptor::DynamicType dyn_type) {
	std::shared_ptr<RigidBodyDescriptor> desc = std::make_shared<RigidBodyDescriptor>();

	desc->name = node.name;
	desc->dyn_type = dyn_type;
	Eigen::Vector3f translation;
	Eigen::Quaternionf rotation;
	Eigen::Vector3f scale;
	if (!parse_rigid_transform(node, translation, rotation, scale)) {
		std::cout << "error parsing rigid transforms." << std::endl;
		return nullptr;
	}
	desc->translation = translation;
	desc->rotation = rotation;

	nl::json ext = nl::json::parse(node.extensions_json_string);
	nl::json& khr = ext[KHR_physics_rigid_bodies];
	nl::json& collider = khr["collider"];
	if (!collider.contains("geometry")) {
		std::cout << "collider.geometry field not found" << std::endl;
		return nullptr;
	}
	nl::json& geometry = collider["geometry"];
	if (geometry.contains("shape")) {
		// implicit shape as collider
		int shape_id = geometry["shape"];
		desc->implicit_shape = std::make_shared<ImplicitShape>(g_shapes[shape_id]);
		desc->implicit_shape->half_dims = desc->implicit_shape->half_dims.cwiseProduct(scale);
	}
	else {
		// mesh/convex hull collider
		assert(false);
		return nullptr;
	}

	if (!collider.contains("physicsMaterial")) {
		std::cout << "collider.physicsMaterial field not found" << std::endl;
		return nullptr;
	}
	nl::json& material_id = collider["physicsMaterial"];
	desc->material = std::make_shared<PhysicsMaterial>(g_materials[material_id]);

	if (dyn_type == RigidBodyDescriptor::DynamicType::Dynamic) {
		nl::json& motion = khr["motion"];
		desc->mass = (float)motion["mass"];
	}
	else if (dyn_type == RigidBodyDescriptor::DynamicType::Static) {
		desc->mass = 0.0f;
	}
	else {
		assert(false);
		return nullptr;
	}

	return desc;
}

static bool load_all_nodes(std::vector<RigidBodyDescriptor>& rigids) {
	// ignore scene hierarchy
	for (size_t i = 0; i < model.nodes.size(); ++i) {
		const tg::Node& node = model.nodes[i];

		NodePhysicsType phy_type = node_physics_type(node);
		if (phy_type.node_type == NodePhysicsType::NodeType::RigidBody) {
			std::shared_ptr<RigidBodyDescriptor> desc = build_rigid_body(node, phy_type.dyn_type);
			if (!desc) {
				std::cout << "error building rigid body. id = " << i << std::endl;
				continue;
			}
			rigids.push_back(*desc);
		}
		else {
			// TODO: build other types of node
			assert(false);
			return false;
		}
	}

	return true;
}

bool load_gltf_physics_world(
	const std::string& filename,
	std::vector<RigidBodyDescriptor>& rigids,
	std::vector<ArticulatedBodyDescriptor>& articulates) {

	loader.SetStoreOriginalJSONForExtrasAndExtensions(true);

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
		std::cout << "error loading extensions" << std::endl;
		g_shapes.clear();
		g_materials.clear();
		g_meshes.clear();
		return false;
	}

	if (!load_all_nodes(rigids)) {
		std::cout << "error loading nodes" << std::endl;
		g_shapes.clear();
		g_materials.clear();
		g_meshes.clear();
		return false;
	}

	g_shapes.clear();
	g_materials.clear();
	g_meshes.clear();

	// TODO: unload tiny gltf

	return true;
}