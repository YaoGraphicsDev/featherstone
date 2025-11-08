#pragma once

#include "gltf_parser_types.h"

#include <vector>
#include <string>
#include <memory>

#include "Eigen/Dense"
#include "Eigen/Core"

struct MeshData {
    std::vector<PVec3f> positions;
    std::vector<PVec3f> normals;
    std::vector<PVec2f> uv0;

    std::vector<uint16_t> indices;
};

struct Renderable {
    std::shared_ptr<MeshData> mesh;
};

struct ImplicitShape {
    enum class Type {
        Box = 0,
        Sphere,
        Cylinder,
        Capsule
    };
    Type type;
    Eigen::Vector3f half_dims;
};

struct PhysicsMaterial {
    float friction;
    float restitution;
};

struct Collider {
    std::string name = "";
    std::shared_ptr<ImplicitShape> implicit_shape = nullptr;
    Eigen::Vector3f implicit_shape_translation = Eigen::Vector3f::Zero();
    Eigen::Quaternionf implicit_shape_rotation = Eigen::Quaternionf::Identity();

    std::shared_ptr<MeshData> convex_hull = nullptr;
    std::shared_ptr<PhysicsMaterial> material = nullptr;
};

struct Physical {
    enum class DynamicType {
        Dynamic = 0,
        Static,
        // Kinematic TODO: kinematic
    };
    DynamicType dyn_type;
    float mass;
    std::vector<Collider> colliders;
};

struct SceneNode {
    int parent = -1;
    std::string name;
    Eigen::Vector3f local_translation = Eigen::Vector3f::Zero();
    Eigen::Quaternionf local_rotation = Eigen::Quaternionf::Identity();
    Eigen::Vector3f world_translation = Eigen::Vector3f::Zero();
    Eigen::Quaternionf world_rotation = Eigen::Quaternionf::Identity();
    std::vector<Renderable> renderables;
    std::shared_ptr<Physical> physical;
};

typedef std::vector<SceneNode> SceneGraph;

struct ObjectRef {
    uint32_t node_id;
    uint32_t renderable_id;
};
typedef std::vector<ObjectRef> SceneGraphFlatRefs;

