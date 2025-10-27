#pragma once

#include "Eigen/Core"
#include "Eigen/Dense"

#include <vector>
#include <string>

struct ImplicitShape {
    enum class Type {
        Box,
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

// TODO: collision filter

struct MeshData {
    std::vector<Eigen::Vector3f> positions;
    std::vector<Eigen::Vector3f> normals;
    std::vector<Eigen::Vector2f> uv0;

    std::vector<uint16_t> indices;
};

struct RigidBodyDescriptor {
    std::string name = "";
    std::shared_ptr<ImplicitShape> implicit_shape = nullptr;
    std::shared_ptr<MeshData> mesh = nullptr;
    std::shared_ptr<PhysicsMaterial> material = nullptr;
    enum class DynamicType {
        Dynamic = 0,
        Static,
        // Kinematic TODO: kinematic
    };
    DynamicType dyn_type;
    Eigen::Vector3f translation;
    Eigen::Quaternionf rotation;
    float mass;
};

struct ArticulatedBodyDescriptor {
    // TODO
};
