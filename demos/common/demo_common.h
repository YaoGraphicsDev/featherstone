#include "gltf_parser.h"

#include "rigidworld.h"
#include "rigidworld_renderer.h"

inline std::shared_ptr<RigidWorldRenderer> g_renderer = nullptr;
inline std::shared_ptr<SPD::RigidWorld> g_world = nullptr;

glm::quat q(Eigen::Quaternionf q);

Eigen::Quaternionf EQ(glm::quat q);

glm::vec3 v3(Eigen::Vector3f v);

glm::mat3 m3(Eigen::Matrix3f M);

struct SignedAxisAnglef {
	Eigen::Vector3f axis; // unit
	float angle;          // signed, in (-pi, pi]
};
SignedAxisAnglef axis_angle(Eigen::Vector3f a, Eigen::Vector3f b, float eps = 1e-6f);

void init_renderer();

void init_world();

size_t add_node_to_renderer(const SceneNode& node);

std::shared_ptr<SPD::Shape> create_shape_from_collider(const Collider& collider);

std::shared_ptr<SPD::Shape> create_compound_shape_from_collider(const std::vector<Collider>& colliders);

std::shared_ptr<SPD::RigidBody> create_rigidbody_from_node(const SceneNode& node);

std::shared_ptr<SPD::ArticulatedBody> create_articulated_body(const SceneGraph& graph, const NodeGroup& art_group, const ArticulationTree& art_tree);

