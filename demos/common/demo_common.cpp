#include "demo_common.h"

#include <iostream>

using namespace Eigen;
using namespace SPD;

glm::quat q(Quaternionf q) {
	return glm::quat(q.w(), q.x(), q.y(), q.z());
}

Quaternionf EQ(glm::quat q) {
	return Quaternionf(q.w, q.x, q.y, q.z);
}

glm::vec3 v3(Vector3f v) {
	return glm::vec3(v.x(), v.y(), v.z());
}

glm::mat3 m3(Matrix3f M) {
	glm::mat3 m;
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			m[j][i] = M(i, j);
		}
	}
	return m;
}


// signed axis-angle from a to b (float version), axis is oriented to be on the same side as a.cross(b).
SignedAxisAnglef axis_angle(Eigen::Vector3f a, Eigen::Vector3f b, float eps) {
	SignedAxisAnglef out;
	out.axis.setZero();
	out.angle = 0.0f;

	float na = a.norm();
	float nb = b.norm();
	if (na <= eps || nb <= eps)
		return out; // undefined, return zero rotation

	Eigen::Vector3f an = a / na;
	Eigen::Vector3f bn = b / nb;

	// Reference hemisphere: a cross b
	Eigen::Vector3f c = an.cross(bn);
	const float cn = c.norm();

	// Shortest rotation quaternion
	Eigen::Quaternionf q;
	q.setFromTwoVectors(an, bn);
	q.normalize();

	// Unsigned angle in [0, pi]
	float w = std::clamp(q.w(), -1.0f, 1.0f);
	float vnorm = q.vec().norm();
	float angle = 2.0f * std::atan2(vnorm, w);

	// Axis from quaternion (undefined if angle is almost 0)
	Eigen::Vector3f axis = (vnorm > eps) ? Vector3f(q.vec() / vnorm) : Vector3f::Zero();

	out.axis = axis;
	out.angle = angle;

	return out;
}


void init_renderer() {
	RigidWorldRenderer::Config renderer_config;
	renderer_config.world_aabb = { glm::vec3(-10.0f, -5.0f, -10.0f), glm::vec3(10.0f, 8.0f, 10.0f) };
	renderer_config.light_dir = { -0.5f, -1.0f, -0.4f };
	renderer_config.cam.position = { 1.55943, 2.80221, 4.21298 };
	renderer_config.cam.target = { 0.0f, 0.0f, 0.0f };
	renderer_config.app_name = std::string(DEMO_NAME);
	g_renderer = std::make_shared<RigidWorldRenderer>(renderer_config);
}

void init_world() {
	Eigen::Vector3f gravity = Eigen::Vector3f(0.0f, -10.0f, 0.0f);
	g_world = std::make_shared<RigidWorld>(gravity);
}

size_t add_node_to_renderer(const SceneNode& node) {
	assert(node.renderables.size() == 1);

	// colliders
	std::vector<Mesh> colliders;
	if (node.physical) {
		for (const Collider& c : node.physical->colliders) {
			if (c.implicit_shape) {
				if (c.implicit_shape->type == ImplicitShape::Type::Box)
					colliders.push_back(std::move(g_renderer->build_mesh(
						RigidWorldRenderer::Shape::Cuboid,
						v3(c.implicit_shape->half_dims),
						v3(c.implicit_shape_translation),
						q(c.implicit_shape_rotation))));
				else if (c.implicit_shape->type == ImplicitShape::Type::Cylinder)
					colliders.push_back(std::move(g_renderer->build_mesh(
						RigidWorldRenderer::Shape::Cylinder,
						v3(c.implicit_shape->half_dims),
						v3(c.implicit_shape_translation),
						q(c.implicit_shape_rotation))));
				else if (c.implicit_shape->type == ImplicitShape::Type::Sphere)
					colliders.push_back(std::move(g_renderer->build_mesh(
						RigidWorldRenderer::Shape::Sphere,
						v3(c.implicit_shape->half_dims),
						v3(c.implicit_shape_translation),
						q(c.implicit_shape_rotation))));
				else
					assert(false); // TODO: other shapes not supported
			}
			else if (c.convex_hull) {
				std::shared_ptr<MeshData> c_mesh = c.convex_hull;
				colliders.push_back(std::move(g_renderer->build_mesh(
					std::vector<glm::vec3>(
						reinterpret_cast<const glm::vec3*>(c_mesh->positions.data()),
						reinterpret_cast<const glm::vec3*>(c_mesh->positions.data() + c_mesh->positions.size())
					),
					std::vector<glm::vec3>(),
					std::vector<glm::vec2>(),
					c_mesh->indices)));
			}
			else {
				assert(false);
			}
		}
	}

	// renderable
	std::shared_ptr<MeshData> r_mesh = node.renderables[0].mesh;
	Mesh renderable = std::move(g_renderer->build_mesh(
		std::vector<glm::vec3>(
			reinterpret_cast<const glm::vec3*>(r_mesh->positions.data()),
			reinterpret_cast<const glm::vec3*>(r_mesh->positions.data() + r_mesh->positions.size())
		),
		std::vector<glm::vec3>(
			reinterpret_cast<const glm::vec3*>(r_mesh->normals.data()),
			reinterpret_cast<const glm::vec3*>(r_mesh->normals.data() + r_mesh->normals.size())
		),
		std::vector<glm::vec2>(
			reinterpret_cast<const glm::vec2*>(r_mesh->uv0.data()),
			reinterpret_cast<const glm::vec2*>(r_mesh->uv0.data() + r_mesh->uv0.size())
		),
		r_mesh->indices));

	size_t key = g_renderer->add_body(colliders, renderable, v3(node.world_translation), q(node.world_rotation));
	return key;
}

std::shared_ptr<Shape> create_shape_from_collider(const Collider& collider) {
	std::shared_ptr<Shape> shape;
	if (collider.implicit_shape) {
		ImplicitShape::Type shape_type = collider.implicit_shape->type;
		if (shape_type == ImplicitShape::Type::Box) {
			shape = std::make_shared<Cuboid>(collider.implicit_shape->half_dims);
		}
		else if (shape_type == ImplicitShape::Type::Sphere) {
			shape = std::make_shared<Sphere>(collider.implicit_shape->half_dims.x());
		}
		else if (shape_type == ImplicitShape::Type::Cylinder) {
			shape = std::make_shared<Cylinder>(collider.implicit_shape->half_dims.x(), collider.implicit_shape->half_dims.y() * 2.0f);
		}
		else {
			assert(false);
		}
	}
	else if (collider.convex_hull) {
		shape = std::make_shared<ConvexHull>(
			reinterpret_cast<const float*>(collider.convex_hull->positions.data()),
			collider.convex_hull->positions.size(),
			collider.convex_hull->indices.data(),
			collider.convex_hull->indices.size());
	}
	else {
		std::cout << "Invalid Collider" << std::endl;
		shape = nullptr;
	}

	return shape;
}

std::shared_ptr<Shape> create_compound_shape_from_collider(const std::vector<Collider>& colliders) {
	std::vector<CompoundShape::Composition> comps; 
	for (const Collider& c : colliders) {
		comps.emplace_back();
 		comps.back().shape = create_shape_from_collider(c);
		if (c.implicit_shape) {
			comps.back().rotation = c.implicit_shape_rotation;
			comps.back().translation = c.implicit_shape_translation;
		}
		else if (c.convex_hull) {
			comps.back().rotation = Quaternionf::Identity();
			comps.back().translation = Vector3f::Zero();
		}
		else {
			assert(false);
		}
	}
	return std::make_shared<CompoundShape>(comps);
}

std::shared_ptr<RigidBody> create_rigidbody_from_node(const SceneNode& node) {
	RigidBody::Config config;

	if (!node.physical) {
		return nullptr;
	}

	config.name = node.name;

	//if (node.physical->colliders.size() == 1) {
	//	config.shape = create_shape_from_collider(node.physical->colliders[0]);
	//}
	//else {
		config.shape = create_compound_shape_from_collider(node.physical->colliders);
	// }

	config.rotation = node.world_rotation;
	config.translation = node.world_translation;
	config.type = static_cast<RigidBody::DynamicType>(node.physical->dyn_type);
	config.density = node.physical->mass / config.shape->vol;
	// averages resitution and friction
	float avg_restitution = 0.0f;
	float avg_friction = 0.0f;
	std::for_each(node.physical->colliders.begin(), node.physical->colliders.end(), [&](Collider& c) {
		avg_restitution += c.material->restitution;
		avg_friction += c.material->friction;
	});
	avg_restitution /= (float)node.physical->colliders.size();
	avg_friction /= (float)node.physical->colliders.size();
	config.restitution_coeff = avg_restitution;
	config.friction_coeff = avg_friction;

	return std::make_shared<RigidBody>(config);
}

std::shared_ptr<ArticulatedBody> create_articulated_body(const SceneGraph& graph, const NodeGroup& art_group, const ArticulationTree& art_tree) {
	assert(!art_tree.empty() && !art_group.empty());
	int base_id = art_group[0];
	if (graph[base_id].physical->dyn_type != Physical::DynamicType::Static) {
		std::cout << "base body has to be static" << std::endl;
		return {};
	}

	// TODO: set restitution and friction
	std::shared_ptr<RigidBody> base = create_rigidbody_from_node(graph[base_id]);
	std::shared_ptr<ArticulatedBody> art = std::make_shared<ArticulatedBody>(*base);

	for (int id : art_group) {
		if (id == base_id) {
			continue;
		}
		std::shared_ptr<RigidBody> rb = create_rigidbody_from_node(graph[id]);
		art->add_body(*rb);
	}

	for (const ArticulationLinkage& link : art_tree) {
		std::vector<ArticulatedBody::SpringParam> spring_params;
		for (auto& dof_spring : link.joint.dof_springs) {
			spring_params.push_back({dof_spring.stiffness, dof_spring.damping});
		}
		art->add_joint(
			link.name,
			(ArticulatedBody::JointType)link.joint.type,
			link.bodyA_id,
			link.bodyB_id,
			link.bodyA_rotation.toRotationMatrix(),
			link.bodyA_translation, !link.enable_collision, spring_params);
	}

	return art;
}
