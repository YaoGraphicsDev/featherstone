#include "gltf_parser.h"

#include "rigidworld.h"
#include "rigidworld_renderer.h"

#include <iostream>

using namespace Eigen;
using namespace SPD;

std::shared_ptr<RigidWorldRenderer> renderer = nullptr;
std::shared_ptr<RigidWorld> world = nullptr;

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

size_t add_node_to_renderer(const SceneNode& node) {
	assert(node.renderables.size() == 1);

	// colliders
	std::vector<Mesh> colliders;
	if (node.physical) {
		for (const Collider& c : node.physical->colliders) {
			if (c.implicit_shape) {
				if (c.implicit_shape->type == ImplicitShape::Type::Box)
					colliders.push_back(std::move(renderer->build_mesh(
						RigidWorldRenderer::Shape::Cuboid,
						v3(c.implicit_shape->half_dims),
						v3(c.implicit_shape_translation),
						q(c.implicit_shape_rotation))));
				else if (c.implicit_shape->type == ImplicitShape::Type::Cylinder)
					colliders.push_back(std::move(renderer->build_mesh(
						RigidWorldRenderer::Shape::Cylinder,
						v3(c.implicit_shape->half_dims),
						v3(c.implicit_shape_translation),
						q(c.implicit_shape_rotation))));
				else if (c.implicit_shape->type == ImplicitShape::Type::Sphere)
					colliders.push_back(std::move(renderer->build_mesh(
						RigidWorldRenderer::Shape::Sphere,
						v3(c.implicit_shape->half_dims),
						v3(c.implicit_shape_translation),
						q(c.implicit_shape_rotation))));
				else
					assert(false); // TODO: other shapes not supported
			}
			else if (c.convex_hull) {
				std::shared_ptr<MeshData> c_mesh = c.convex_hull;
				colliders.push_back(std::move(renderer->build_mesh(
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
	Mesh renderable = std::move(renderer->build_mesh(
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

	size_t key = renderer->add_body(colliders, renderable, v3(node.world_translation), q(node.world_rotation));
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
	
	if (node.physical->colliders.size() == 1) {
		config.shape = create_shape_from_collider(node.physical->colliders[0]);
	}
	else {
		config.shape = create_compound_shape_from_collider(node.physical->colliders);
	}
	
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

int main() {
	SceneGraph graph;
	SceneGraphFlatRefs refs;
	if (!load_gltf(std::string(SCENES_DIR) + "wrecking_ball/wrecking_ball.gltf", graph, refs)) {
		std::cout << "error loading gltf resource" << std::endl;
		return 0;
	}
	
	RigidWorldRenderer::Config renderer_config;
	renderer_config.world_aabb = { glm::vec3(-15.0f, -1.0f, -15.0f), glm::vec3(15.0f, 20.0f, 15.0f) };
	renderer_config.cam.position = { 15.0f, 15.0f, 15.0f };
	renderer_config.cam.target = { 0.0f, 2.0f, 0.0f };
	renderer_config.light_dir = { -0.5f, -1.0f, -0.4f };
	renderer_config.cam.position = { 15.5943, 28.0221, 42.1298 };
	renderer_config.cam.target = { 15.2499, 27.5946, 41.2939 };
	renderer = std::make_shared<RigidWorldRenderer>(renderer_config);

	std::vector<size_t> render_keys;
	for (int i = 0; i < graph.size(); ++i) {
		render_keys.push_back(add_node_to_renderer(graph[i]));
	}

	world = std::make_shared<RigidWorld>();
	std::vector<std::shared_ptr<RigidBody>> rigidbodies;
	for (int i = 0; i < graph.size(); ++i) {
		rigidbodies.push_back(create_rigidbody_from_node(graph[i]));
		world->add_body(rigidbodies[i]);
	}

	auto update_world = [&](float frame_dt, size_t frame_id) {
		world->step(0.01667f);
		for (size_t i = 0; i < rigidbodies.size(); ++i) {
			auto rb = rigidbodies[i];
			if (!rb) {
				continue;
			}
			renderer->update_body(render_keys[i], q(rb->rotation), v3(rb->translation));
		}
	};

	RigidWorldRenderer::Options opts;
	// opts.show_light_config = true;
	renderer->run(update_world, nullptr, opts);

	return 0;
}