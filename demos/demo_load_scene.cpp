#include "gltf_parser.h"

#include "rigidworld.h"
#include "rigidworld_renderer.h"

#include <iostream>

using namespace Eigen;

std::shared_ptr<RigidWorldRenderer> renderer = nullptr;

glm::quat q(Quaternionf q) {
	return glm::quat(q.w(), q.x(), q.y(), q.z());
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

int main() {
	SceneGraph graph;
	SceneGraphFlatRefs refs;
	if (!load_gltf(std::string(SCENES_DIR) + "wrecking_ball/wrecking_ball.gltf", graph, refs)) {
		std::cout << "error loading gltf resource" << std::endl;
		return 0;
	}
	
	RigidWorldRenderer::Config renderer_config;
	renderer_config.world_aabb = { glm::vec3(-10.0f, -1.0f, -10.0f), glm::vec3(10.0f, 15.0f, 10.0f) };
	renderer_config.cam.position = { 15.0f, 15.0f, 15.0f };
	renderer_config.cam.target = { 0.0f, 2.0f, 0.0f };
	renderer_config.light_dir = { -0.5f, -1.0f, -0.4f };
	renderer = std::make_shared<RigidWorldRenderer>(renderer_config);

	for (int i = 0; i < graph.size(); ++i) {
		add_node_to_renderer(graph[i]);
	}

	auto update_world = [&](float frame_dt, size_t frame_id) {
	};

	renderer->run(update_world, nullptr);

	//std::vector<RigidBodyDescriptor> rigid_descs;
	//std::vector<ArticulatedBodyDescriptor> articulated_descs;
	//load_gltf_physics_world(std::string(SCENES_DIR) + "wrecking_ball/wrecking_ball.gltf", rigid_descs, articulated_descs);

	//std::shared_ptr<RigidWorld> world = std::make_shared<RigidWorld>();

	//RigidWorldRenderer::Config renderer_config;
	//renderer_config.world_aabb = { glm::vec3(-10.0f, -1.0f, -10.0f), glm::vec3(10.0f, 15.0f, 10.0f) };
	//renderer_config.cam.position = { 15.0f, 15.0f, 15.0f };
	//renderer_config.cam.target = { 0.0f, 2.0f, 0.0f };
	//renderer_config.light_dir = { -0.5f, -1.0f, -0.4f };
	//std::shared_ptr<RigidWorldRenderer> renderer = std::make_shared<RigidWorldRenderer>(renderer_config);

	//std::vector<std::shared_ptr<RigidBody>> rigidbodies;
	//std::vector<size_t> render_keys;
	//for (const auto& desc : rigid_descs) {
	//	auto rb = build_rigidbody(desc);
	//	rigidbodies.push_back(rb);
	//	world->add_body(rb);
	//	render_keys.push_back(add_rigidbody_to_renderer(renderer, rb));
	//}

	//auto update_world = [&](float frame_dt, size_t frame_id) {
	//	world->step(0.01667f);
	//	for (size_t i = 0; i < rigidbodies.size(); ++i) {
	//		auto rb = rigidbodies[i];
	//		renderer->update_body(render_keys[i], q(rb->rotation), v3(rb->translation));
	//	}
	//};

	//renderer->run(update_world, nullptr);

	return 0;
}