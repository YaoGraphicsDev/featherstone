#include "gltf_physics_body_parser.h"

#include "rigidworld.h"
#include "rigidworld_renderer.h"

#include <iostream>

using namespace SPD;
using namespace Eigen;

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

std::shared_ptr<RigidBody> build_rigidbody(const RigidBodyDescriptor& desc) {
	std::shared_ptr<RigidBody> rb = nullptr;
	RigidBody::Config config;
	if (desc.implicit_shape->type == ImplicitShape::Type::Box) {
		config.shape.reset(new Cuboid(desc.implicit_shape->half_dims));
	}
	else {
		assert(false);
	}
	config.rotation = desc.rotation.normalized();
	config.translation = desc.translation;
	config.type = static_cast<RigidBody::DynamicType>(desc.dyn_type);
	config.density = desc.mass / config.shape->vol;
	config.restitution_coeff = desc.material->restitution;
	config.friction_coeff = desc.material->friction;

	rb.reset(new RigidBody(config));
	return rb;
}

size_t add_rigidbody_to_renderer(std::shared_ptr<RigidWorldRenderer> renderer, std::shared_ptr<RigidBody> rb) {
	RigidWorldRenderer::Shape render_shape;
	if (rb->shape->type == SPD::Shape::Type::Cuboid) {
		render_shape = RigidWorldRenderer::Shape::Cuboid;
	}
	else {
		assert(false);
	}
	return renderer->add_body(render_shape, v3(std::dynamic_pointer_cast<SPD::Cuboid>(rb->shape)->half_dims));
}

int main() {
	std::vector<RigidBodyDescriptor> rigid_descs;
	std::vector<ArticulatedBodyDescriptor> articulated_descs;
	load_gltf_physics_world(std::string(SCENES_DIR) + "wrecking_ball/wrecking_ball.gltf", rigid_descs, articulated_descs);

	std::shared_ptr<RigidWorld> world = std::make_shared<RigidWorld>();

	RigidWorldRenderer::Config renderer_config;
	renderer_config.world_aabb = { glm::vec3(-10.0f, -1.0f, -10.0f), glm::vec3(10.0f, 15.0f, 10.0f) };
	renderer_config.cam.position = { 15.0f, 15.0f, 15.0f };
	renderer_config.cam.target = { 0.0f, 2.0f, 0.0f };
	renderer_config.light_dir = { -0.5f, -1.0f, -0.4f };
	std::shared_ptr<RigidWorldRenderer> renderer = std::make_shared<RigidWorldRenderer>(renderer_config);

	std::vector<std::shared_ptr<RigidBody>> rigidbodies;
	std::vector<size_t> render_keys;
	for (const auto& desc : rigid_descs) {
		auto rb = build_rigidbody(desc);
		rigidbodies.push_back(rb);
		world->add_body(rb);
		render_keys.push_back(add_rigidbody_to_renderer(renderer, rb));
	}

	auto update_world = [&](float frame_dt, size_t frame_id) {
		world->step(0.01667f);
		for (size_t i = 0; i < rigidbodies.size(); ++i) {
			auto rb = rigidbodies[i];
			renderer->update_body(render_keys[i], q(rb->rotation), v3(rb->translation));
		}
	};

	renderer->run(update_world, nullptr);

	return 0;
}