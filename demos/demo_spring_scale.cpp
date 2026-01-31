#include "demo_common.h"
#include "command_server.h"
#include "json.hpp" // nlohmann json

#include <iostream>
#include <sstream>

using namespace Eigen;
using namespace SPD;

int main() {
	Scene scene;
	if (!load_gltf(std::string(SCENES_DIR) + "articulated/spring_scale.gltf", scene, GLTFParseOption::BlenderExport)) {
		std::cout << "error loading gltf resource" << std::endl;
		return 0;
	}

	init_renderer();

	// add to renderer
	std::vector<size_t> render_keys;
	for (int i = 0; i < scene.graph.size(); ++i) {
		render_keys.push_back(add_node_to_renderer(scene.graph[i]));
	}

	init_world();

	// add articulated body
	assert(scene.art_forest.size() == scene.art_groups.size());
	assert(scene.art_forest.size() == 1);
	std::shared_ptr<ArticulatedBody> artbody = create_articulated_body(scene.graph, scene.art_groups[0], scene.art_forest[0]);

	// configure constraints manually. Havent yet found a way to store this in a .gltf
	std::shared_ptr<ArticulatedBody::Joint> rack = artbody->get_joint("rack_prismatic");
	std::shared_ptr<ArticulatedBody::Joint> follow = artbody->get_joint("following_revolute");
	if (!rack) {
		std::cout << "Cannot find rack prismatic joint" << std::endl;
		assert(false); 
		return 0;
	}
	if (!follow) {
		std::cout << "Cannot find following revolute joint" << std::endl;
		assert(false);
		return 0;
	}
	artbody->add_constraint("rack_follow", rack, follow, -0.45f);
	g_world->add_body(artbody);

	// add rigid bodies
	std::vector<std::shared_ptr<RigidBody>> rigidbodies;
	for (int i : scene.rigidbody_group) {
		rigidbodies.push_back(create_rigidbody_from_node(scene.graph[i]));
		g_world->add_body(rigidbodies.back());
	}

	float ki = 30.0f;
	auto update_world = [&](float frame_dt, size_t frame_id) {
		g_world->step(0.01667f);

		// update all rigid bodies
		for (int i = 0; i < scene.rigidbody_group.size(); ++i) {
			g_renderer->update_body(render_keys[scene.rigidbody_group[i]], q(rigidbodies[i]->rotation), v3(rigidbodies[i]->translation));
		}

		// update all articulated bodies
		const NodeGroup& group = scene.art_groups[0];
		for (int i = 0; i < group.size(); ++i) {
			g_renderer->update_body(
				render_keys[group[i]],
				q(artbody->bodies[i]->rotation),
				v3(artbody->bodies[i]->translation));
		}
	};

	auto draw_articulated_joints = [&](std::shared_ptr<ArticulatedBody> artbody) {
		for (int i = 1; i < artbody->tree_joints.size(); ++i) {
			std::shared_ptr<ArticulatedBody::Joint> c = artbody->tree_joints[i];
			g_renderer->draw_bases(v3(c->b0->translation + c->b0->bases * c->bt0), m3(c->b0->bases * c->bb0), 1.0f);
			g_renderer->draw_bases(v3(c->b1->translation + c->b1->bases * c->bt1), m3(c->b1->bases * c->bb1), 1.0f);
		}
		for (int i = 0; i < artbody->loop_joints.size(); ++i) {
			std::shared_ptr<ArticulatedBody::Joint> c = artbody->loop_joints[i];
			g_renderer->draw_bases(v3(c->b0->translation + c->b0->bases * c->bt0), m3(c->b0->bases * c->bb0), 1.0f);
			g_renderer->draw_bases(v3(c->b1->translation + c->b1->bases * c->bt1), m3(c->b1->bases * c->bb1), 1.0f);
		}
	};

	auto debug_draw = [&](float frame_dt, size_t frame_id) {
		// renderer.draw_bases(glm::vec3(0.0f), glm::mat3(1.0f), 1.0f);
		draw_articulated_joints(artbody);
	};


	RigidWorldRenderer::Options opts;
	// opts.show_light_config = true;
	opts.movable_light = true;
	g_renderer->run(update_world, debug_draw, opts);

	return 0;
}