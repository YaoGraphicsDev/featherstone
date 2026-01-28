#include "demo_common.h"

#include <iostream>

using namespace Eigen;
using namespace SPD;

int main() {
	Scene scene;
	if (!load_gltf(std::string(SCENES_DIR) + "wrecking_ball/wrecking_ball.gltf", scene, GLTFParseOption::BlenderExport)) {
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

	// add articulated bodies
	std::vector<std::shared_ptr<ArticulatedBody>> artbodies;
	assert(scene.art_forest.size() == scene.art_groups.size());
	for (int i = 0; i < scene.art_forest.size(); ++i) {
		artbodies.push_back(create_articulated_body(scene.graph, scene.art_groups[i], scene.art_forest[i]));
		g_world->add_body(artbodies.back());
	}

	// add rigid bodies
	std::vector<std::shared_ptr<RigidBody>> rigidbodies;
	for (int i : scene.rigidbody_group) {
		rigidbodies.push_back(create_rigidbody_from_node(scene.graph[i]));
		g_world->add_body(rigidbodies.back());
	}
	
	auto update_world = [&](float frame_dt, size_t frame_id) {
		g_world->step(0.01667f);

		// update all rigid bodies
		for (int i = 0; i < scene.rigidbody_group.size(); ++i) {
			g_renderer->update_body(render_keys[scene.rigidbody_group[i]], q(rigidbodies[i]->rotation), v3(rigidbodies[i]->translation));
		}

		// update all articulated bodies
		for (int g = 0; g < scene.art_groups.size(); ++g) {
			const NodeGroup& group = scene.art_groups[g];
			for (int i = 0; i < group.size(); ++i) {

				g_renderer->update_body(
					render_keys[group[i]],
					q(artbodies[g]->bodies[i]->rotation),
					v3(artbodies[g]->bodies[i]->translation));
			}
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
		for (auto artbody : artbodies) {
			draw_articulated_joints(artbody);
		}
	};


	RigidWorldRenderer::Options opts;
	// opts.show_light_config = true;
	g_renderer->run(update_world, debug_draw, opts);

	return 0;
}