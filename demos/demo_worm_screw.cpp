#include "demo_common.h"
#include "command_server.h"
#include "json.hpp" // nlohmann json

#include <iostream>
#include <sstream>

using namespace Eigen;
using namespace SPD;

int main(int argc, char** argv) {
	Scene scene;
	if (!load_gltf(std::string(SCENES_DIR) + "articulated/worm_screw_jack.gltf", scene, GLTFParseOption::BlenderExport)) {
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
	std::shared_ptr<ArticulatedBody::Joint> base_rev_left = artbody->get_joint("base_rev_left");
	std::shared_ptr<ArticulatedBody::Joint> base_rev_right = artbody->get_joint("base_rev_right");
	std::shared_ptr<ArticulatedBody::Joint> push_rev_left = artbody->get_joint("push_rev_left");
	std::shared_ptr<ArticulatedBody::Joint> push_rev_right = artbody->get_joint("push_rev_right");
	std::shared_ptr<ArticulatedBody::Joint> screw = artbody->get_joint("screw");
	std::shared_ptr<ArticulatedBody::Joint> worm_screw = artbody->get_joint("worm_screw");
	artbody->add_constraint("base_gear", base_rev_left, base_rev_right, 1.0f);
	artbody->add_constraint("push_gear", push_rev_left, push_rev_right, -1.0f);
	artbody->add_constraint("screw", screw, 0, screw, 1, 2.0f);
	artbody->add_constraint("worm", worm_screw, 0, worm_screw, 1, 30.0f);
	artbody->add_constraint("worm_linear", worm_screw, 1, screw, 0, 0.8f);

	g_world->add_body(artbody);

	// add rigid bodies
	std::vector<std::shared_ptr<RigidBody>> rigidbodies;
	for (int i : scene.rigidbody_group) {
		rigidbodies.push_back(create_rigidbody_from_node(scene.graph[i]));
		g_world->add_body(rigidbodies.back());
	}

	// start up command server
	CommandServerWin server(7777);
	server.start();

	// PI control parameters
	float kp = 0.2f;
	float ki = 1.5f;
	float dq_target = 0.0f;

	bool pi_online = true;


	auto command_handler = [&](std::optional<std::string> line) {
		if (!line.has_value()) {
			return;
		}

		std::istringstream iss(*line);
		std::string cmd;
		std::string param;
		float value;
		nlohmann::json j;
		iss >> cmd;
		if (cmd == "ls") {
			iss >> param;
			if (param == "joints") {
				for (int i = 1; i < artbody->tree_joints.size(); ++i) {
					j["tree"][i - 1] = artbody->tree_joints[i]->name;
				}
				for (int i = 0; i < artbody->loop_joints.size(); ++i) {
					j["loop"][i] = artbody->loop_joints[i]->name;
				}
				server.send_reply(j.dump(4));
			}
			else if (param == "bodies") {
				std::string names;
				for (auto b : artbody->bodies) {
					j.push_back(b->name);
				}
				server.send_reply(j.dump(4));
			}
			else {
				server.send_reply("invalid parameter for [ls]");
			}
		}
		else if (cmd == "apply_tau") {
			iss >> param;
			if (auto joint = artbody->get_joint(param)) {
				for (int i = 0; i < joint->taue.size(); ++i) {
					if (iss >> value) {
						joint->taue(i) = value;
					}
				}
				std::ostringstream oss;
				oss << joint->taue.transpose();
				server.send_reply("tau applies to " + joint->name + ", value = " + oss.str());
			}
			else {
				server.send_reply("Cannot find joint [" + param + "]");
			}
		}
		else if (cmd == "worm_screw_target") {
			iss >> dq_target;
			std::ostringstream oss;
			oss << dq_target;
			server.send_reply("worm screw target = " + oss.str());
		}
		else if (cmd == "worm_screw_get") {
			server.send_reply("worm screw tau = " + std::to_string(worm_screw->taue(0)) + ", dq = " + std::to_string(worm_screw->dq(0)));
		}
		else if (cmd == "pid_toggle") {
			pi_online = !pi_online;
			if (pi_online) {
				server.send_reply("PID online");
			}
			else {
				server.send_reply("PID offline");
			}

		}
		else {
			server.send_reply("invalid command");
		}
	};

	float I = 0.0f;
	float e = 0.0f;
	float tau = 0.0f;
	float error = 0.0f;

	auto update_world = [&](float frame_dt, size_t frame_id) {
		// parse command line
		auto cmd_line = server.try_pop_command();
		command_handler(cmd_line);

		if (pi_online) {
			error = dq_target - worm_screw->dq(0);
			I += error / 60.0f; // may explode if stalls

			worm_screw->taue(0) = kp * error + ki * I;
		}

		g_world->step(0.016667f);

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

	server.stop();

	return 0;
}