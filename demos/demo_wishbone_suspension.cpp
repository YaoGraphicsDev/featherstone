#include "demo_common.h"
#include "command_server.h"
#include "json.hpp" // nlohmann json

#include <iostream>
#include <sstream>

using namespace Eigen;
using namespace SPD;

int main(int argc, char** argv) {
	Scene scene;
	if (!load_gltf(std::string(SCENES_DIR) + "articulated/wishbone.gltf", scene, GLTFParseOption::BlenderExport)) {
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
	std::shared_ptr<ArticulatedBody::Joint> steering = artbody->get_joint("steering");
	if (!steering) {
		std::cout << "Cannot find steering joint" << std::endl;
		assert(false);
		return 0;
	}
	std::shared_ptr<ArticulatedBody::Body> wheel = artbody->get_body("wheel");
	if (!wheel) {
		std::cout << "Cannot find wheel body" << std::endl;
		return 0;
	}

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
	float kp = -50.0f;
	float ki = -35.0f;
	float kd = -25.0f;
	float full_steer = glm::pi<float>() * 0.15f;

	float wheel_rotate_target = 0.0f;
	float rig_height = 0.0f;
	float I = 0.0f;
	bool pd_online = true;

	bool rig_bump = false;

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
		else if (cmd == "steer") {
			iss >> value;
			if (value < -1.0f || value > 1.0f) {
				server.send_reply("pick a value between -1.0f and 1.0f");
			}
			else {
				wheel_rotate_target = value * full_steer;
				server.send_reply((value > 0.0f ? "left " : "right ") + std::to_string(std::abs(value)) + " full steer");
			}
		}
		else if (cmd == "pid_toggle") {
			pd_online = !pd_online;
			if (pd_online) {
				server.send_reply("PID online");
			}
			else {
				server.send_reply("PID offline");
			}
		}
		else if (cmd == "bump") {
			rig_bump = true;
			server.send_reply("bump");
		}
		else {
			server.send_reply("invalid command");
		}
	};

	auto update_world = [&](float frame_dt, size_t frame_id) {
		// parse command line
		auto cmd_line = server.try_pop_command();
		command_handler(cmd_line);

		if (pd_online) {
			// figure out the turning angle
			Vector3f wheel_axis = wheel->rotation * -Vector3f::UnitZ();
			wheel_axis.y() = 0.0f;
			SignedAxisAnglef aa = axis_angle(-Vector3f::UnitZ(), wheel_axis);
			float turning_angle = aa.axis.y() * aa.angle;

			float error = wheel_rotate_target - turning_angle;
			I += error / 60.0f; // may explode if stalls
			steering->taue(0) = kp * error + ki * I + kd * steering->dq(0);
		}

		if (rig_bump) {
			rig_bump = !rig_bump;
			auto rig_articulate = artbody->get_joint("rig_articulate");
			if (rig_articulate) {
				rig_articulate->taue(0) = -8000.0f;
			}
		}
		else {
			auto rig_articulate = artbody->get_joint("rig_articulate");
			if (rig_articulate) {
				rig_articulate->taue(0) = 0.0f;
			}
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
			g_renderer->draw_bases(v3(c->b0->translation + c->b0->bases * c->bt0), m3(c->b0->bases * c->bb0), 0.3f);
			g_renderer->draw_bases(v3(c->b1->translation + c->b1->bases * c->bt1), m3(c->b1->bases * c->bb1), 0.3f);
		}
		for (int i = 0; i < artbody->loop_joints.size(); ++i) {
			std::shared_ptr<ArticulatedBody::Joint> c = artbody->loop_joints[i];
			g_renderer->draw_bases(v3(c->b0->translation + c->b0->bases * c->bt0), m3(c->b0->bases * c->bb0), 0.3f);
			g_renderer->draw_bases(v3(c->b1->translation + c->b1->bases * c->bt1), m3(c->b1->bases * c->bb1), 0.3f);
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