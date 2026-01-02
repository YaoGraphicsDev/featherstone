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

std::shared_ptr<ArticulatedBody> create_articulated_body(const SceneGraph& graph, const NodeGroup& art_group, const ArticulationTree& art_tree) {
	assert(!art_tree.empty() && !art_group.empty());
	int base_id = art_group[0];
	if (graph[base_id].physical->dyn_type !=  Physical::DynamicType::Static) {
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

	// TODO: blender exclusive
	//Eigen::Matrix3f joint_frame_blender;
	////joint_frame_blender <<
	////	1, 0, 0,
	////	0, 0, -1,
	////	0, 1, 0;
	//joint_frame_blender <<
	//	1, 0, 0,
	//	0, 0, 1,
	//	0, -1, 0;

	for (const ArticulationLinkage& link : art_tree) {
		art->add_constraint(
			(ArticulatedBody::ConstraintType)link.joint.type,
			link.bodyA_id,
			link.bodyB_id,
			link.bodyA_rotation.toRotationMatrix(), // * joint_frame_blender,
			link.bodyA_translation);
	}

	art->build_tree();
	return art;
}

int main() {
	Scene scene;
	if (!load_gltf(std::string(SCENES_DIR) + "articulated/articulated_simplified.gltf", scene, GLTFParseOption::BlenderExport)) {
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

	// add to renderer
	std::vector<size_t> render_keys;
	for (int i = 0; i < scene.graph.size(); ++i) {
		render_keys.push_back(add_node_to_renderer(scene.graph[i]));
	}

	// add to physics world
	Eigen::Vector3f gravity = Eigen::Vector3f(0.0f, -10.0f, 0.0f);
	world = std::make_shared<RigidWorld>(gravity);

	// add articulated bodies
	std::vector<std::shared_ptr<ArticulatedBody>> artbodies;
	assert(scene.art_forest.size() == scene.art_groups.size());
	for (int i = 0; i < scene.art_forest.size(); ++i) {
		artbodies.push_back(create_articulated_body(scene.graph, scene.art_groups[i], scene.art_forest[i]));
		world->add_body(artbodies.back());
	}

	// add rigid bodies
	std::vector<std::shared_ptr<RigidBody>> rigidbodies;
	for (int i : scene.rigidbody_group) {
		rigidbodies.push_back(create_rigidbody_from_node(scene.graph[i]));
		world->add_body(rigidbodies.back());
	}
	
	auto update_world = [&](float frame_dt, size_t frame_id) {
		if (frame_id == 210) {
			int a = 0;
		}
		world->step(0.01667f);

		// update all rigid bodies
		for (int i = 0; i < scene.rigidbody_group.size(); ++i) {
			renderer->update_body(render_keys[scene.rigidbody_group[i]], q(rigidbodies[i]->rotation), v3(rigidbodies[i]->translation));
		}

		// update all articulated bodies
		for (int g = 0; g < scene.art_groups.size(); ++g) {
			const NodeGroup& group = scene.art_groups[g];
			for (int i = 0; i < group.size(); ++i) {

				renderer->update_body(
					render_keys[group[i]],
					q(artbodies[g]->bodies[i]->rotation),
					v3(artbodies[g]->bodies[i]->translation));
			}
		}
	};

	auto draw_articulated_joints = [&](std::shared_ptr<ArticulatedBody> artbody) {
		for (int i = 1; i < artbody->tree_joints.size(); ++i) {
			std::shared_ptr<ArticulatedBody::Constraint> c = artbody->tree_joints[i];
			renderer->draw_bases(v3(c->b0->translation + c->b0->bases * c->bt0), m3(c->b0->bases * c->bb0), 3.0f);
			renderer->draw_bases(v3(c->b1->translation + c->b1->bases * c->bt1), m3(c->b1->bases * c->bb1), 3.0f);
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
	renderer->run(update_world, debug_draw, opts);

	return 0;
}