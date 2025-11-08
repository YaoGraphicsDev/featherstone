#pragma once

#include "rigidbody.hpp"
#include "spshapes.hpp"
#include "contact_solver.h"

#include "btBulletCollisionCommon.h"

#include <map>

namespace SPD {

struct RigidWorld {
	RigidWorld(Eigen::Vector3f gravity = Eigen::Vector3f(0.0f, -10.0f, 0.0f));

	~RigidWorld();

	void add_body(std::shared_ptr<RigidBody> body);

	void step(float dt);
	
	void collide();

	void integrate_velocity(float dt);

	void integrate_position(float dt);

	float new_penetration(
		const RigidBody& b0, Eigen::Vector3f local_p0,
		const RigidBody& b1, Eigen::Vector3f local_p1,
		Eigen::Vector3f n_01);

	
	// Eigen::Vector3f find_penetration(const btCollisionObject* obj0, const btCollisionObject* obj1);

	Eigen::Vector3f gravity;

	std::vector<std::shared_ptr<RigidBody>> bodies;

	struct Collider {
		static std::shared_ptr<Collider> create(const RigidBody& rigidbody, int user_id);
		void update(Eigen::Vector3f translation, Eigen::Quaternionf rotation);
		std::shared_ptr<btCollisionShape> shape;
		std::shared_ptr<btCollisionObject> obj;
	};
	std::vector<std::shared_ptr<Collider>> colliders;

	struct CollisionWorld {
		std::shared_ptr<btCollisionWorld> world = nullptr;
		std::shared_ptr<btDefaultCollisionConfiguration> config = nullptr;
		std::shared_ptr<btCollisionDispatcher> dispatcher = nullptr;
		std::shared_ptr<btDbvtBroadphase> broadphase = nullptr;
	};
	std::shared_ptr<CollisionWorld> collision_world;

	std::shared_ptr<ContactSolver> contact_solver;

	static const uint32_t max_velocity_solve_iterations = 10;
	static const uint32_t max_position_solve_iterations = 5;
};

}