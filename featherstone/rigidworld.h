#pragma once

#include "rigidbody.hpp"
#include "articulatedbody.hpp"
#include "spshapes.hpp"
#include "contact_solver.h"
#include "loop_joint_solver.h"

#include "btBulletCollisionCommon.h"

#include <map>

namespace SPD {

struct RigidWorld {
	RigidWorld(Eigen::Vector3f gravity = Eigen::Vector3f(0.0f, -10.0f, 0.0f));

	~RigidWorld();

	void add_body(std::shared_ptr<RigidBody> rigidbody);

	void add_body(std::shared_ptr<ArticulatedBody> body);

	void step(float dt);
	
	void collide();

	void integrate_velocity(float dt);

	void integrate_position(float dt);
	
	// Eigen::Vector3f find_penetration(const btCollisionObject* obj0, const btCollisionObject* obj1);

	Eigen::Vector3f gravity;

	std::vector<std::shared_ptr<RigidBody>> rigidbodies;
	std::vector<std::shared_ptr<ArticulatedBody>> artbodies;

	struct Collider {
		static std::shared_ptr<Collider> create(const RigidBody& rigidbody, int user_id);
		static std::vector<std::shared_ptr<Collider>> create(const ArticulatedBody& artbody, int user_id);
		void update(Eigen::Vector3f translation, Eigen::Quaternionf rotation);
		std::shared_ptr<btCollisionShape> shape;
		std::shared_ptr<btCollisionObject> obj;
	};
	std::vector<std::shared_ptr<Collider>> rigid_colliders;
	std::vector<std::vector<std::shared_ptr<Collider>>> art_colliders;

	// Handle collision disabling across joints
	struct AdjacentLinkFilter : btOverlapFilterCallback {
		typedef std::set<int> Adjacency;
		typedef std::map<int, Adjacency> AdjacencyMap;
		std::vector<AdjacencyMap> collision_disable_maps;
		void add_body(std::shared_ptr<ArticulatedBody> artbody);
		bool needBroadphaseCollision(btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1) const override;
	};

	struct CollisionWorld {
		std::shared_ptr<btCollisionWorld> world = nullptr;
		std::shared_ptr<btDefaultCollisionConfiguration> config = nullptr;
		std::shared_ptr<btCollisionDispatcher> dispatcher = nullptr;
		std::shared_ptr<btDbvtBroadphase> broadphase = nullptr;
		std::shared_ptr<AdjacentLinkFilter> articulation_collision_filter = nullptr;
	};
	std::shared_ptr<CollisionWorld> collision_world;

	std::shared_ptr<ContactSolver> contact_solver;
	std::shared_ptr<LoopJointSolver> loop_joint_solver;

	static const uint32_t max_velocity_solve_iterations = 10;
	static const uint32_t max_position_solve_iterations = 8;
};

}