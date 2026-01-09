#pragma once

#include "spshapes.hpp"
#include "rigidbody.hpp"

#include <memory>
#include <vector>
#include <map>
#include <set>
#include <list>

#include "Eigen/Core"

namespace SPD {

struct ArticulatedBody {
	struct Constraint;

	struct Body {
		std::shared_ptr<Shape> shape = nullptr;

		// values determined by the time constraint is set
		std::list<std::shared_ptr<Constraint>> parent_joints; // loop joints will get picked out when building tree. There will only be one parent joint per body left after build_tree() is done
		std::vector<std::shared_ptr<Constraint>> children_joints;
		Dyad I = Dyad::Identity();

		// values set when building dynamic tree
		int id = 0;

		// values updated every frame
		Eigen::Quaternionf rotation = Eigen::Quaternionf::Identity();
		Eigen::Vector3f translation = Eigen::Vector3f::Zero();
		Eigen::Matrix3f bases = rotation.toRotationMatrix(); // body origin space bases in world space.
		MVector v = MVector::Zero(); // body velocity, not origin velocity
		//// MVector a = MVector::Zero();

		// provided by eternal source
		FVector fe0 = FVector::Zero(); // external force, in body0 space
		float restitution_coeff = 0.3f;
		float friction_coeff = 0.5f;
		float mass = 1.0f;
	};

	ArticulatedBody() {
		// add an invisible fixed shape
		bodies = { std::make_shared<Body>() };
	}

	ArticulatedBody(const RigidBody& rb) {
		assert(rb.type == RigidBody::DynamicType::Static);
		add_body(rb);
	}

	~ArticulatedBody() {};

	std::shared_ptr<Body> base() {
		assert(bodies.size() >= 1);
		return bodies.front();
	}

	void set_gravity(Eigen::Vector3f gravity) {
		this->gravity = gravity;
	}

	// std::shared_ptr<Body> add_body(std::shared_ptr<Shape> shape, Eigen::Quaternionf rotation, Eigen::Vector3f translation);

	std::shared_ptr<Body> add_body(const RigidBody& rb);

	enum class ConstraintType {
		Revolute = 0,
		Prismatic,
		Ball,
		Free
	};
	struct Constraint {
		ConstraintType type;
		std::shared_ptr<Body> b0;
		std::shared_ptr<Body> b1;
		Eigen::Matrix3f bb0; // bases of joint space, in body 0 origin space
		Eigen::Vector3f bt0; // translation of joint space, in body 0 orign space
		Eigen::Matrix3f bb1; // bases of joint space, in body 1 origin space
		Eigen::Vector3f bt1; // translation of joint space, in body 1 origin space
		MTransform X_0_J0; // transform from body 0 origin space to joint space 0
		MTransform X_1_J1; // transform from body 1 origin space to joint space 1
		MTransform X_J0_J1; // transform from joint 0 to joint space 1
		const MSubspace* S; // motion subspace
		const FSubspace* T; // constraint force subspace
		const FSubspace* Ta; // active force subspace
		MCoordinates q;
		MCoordinates dq;
		MCoordinates ddq;
		FCoordinates bias; // joint space bias force

		FCoordinates taue; // external joint force, set by external source

		bool disable_collision = true; // disable collision across joint
	};
	//struct RevoluteJoint : public Constraint {

	//};
	//struct PrismaticJoint : public Constraint {
	//	MSubspace<1> S;
	//	MCoordinate<1> q;
	//};
	// ball joint is more complicated than this. See page 80 of the book
	//struct BallJoint : public Constraint {
	//	Eigen::Matrix3f q;
	//};

	std::shared_ptr<Constraint> add_constraint(
		ConstraintType type,
		size_t id0,
		size_t id1,
		Eigen::Matrix3f base0,
		Eigen::Vector3f trans0);

	std::shared_ptr<Constraint> add_constraint(
		ConstraintType type,
		std::shared_ptr<Body> b0,
		std::shared_ptr<Body> b1,
		Eigen::Matrix3f base0,
		Eigen::Vector3f trans0);

	bool build_tree();

	void integrate_velocity(float dt);

	void integrate_position(float dt);

	void project_velocity();

	void project_position();

	void move_constraints();
	
	// body_id -- contact body
	// return Jacobian in body0 space
	MSubspace jacobian_0(size_t body_id);

	MCoordinates dq(size_t body_id);

	void apply_delta_dq(MCoordinates delta_dq);

	void apply_delta_q(MCoordinates delta_q);

	JDyad H_inv(const Unitless& J);

	// recursive newton-euler algo
	void compute_bias_RNEA();

	void solve_ddq();

	// TODO: temporary
	void joint_damping();

	void clear_joint_forces();

	void compute_H(); // joint space inertia matrix

	GPower compute_delta(ConstraintType type, const MTransform& X); // positional error of loop joints

	void compute_K_k(); // joint space acceleration constraint parameters

	// joint motion subspace
	const std::map<ConstraintType, MSubspace> S = {
		{ConstraintType::Prismatic, subspace({{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f}})},
		{ConstraintType::Revolute, subspace({{0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f}})},
	}; 
	// joint active force subspace
	const std::map<ConstraintType, FSubspace> Ta = {
		{ConstraintType::Prismatic, subspace({{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f}})},
		{ConstraintType::Revolute, subspace({{0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f}})},
	};
	// joint constraint force subspace
	const std::map<ConstraintType, FSubspace> T = {
		{ConstraintType::Prismatic, subspace({
			{ 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
			{ 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f },
			{ 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f },
			{ 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f },
			{ 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f }})},
		{ConstraintType::Revolute, subspace({
			{ 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
			{ 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f },
			{ 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f },
			{ 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f },
			{ 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f }})},
	};

	Eigen::Matrix<float, 6, Eigen::Dynamic, 0, 6, 6> subspace(std::initializer_list<std::array<float, 6>> columns);

	std::vector<std::shared_ptr<Body>> bodies;
	std::vector<std::shared_ptr<Constraint>> tree_joints;
	std::vector<std::shared_ptr<Constraint>> loop_joints;
 	std::vector<int> lambda;
	std::vector<std::set<int>> mu;
	std::vector<std::set<int>> nu;

	// set by build_tree()
	MTransform X_w_0; // transformation from world space to body 0 space
	MTransform X_0_w;
	// set by move_constraints()
	std::vector<MTransform> X_0_; // transformation from body0 space to any body space.
	std::vector<MTransform> X_Li_; // transformation from parent body space to chile body space
	// set by build_tree()
	std::vector<MTransform> XP; // loop joints' locations in predecessor body
	std::vector<MTransform> XS; // loop joints' locations in sucessor body
	std::vector<FCoordinates> SI; // Sequential Impulse on loop joints

	Eigen::Vector3f gravity;

	std::vector<MVector> a_vp; // velocity product. Acceleration of bodies if tree joint accelerations (ddq) are zero

	JDyad H; // joint space inertia matrix H
	Eigen::LLT<JDyad> H_llt;
	std::shared_ptr<BlockAccess> H_acc = nullptr;

	GPower K;
	std::shared_ptr<BlockAccess> K_acc;

	GPower _k;
	std::shared_ptr<BlockAccess> k_acc;

	const float alpha = 0.5f;
	const float beta = 0.5f;
};

}