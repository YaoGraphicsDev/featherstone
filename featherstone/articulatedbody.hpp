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
	struct Joint;

	struct Body {
		std::string name = "";
		std::shared_ptr<Shape> shape = nullptr;

		// values determined by the time constraint is set
		std::list<std::shared_ptr<Joint>> parent_joints; // loop joints will get picked out when building tree. There will only be one parent joint per body left after build_tree() is done
		std::vector<std::shared_ptr<Joint>> children_joints;
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

	enum class JointType {
		Revolute = 0,
		Prismatic,
		Cylindrical,
		Spherical
	};
	struct Joint {
		std::string name;
		JointType type;
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
		MSubspace S_p; // parameterized motion subspace instance
		FSubspace T_p; // parameterized constraint force subspace instance
		FSubspace Ta_p; // parameterized active force subspace instance
		MCoordinates q;
		Eigen::Quaternionf qs; // exclusive to spherical joint
		MCoordinates dq;
		MCoordinates ddq;
		FCoordinates bias; // joint space bias force

		FCoordinates taue; // external joint force, set by external source

		bool disable_collision = true; // disable collision across joint

		// spring parameters
		bool enable_spring = false;
		MCoordinates ks; // stiffness
		MCoordinates ds; // damping

		// values set when building dynamic tree
		int id;
	};

	struct SpringParam {
		float k;
		float d;
	};

	std::shared_ptr<Joint> add_joint(
		std::string name,
		JointType type,
		size_t id0,
		size_t id1,
		Eigen::Matrix3f base0,
		Eigen::Vector3f trans0,
		bool disable_collision = true,
		std::vector<SpringParam> spring_params = {});

	std::shared_ptr<Joint> add_joint(
		std::string name,
		JointType type,
		std::shared_ptr<Body> b0,
		std::shared_ptr<Body> b1,
		Eigen::Matrix3f base0,
		Eigen::Vector3f trans0,
		bool disable_collision = true,
		std::vector<SpringParam> spring_params = {});

	std::shared_ptr<Joint> get_joint(const std::string& name) const;

	std::shared_ptr<Body> get_body(const std::string& name) const;

	// gear/belt/rack-and-pinion
	struct Constraint {
		std::string name;
		std::shared_ptr<Joint> j0;
		int msubspace_col0;
		std::shared_ptr<Joint> j1;
		int msubspace_col1;
		float r01; // motion ratio of joint 1 over joint 0
		MCoordinates C; // set by build_tree()
		bool disable_collision = true;
	};
	std::shared_ptr<Constraint> add_constraint(std::string name, std::shared_ptr<Joint> j0, std::shared_ptr<Joint> j1, float r01);

	std::shared_ptr<Constraint> add_constraint(std::string name,
		std::shared_ptr<Joint> j0, int msubspace_col0, // motion subspace column
		std::shared_ptr<Joint> j1, int msubspace_col1, float r01);

	bool build_tree();

	void integrate_velocity(float dt);

	void integrate_position(float dt);

	void project_velocity();

	void project_position();

	void move_joints();
	
	// body_id -- contact body
	// return Jacobian in body0 space
	MSubspace jacobian_0(size_t body_id);

	MCoordinates q(bool full_stack = false);

	MCoordinates dq(bool full_stack = false);

	void apply_delta_dq(MCoordinates delta_dq);

	void apply_delta_q(MCoordinates delta_q);

	JDyad H_inv(const Unitless& J);

	// recursive newton-euler algo
	void compute_bias_RNEA();

	void solve_ddq(float dt);

	void compute_H();

	void compute_H_spring(float dt); // joint space inertia matrix

	GPower compute_delta(JointType type, const MTransform& X); // positional error of loop joints

	void compute_K_k(); // joint space acceleration constraint parameters

	// joint motion subspace
	const std::map<JointType, MSubspace> S = {
		{JointType::Prismatic, subspace({{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f}})},
		{JointType::Revolute, subspace({{0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f}})},
		{JointType::Cylindrical, subspace({
			{0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f},
			{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f}})},
		{JointType::Spherical, subspace({
			{1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
			{0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f},
			{0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f}
			})},
	}; 
	// joint active force subspace
	const std::map<JointType, FSubspace> Ta = {
		{JointType::Prismatic, subspace({{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f}})},
		{JointType::Revolute, subspace({{0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f}})},
		{JointType::Cylindrical, subspace({
			{0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f},
			{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f}})},
		{JointType::Spherical, subspace({
			{1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
			{0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f},
			{0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f}
			})},
	};
	// joint constraint force subspace
	const std::map<JointType, FSubspace> T = {
		{JointType::Prismatic, subspace({
			{ 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
			{ 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f },
			{ 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f },
			{ 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f },
			{ 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f }})},
		{JointType::Revolute, subspace({
			{ 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
			{ 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f },
			{ 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f },
			{ 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f },
			{ 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f }})},
		{JointType::Cylindrical, subspace({
			{ 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
			{ 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f },
			{ 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f },
			{ 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f }})},
		{JointType::Spherical, subspace({
			{ 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f },
			{ 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f },
			{ 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f }})}
	};

	Eigen::Matrix<float, 6, Eigen::Dynamic, 0, 6, 6> subspace(std::initializer_list<std::array<float, 6>> columns);

	std::vector<std::shared_ptr<Body>> bodies;
	std::vector<std::shared_ptr<Joint>> tree_joints;
	std::vector<std::shared_ptr<Joint>> loop_joints;
	std::vector<std::shared_ptr<Constraint>> constraints; // a constraint will always create a loop
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

	bool enable_springs = false;
	Unitless Ks; // spring stiffness matrix
	Unitless Ds; // spring damping matrix

	JDyad H; // joint space inertia matrix H
	Eigen::LLT<JDyad> H_llt;
	std::shared_ptr<BlockAccess> H_acc = nullptr;

	JDyad H_spring;
	Eigen::LLT<JDyad> H_spring_llt;

	Unitless Z; // orthogonal complement of constraint 
	Unitless ZT;

	JDyad H_reduced; // Z^T H Z
	Eigen::LLT<JDyad> H_reduced_llt;

	JDyad H_spring_reduced; // H + dt^2 Ks + dt Ds
	Eigen::LLT<JDyad> H_spring_reduced_llt;

	GPower K;
	std::shared_ptr<BlockAccess> K_acc;
	GPower K_reduced; // K Z, should not multiply Z^T with K as there will be a dimension mismatch if one of the loop joints has a 2d motion space

	GPower _k;
	std::shared_ptr<BlockAccess> k_acc;

};

}