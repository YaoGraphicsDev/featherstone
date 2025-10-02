#pragma once

#include "spshapes.hpp"

#include <memory>
#include <vector>
#include <map>
#include <set>

#include "Eigen/Core"
#include "glm/gtc/quaternion.hpp"

namespace SPD{

struct ArticulatedBody {
	struct Constraint;
	struct Body {
		// added with rigidbody
		std::shared_ptr<Shape> shape = nullptr;

		// values determined by the time constraint is set
		std::shared_ptr<Constraint> parent_joint = nullptr;
		std::vector<std::shared_ptr<Constraint>> children_joints;
		Dyad I = Dyad::Identity();

		// values set when building dynamic tree
		int id = 0;

		// values updated every frame
		Eigen::Quaternionf rotation = Eigen::Quaternionf::Identity();
		Eigen::Vector3f translation = Eigen::Vector3f::Zero();
		Eigen::Matrix3f bases = rotation.toRotationMatrix(); // body space bases in world space.
		MVector v = MVector::Zero();
		MVector a = MVector::Zero();

		// provided by eternal source
		FVector fe0 = FVector::Zero(); // external force, in body0 space
	};

	ArticulatedBody() {
		// add an invisible fixed shape
		bodies = { std::make_shared<Body>() };
	}

	ArticulatedBody(std::shared_ptr<Shape> shape, Eigen::Quaternionf rotation, Eigen::Vector3f translation) {
		add_body(shape, rotation, translation);
	}

	~ArticulatedBody() {};

	std::shared_ptr<Body> base() {
		assert(bodies.size() >= 1);
		return bodies.front();
	}

	void set_gravity(Eigen::Vector3f gravity) {
		this->gravity = gravity;
	}

	std::shared_ptr<Body> add_body(std::shared_ptr<Shape> shape, Eigen::Quaternionf rotation, Eigen::Vector3f translation);

	enum class ConstraintType {
		Revolute,
		Prismatic,
		Ball,
		Free
	};
	struct Constraint {
		ConstraintType type;
		std::shared_ptr<Body> b0;
		std::shared_ptr<Body> b1;
		Eigen::Matrix3f bb0; // bases of joint space, in body space 0
		Eigen::Vector3f bt0; // translation of joint space, in body space 0
		Eigen::Matrix3f bb1; // bases of joint space, in body space 1
		Eigen::Vector3f bt1; // translation of joint space, in body space 1
		MTransform X_0_J0; // transform from body 0 space to joint space 0
		MTransform X_1_J1; // transform from body 1 space to joint space 1
		MTransform X_J0_J1; // transform from joint 0 to joint space 1  TODO: didnt get updated properly
		MSubspace S;
		MCoordinates q;
		MCoordinates dq;
		MCoordinates ddq;
		FCoordinates bias; // joint space bias force

		FCoordinates taue; // external joint force, set by external source
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

	// set b0 to nullptr to connect body to a base
	std::shared_ptr<Constraint> add_constraint(
		ConstraintType type,
		std::shared_ptr<Body> b0,
		std::shared_ptr<Body> b1,
		Eigen::Matrix3f base0,
		Eigen::Vector3f trans0);

	bool build_tree();

	void step(float dt);

	void set_constraint_status(
		std::shared_ptr<Constraint> c,
		std::initializer_list<float> ql,
		std::initializer_list<float> dql = {},
		std::initializer_list<float> ddql = {});

	void move_constraints(float dt);

	// update bodies from joint space velocity and acceleration
	void move_bodies();

	// recursive newton-euler algo
	void compute_bias_RNEA();

	void configure_H();

	void solve_ddq();

	// TODO: temporary
	// void apply_velo_friction();

	Eigen::Ref<JDyad> H_block(int row, int col);

	void compute_H(); // joint space inertia matrix

	template<typename F, typename T>
	std::vector<const F*> field_array(const std::vector<std::shared_ptr<T>>& arr, size_t offset) {
		std::vetor<const F*> 
	}

	std::vector<std::shared_ptr<Body>> bodies;
	std::vector<std::shared_ptr<Constraint>> joints;
	std::vector<int> lambda;
	std::vector<std::set<int>> mu;
	std::vector<std::set<int>> nu;

	std::vector<MTransform> X_0_; // Set by RNEA. See RNEA for details
	std::vector<MTransform> X_Li_;

	Eigen::Vector3f gravity;
	JDyad H; // joint space inertia matrix H
	struct HBlockRange {
		int start_row;
		int start_col;
		int rows;
		int cols;
	};
	std::vector<std::vector<HBlockRange>> H_blocks;
};

}