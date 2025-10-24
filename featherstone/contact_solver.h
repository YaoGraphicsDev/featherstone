#pragma once

#include "spvec.hpp"
#include "math_utils.h"
#include "rigidbody.hpp"
#include "btBulletCollisionCommon.h"

#include <map>

namespace SPD {

const static size_t max_manifold_points = 4; // dictated by bullet's collision manifold capacity of contact points
const static float restitution_threshold = 1.0f;

struct ContactSolver {
	ContactSolver() {
		old_cb = gContactDestroyedCallback;
		gContactDestroyedCallback = ContactPersistentData::destroy_persistent_data_cb;
	}
	~ContactSolver() {
		gContactDestroyedCallback = old_cb;
	}

	struct VelocityConstraintPoint {
		MTransform Xortho_0_c; // transforms from com to contact point, no rotation
		MTransform Xortho_1_c;
		MTransform Xortho_c_0; // inverse of the above two
		MTransform Xortho_c_1;
		InvDyad inv_I0; // inverse inertia at contact point
		InvDyad inv_I1;
		F3Subspace N_01; // 3x3. friction in xy direction, restitution in z direction. pointing from body 0 to body 1
		float* si_n; // accumulated sequential impulse, normal
		FCoordinates* si_t; // accumulated sequential impulse, tangential
		Unitless eff_mass; // 3x3 effective mass at contact point, first 2 elements tangential, 3rd normal
		float v_bias; // velocity bias, produced by resitution
	};

	struct VelocityConstraint {
		std::vector<VelocityConstraintPoint> cps;
		size_t id0;
		size_t id1;
		float friction_coeff;
		float restitution_coeff;
	};

	struct PositionConstraintPoint {
		Eigen::Vector3f local_p0;
		Eigen::Vector3f local_p1;
		FVector n_01;
	};
	struct PositionConstraint {
		std::vector<PositionConstraintPoint> cps;
		size_t id0;
		size_t id1;
	};

	void initialize(
		const std::vector<std::shared_ptr<RigidBody>>& bodies,
		std::shared_ptr<btCollisionDispatcher> dispatcher);

	void warm_start();

	void solve_velocity();

	void solve_position();

	// void out(std::vector<std::shared_ptr<RigidBody>>& bodies);

	std::vector<VelocityConstraint> vcs;
	std::vector<PositionConstraint> pcs;

	std::map<size_t, std::shared_ptr<RigidBody>> body_map;

	//struct Position {
	//	Eigen::Vector3f trans;
	//	Eigen::Quaternionf rot;
	//};
	//std::map<size_t, MVector> v_com; // cached velocity values at com
	//std::map<size_t, Position> pos_com; // cached position values at com
	//std::map<size_t, RigidBody::DynamicType> dyn_types; // type cache

	

	struct ContactPersistentData {
		ContactPersistentData() : si_n(0.0f), si_t(FCoordinates::Zero(2, 1)) {}

		float si_n;
		FCoordinates si_t;

		static bool destroy_persistent_data_cb(void* data) {
			delete (ContactPersistentData*)data;
			return true;
		}
	};
	ContactDestroyedCallback old_cb;
};

}