#pragma once

#include "spvec.hpp"
#include "math_utils.h"
#include "rigidbody.hpp"
#include "articulatedbody.hpp"
#include "solver_wrapper.hpp"
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
		//MTransform Xortho_0_c; // transforms from com to contact point, no rotation
		//MTransform Xortho_1_c;
		//MTransform Xortho_c_0; // inverse of the above two
		//MTransform Xortho_c_1;
		//MSubspace Jc_0;
		//MSubspace Jc_1;
		//InvDyad inv_I0; // inverse inertia at contact point
		//InvDyad inv_I1;
		std::shared_ptr<BodyContactPointVelocity> bp0;
		std::shared_ptr<BodyContactPointVelocity> bp1;
		F3Subspace N_01; // 3x3. friction in xy direction, restitution in z direction. pointing from body 0 to body 1
		float* si_n; // accumulated sequential impulse, normal
		FCoordinates* si_t; // accumulated sequential impulse, tangential
		// Eigen::Matrix3f eff_mass;
		std::shared_ptr<InvOrPinvSolver> eff_mass_t_solve; // 2x2 tengential effective mass at contact point
		// Unitless eff_mass_t; 
		float eff_mass_n; // normal effective mass at contact point
		float v_bias; // velocity bias, produced by resitution
	};

	struct VelocityConstraint {
		std::vector<VelocityConstraintPoint> cps;
		float friction_coeff;
		float restitution_coeff;
	};

	struct PositionConstraintPoint {
		std::shared_ptr<BodyWrapper> b0;
		std::shared_ptr<BodyWrapper> b1;
		Eigen::Vector3f local_p0;
		Eigen::Vector3f local_p1;
		FVector n_01;
	};
	struct PositionConstraint {
		std::vector<PositionConstraintPoint> cps;
	};

	void initialize(
		const std::vector<std::shared_ptr<RigidBody>>& bodies,
		const std::vector<std::shared_ptr<ArticulatedBody>>& artbodies,
		std::shared_ptr<btCollisionDispatcher> dispatcher);

	void warm_start();

	void solve_velocity();

	void solve_position();

	// void project_velocity();

	// void out(std::vector<std::shared_ptr<RigidBody>>& bodies);

	std::vector<VelocityConstraint> vcs;
	std::vector<PositionConstraint> pcs;

	// std::map<size_t, std::shared_ptr<RigidBody>> body_map;
	// std::map<size_t, std::shared_ptr<ArticulatedBody>> artbody_map;

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