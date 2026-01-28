#pragma once
#include "articulatedbody.hpp"
#include "solver_wrapper.hpp"

#include <vector>

namespace SPD {

struct LoopJointSolver {
	void initialize(const std::vector<std::shared_ptr<ArticulatedBody>>& artbodies);

	void warm_start();

	void solve_velocity();

	void solve_position();

	struct VelocityConstraint {
		std::shared_ptr<ArticulatedBCPVelocity> bpp; // predecessor
		std::shared_ptr<ArticulatedBCPVelocity> bps; // successor
		FSubspace T_ortho; // joint force space, in orthogonal joint space
		// Unitless eff_mass_; // effective mass. space invariant.
		std::shared_ptr<InvOrPinvSolver> eff_mass_solve;
		FCoordinates* si; // sequential impulse, joint force coordinates
	};

	std::vector<VelocityConstraint> vcs;

	struct PositionConstraint {
		std::shared_ptr<ArticulatedBodyWrapper> bwp;
		std::shared_ptr<ArticulatedBodyWrapper> bws;
		Eigen::Vector3f local_pp;
		Eigen::Vector3f local_ps;
		//F3Subspace T_linear_ortho;

		// std::shared_ptr<ArticulatedBody> ab;
		int loop_joint_id;
		FSubspace T_ortho; // joint force space, in orthogonal joint space
	};

	std::vector<PositionConstraint> pcs;
};

}
