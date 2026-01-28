#include "loop_joint_solver.h"

#include <iostream>

using namespace Eigen;

namespace SPD {



void LoopJointSolver::initialize(const std::vector<std::shared_ptr<ArticulatedBody>>& artbodies) {
	vcs.clear();
	pcs.clear();

	for (std::shared_ptr<ArticulatedBody> body : artbodies) {
		for (int i = 0; i < body->loop_joints.size(); ++i) {
			auto lj = body->loop_joints[i];
			std::shared_ptr<ArticulatedBody::Body> pb = lj->b0; // predecessor body
			std::shared_ptr<ArticulatedBody::Body> sb = lj->b1; // succcessor body
			std::shared_ptr<ArticulatedBodyWrapper> bwp = std::make_shared<ArticulatedBodyWrapper>(body, pb->id);
			std::shared_ptr<ArticulatedBodyWrapper> bws = std::make_shared<ArticulatedBodyWrapper>(body, sb->id);

			VelocityConstraint vc;
			vc.bpp = std::make_shared<ArticulatedBCPVelocity>(bwp, pb->bases * lj->bt0 + pb->translation);
			vc.bps = std::make_shared<ArticulatedBCPVelocity>(bws, sb->bases * lj->bt1 + sb->translation);


			MTransform XM_ortho_joint = m_transform(Matrix3f::Identity(), pb->bases * lj->bb0, Vector3f::Zero());
			FTransform XF_joint_ortho = transpose_transform(XM_ortho_joint);
			vc.T_ortho = XF_joint_ortho * *lj->T;
			Unitless eff_mass = vc.T_ortho.transpose() * ((vc.bpp->inv_Ic() + vc.bps->inv_Ic()) * vc.T_ortho);

			vc.eff_mass_solve.reset(new InvOrPinvSolver(eff_mass));

			vc.si = &body->SI[i];
			vcs.push_back(vc);

			PositionConstraint pc;
			pc.bwp = bwp;
			pc.bws = bws;
			pc.local_pp = lj->bt0;
			pc.local_ps = lj->bt1;
			pc.loop_joint_id = i;
			pc.T_ortho = vc.T_ortho;

			pcs.push_back(pc);
		}
	}
}

void LoopJointSolver::warm_start() {
	for (const VelocityConstraint& vc : vcs) {
		FCoordinates imp_ortho_ps = vc.T_ortho * *vc.si; // impulse from predecessor to successor
		vc.bpp->apply_impulse(-imp_ortho_ps);
		vc.bps->apply_impulse(imp_ortho_ps);
	}
}

void LoopJointSolver::solve_velocity() {
	for (VelocityConstraint& vc : vcs) {
		// get fresh velocity values
		MVector v_ortho_ps = vc.bps->vc() - vc.bpp->vc();

		FCoordinates lambda = -vc.eff_mass_solve->solve(vc.T_ortho.transpose() * v_ortho_ps); // 2x1


		// sequential impulse
		*vc.si += lambda;

		FVector imp_ortho_ps = vc.T_ortho * lambda; // impulse pointing from 0 to 1
		vc.bpp->apply_impulse(-imp_ortho_ps);
		vc.bps->apply_impulse(imp_ortho_ps);
	}
}

void LoopJointSolver::solve_position() {
	for (PositionConstraint& pc : pcs) {
		assert(pc.bwp->ab.get() == pc.bws->ab.get());
		ArticulatedBCPPosition::LoopClosureDisplacement displacement =
			ArticulatedBCPPosition::loop_closure_displacement_ps(pc.bwp->ab, pc.loop_joint_id);
		
		// Prevent large corrections and allow slop.
		const float baumgarte = 0.2f;
		const float slop = 0.001f;
		const float slop2 = 0.000001f;
		const float max_correction = 0.4f;

		bool need_correction = false;

		// rotational error
		Vector3f rot_err = Vector3f::Zero();
		float angle = displacement.ang_axis.angle();
		if (std::abs(angle) > slop) {
			need_correction |= true;
			if (angle < 0.0f) {
				angle = std::clamp(baumgarte * (angle + slop), -max_correction, 0.0f);
			}
			else {
				angle = std::clamp(baumgarte * (angle - slop), 0.0f, max_correction);
			}
			rot_err = displacement.ang_axis.axis() * angle;
		}

		// linear error
		Vector3f linear_err = Vector3f::Zero();
		// Vector3f linear_err = displacement.linear;
		float linear_err_norm = displacement.linear.norm();
		if (linear_err_norm > slop) {
			need_correction |= true;
			Vector3f linear_err_unit = displacement.linear / linear_err_norm;
			linear_err_norm = std::clamp(baumgarte * (linear_err_norm - slop), 0.0f, max_correction);
			linear_err = linear_err_unit * linear_err_norm;
		}

		if (!need_correction) {
			continue;
		}

		MVector err6;
		err6 << rot_err, linear_err; // 6x1

		Unitless err_proj = pc.T_ortho.transpose() * err6; // nx1

		Vector3f pp = pc.bwp->p_world(pc.local_pp);
		Vector3f ps = pc.bws->p_world(pc.local_ps);
		Vector3f p = (pp + ps) * 0.5f;

		std::shared_ptr<ArticulatedBCPPosition> bpp = std::make_shared<ArticulatedBCPPosition>(pc.bwp, p);
		std::shared_ptr<ArticulatedBCPPosition> bps = std::make_shared<ArticulatedBCPPosition>(pc.bws, p);

		InvDyad inv_Ip = bpp->inv_Ic();
		InvDyad inv_Is = bps->inv_Ic();

		// TODO: is it necessary to check singularity of effective mass? see positional contact solve for reasoning
		// positional impulse
		JDyad eff_mass = pc.T_ortho.transpose() * (inv_Ip + inv_Is) * pc.T_ortho; // nxn
 		InvOrPinvSolver eff_mass_solve(eff_mass);
		FCoordinates lambda = -eff_mass_solve.solve(err_proj); // nx1

		FVector imp_ps = pc.T_ortho * lambda; // 6x1, impulse pointing from predecessor to successor

		bpp->apply_positional_impulse(-imp_ps);
		bps->apply_positional_impulse(imp_ps);
	}
}

}
