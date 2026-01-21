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
			if (lj->type == ArticulatedBody::JointType::Prismatic) {
				F3Subspace T_linear_joint(3, 2);
				T_linear_joint <<
					1, 0,
					0, 1,
					0, 0;
				pc.T_linear_ortho = XF_joint_ortho.bottomRightCorner(3, 3) * T_linear_joint;
			}
			else if (lj->type == ArticulatedBody::JointType::Revolute) {
				pc.T_linear_ortho = XF_joint_ortho.bottomRightCorner(3, 3);
			}
			else {
				assert(false);
			}
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
		Vector3f pp = pc.bwp->p_world(pc.local_pp);
		Vector3f ps = pc.bws->p_world(pc.local_ps);
		Vector3f p = (pp + ps) * 0.5f;

		std::shared_ptr<ArticulatedBCPPosition> bpp = std::make_shared<ArticulatedBCPPosition>(pc.bwp, p);
		std::shared_ptr<ArticulatedBCPPosition> bps = std::make_shared<ArticulatedBCPPosition>(pc.bws, p);

		Unitless displacement = pc.T_linear_ortho.transpose() * (ps - pp); // nx1
		// Prevent large corrections and allow slop.
		const float baumgarte = 0.3f;
		const float slop = 0.001f;
		const float max_correction = 0.5f;

		bool need_correction = false;
		for (int i = 0; i < displacement.rows(); ++i) {
			float& d = displacement(i, 0);
			if (std::abs(d) > slop) {
				need_correction = true;
				break;
			}
		}
		if (!need_correction) {
			continue;
		}

		for (int i = 0; i < displacement.rows(); ++i) {
			float& d = displacement(i, 0);
			if (d < 0.0f) {
				d = std::clamp(baumgarte * (d + slop), -max_correction, 0.0f);
			}
			else {
				d = std::clamp(baumgarte * (d - slop), 0.0f, max_correction);
			}
		}

		InvDyad inv_Ip = bpp->inv_Ic();
		InvDyad inv_Is = bps->inv_Ic();

		// TODO: is it necessary to check singularity of effective mass? see positional contact solve for reasoning
		// positional impulse
		JDyad eff_mass = pc.T_linear_ortho.transpose() * (inv_Ip.bottomRightCorner(3, 3) + inv_Is.bottomRightCorner(3, 3)) * pc.T_linear_ortho;
		InvOrPinvSolver eff_mass_solve(eff_mass);
		FCoordinates lambda = -eff_mass_solve.solve(displacement);

		FVector imp_ps = FVector::Zero(6, 1);
		imp_ps.tail(3) = pc.T_linear_ortho * lambda; // impulse pointing from predecessor to successor

		bpp->apply_positional_impulse(-imp_ps);
		bps->apply_positional_impulse(imp_ps);
	}
}

}
