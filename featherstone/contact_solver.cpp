#include "contact_solver.h"
#include "solver_wrapper.hpp"

#include <iostream>

using namespace Eigen;

namespace SPD {

void ContactSolver::initialize(
	const std::vector<std::shared_ptr<RigidBody>>& bodies,
	const std::vector<std::shared_ptr<ArticulatedBody>>& artbodies,
	std::shared_ptr<btCollisionDispatcher> dispatcher) {
	int n_man = dispatcher->getNumManifolds();
	// artbody_map.clear();

	// set up velocity constraints
 	vcs.clear();
	for (int m = 0; m < n_man; ++m) {
		btPersistentManifold* manifold = dispatcher->getManifoldByIndexInternal(m);
		int n_cps = manifold->getNumContacts();
		assert(n_cps >= 0);
		if (n_cps == 0) {
			// no effective contact point
			continue;
		}
		
		vcs.emplace_back();
		VelocityConstraint& vc = vcs.back();

		const btCollisionObject* obj0 = manifold->getBody0();
		const btCollisionObject* obj1 = manifold->getBody1();
		int id0 = obj0->getUserIndex();
		int id1 = obj1->getUserIndex();
		int sub_id0 = obj0->getUserIndex2();
		int sub_id1 = obj1->getUserIndex2();

		std::shared_ptr<BodyWrapper> b0 = nullptr;
		if (sub_id0 < 0) {
			b0 = std::make_shared<RigidBodyWrapper>(bodies[id0]);
		}
		else {
			std::shared_ptr<ArticulatedBodyWrapper> ab0 = std::make_shared<ArticulatedBodyWrapper>(artbodies[id0], sub_id0);
			b0 = ab0;
			// artbody_map[id0] = ab0->ab;
		}
		std::shared_ptr<BodyWrapper> b1 = nullptr;
		if (sub_id1 < 0) {
			b1 = std::make_shared<RigidBodyWrapper>(bodies[id1]);
		}
		else {
			std::shared_ptr<ArticulatedBodyWrapper> ab1 = std::make_shared<ArticulatedBodyWrapper>(artbodies[id1], sub_id1);
			b1 = ab1;
			// artbody_map[id1] = ab1->ab;
		}

		vc.restitution_coeff = std::min(b0->restitution_coeff(), b1->restitution_coeff());
		vc.friction_coeff = std::sqrt(b0->friction_coeff() * b1->friction_coeff());
		
		vc.cps.resize(n_cps);
		for (int c = 0; c < n_cps; ++c) {
			const btManifoldPoint& btcp = manifold->getContactPoint(c);
			if (btcp.getLifeTime() == 1) {
				// new contact
				assert(!btcp.m_userPersistentData);
				btcp.m_userPersistentData = new ContactPersistentData();
			}
			assert(btcp.m_userPersistentData);

			Vector3f p = (EV3(btcp.m_positionWorldOnA) + EV3(btcp.m_positionWorldOnB)) * 0.5f;
			VelocityConstraintPoint& cp = vc.cps[c];
			cp.bp0 = BodyContactPointVelocity::create(b0, p);
			cp.bp1 = BodyContactPointVelocity::create(b1, p);

			// work out the subspace of friction and restitution
			Vector3f n_01 = -EV3(btcp.m_normalWorldOnB).normalized(); // direction of restitution
			Quaternionf rot = Quaternionf::FromTwoVectors(Vector3f::UnitZ(), n_01);
			Vector3f tx = rot * Vector3f::UnitX();
			Vector3f ty = rot * Vector3f::UnitY();
			cp.N_01 = F3Subspace::Zero(3, 3);
			cp.N_01.col(0) = tx;
			cp.N_01.col(1) = ty;
			cp.N_01.col(2) = n_01;
			cp.si_n = &static_cast<ContactPersistentData*>(btcp.m_userPersistentData)->si_n;
			cp.si_t = &static_cast<ContactPersistentData*>(btcp.m_userPersistentData)->si_t;
			Unitless eff_mass = cp.N_01.transpose() * ((cp.bp0->inv_Ic().bottomRightCorner(3, 3) + cp.bp1->inv_Ic().bottomRightCorner(3, 3)) * cp.N_01); // TODO: effective mass matrix has to be orthogal. Try and prove it
			// tangential effective mass
			cp.eff_mass_t_solve.reset(new InvOrPinvSolver(eff_mass.topLeftCorner(2, 2)));

			// normal effective mass
			if (std::abs(eff_mass(2, 2)) < 1E-4) {
				/*Initially I thought this could never happen because : if a mechanism does not allow dof in contact normal direction,
				how did it come into contact with the surface in the first place?
				But through some experimental scenes I found that if a precise edge-edge collision occurs between 2 objects,
				detection module's behaviour can be somewhat arbitrary: generating contact normal on either face of the edge.
				Thus the normal could go in a direction that the mechanism does not have dof of
				*/

				std::cout << "velocity solve: contact normal direction DoF lost. normal effective mass = " << eff_mass(2, 2) << ", contact normal = " << n_01.transpose() << std::endl;
				// A big inverse effective mass, to generate a big impulse. Dof will not respond to it anyway.
				// Not too big, in case the resulting impulse somehow multiplies with a numerical fluctuation and explodes
				cp.eff_mass_n = 1E4;
			}
			else {
				cp.eff_mass_n = 1.0f / eff_mass(2, 2);
			}

			float v_01_n = n_01.dot(cp.bp1->vc().tail<3>() - cp.bp0->vc().tail<3>());

			cp.v_bias = v_01_n < -restitution_threshold ? v_01_n * vc.restitution_coeff : 0.0f;
		}
	}

	// set up position constraints
	pcs.clear();
	for (int m = 0; m < n_man; ++m) {
		btPersistentManifold* manifold = dispatcher->getManifoldByIndexInternal(m);
		int n_cps = manifold->getNumContacts();
		assert(n_cps >= 0);
		if (n_cps == 0) {
			// no effective contact point
			continue;
		}

		pcs.emplace_back();
		PositionConstraint& pc = pcs.back();

		const btCollisionObject* obj0 = manifold->getBody0();
		const btCollisionObject* obj1 = manifold->getBody1();		
		int id0 = obj0->getUserIndex();
		int id1 = obj1->getUserIndex();
		int sub_id0 = obj0->getUserIndex2();
		int sub_id1 = obj1->getUserIndex2();

		std::shared_ptr<BodyWrapper> b0 = nullptr;
		if (sub_id0 < 0) {
			b0 = std::make_shared<RigidBodyWrapper>(bodies[id0]);
		}
		else {
			b0 = std::make_shared<ArticulatedBodyWrapper>(artbodies[id0], sub_id0);
		}
		std::shared_ptr<BodyWrapper> b1 = nullptr;
		if (sub_id1 < 0) {
			b1 = std::make_shared<RigidBodyWrapper>(bodies[id1]);
		}
		else {
			b1 = std::make_shared<ArticulatedBodyWrapper>(artbodies[id1], sub_id1);
		}

		pc.cps.resize(n_cps);
		for (int c = 0; c < n_cps; ++c) {
			const btManifoldPoint& btcp = manifold->getContactPoint(c);
			PositionConstraintPoint& cp = pc.cps[c];
			cp.b0 = b0;
			cp.b1 = b1;
			cp.local_p0 = EV3(btcp.m_localPointA);
			cp.local_p1 = EV3(btcp.m_localPointB);
			cp.n_01 << Vector3f::Zero(), -EV3(btcp.m_normalWorldOnB).normalized();
		}
	}
}

void ContactSolver::warm_start() {
	for (VelocityConstraint& vc : vcs) {
		for (VelocityConstraintPoint& cp : vc.cps) {
			FCoordinates lambda = FCoordinates::Zero(3, 1);
			lambda << (*cp.si_t)(0), (*cp.si_t)(1), *cp.si_n;
			Vector3f imp3_01 = cp.N_01 * lambda;
			FVector imp_01;
			imp_01 << Vector3f::Zero(), imp3_01;
			cp.bp0->apply_impulse(-imp_01);
			cp.bp1->apply_impulse(imp_01);
		}
	}
}

void ContactSolver::solve_velocity() {
 	for (VelocityConstraint& vc : vcs) {
 		for (VelocityConstraintPoint& cp : vc.cps) {
			// solve tangential constraints
			// get fresh velocity values
			Vector3f v3_01 = (cp.bp1->vc() - cp.bp0->vc()).tail<3>(); // friction only works against linear velocity at contact point, 3x1

			F3Subspace t3_01 = cp.N_01.leftCols(2); // 3x2
			FCoordinates vt_01 = t3_01.transpose() * v3_01; // 2x1

			//if (std::abs(cp.eff_mass.topLeftCorner(2, 2).determinant()) < 1E-4) {
			//	std::cout << cp.eff_mass.topLeftCorner(2, 2) << std::endl;
			//	std::cout << "determinant = " << cp.eff_mass.topLeftCorner(2, 2).determinant() << std::endl;
			//	std::cout << "eigenvalue = " << cp.eff_mass.topLeftCorner(2, 2).eigenvalues() << std::endl;
			//	continue;
			//}

			// FCoordinates lambda = -cp.eff_mass_t * vt_01; // 2x1
			FCoordinates lambda = -cp.eff_mass_t_solve->solve(vt_01); // 2x1
			// sequential impulse
 			float max_friction = vc.friction_coeff * *cp.si_n;
 			FCoordinates new_impulse = *cp.si_t + lambda;
			float new_impulse_norm = new_impulse.norm();
			if (new_impulse_norm > max_friction) {
				new_impulse *= (max_friction / new_impulse_norm);
			}

			lambda = new_impulse - *cp.si_t;
			*cp.si_t = new_impulse;

			Vector3f imp3_01 = t3_01 * lambda; // 3x1
			FVector imp_01;
			imp_01 << Vector3f::Zero(), imp3_01;
			cp.bp0->apply_impulse(-imp_01);
			cp.bp1->apply_impulse(imp_01);
		}
		for (VelocityConstraintPoint& cp : vc.cps) {
			MVector v_01 = cp.bp1->vc() - cp.bp0->vc(); // get fresh velocity values
			
			Vector3f n3_01 = cp.N_01.col(2);
			float vn_01 = n3_01.dot(v_01.tail<3>());

			float lambda = -(vn_01 + cp.v_bias) * cp.eff_mass_n;
			// sequential impulse
			float new_impulse = std::max(*cp.si_n + lambda, 0.0f);
			lambda = new_impulse - *cp.si_n;
			*cp.si_n = new_impulse;

			Vector3f imp3_01 = lambda * n3_01; // impulse pointing from 0 to 1
			FVector imp_01;
			imp_01 << Vector3f::Zero(), imp3_01;
			cp.bp0->apply_impulse(-imp_01);
			cp.bp1->apply_impulse(imp_01);
		}
	}
}

//void ContactSolver::project_velocity() {
//	for (auto& p : artbody_map) {
//		p.second->project_velocity();
//	}
//}

// TODO: early-termination if positional delta is lower than a certain threshold
void ContactSolver::solve_position() {
	for (PositionConstraint& pc : pcs) {
		for (PositionConstraintPoint& cp : pc.cps) {
			Vector3f p0 = cp.b0->p_world(cp.local_p0);
			Vector3f p1 = cp.b1->p_world(cp.local_p1);
			Vector3f p = (p0 + p1) * 0.5f;
			
			std::shared_ptr<BodyContactPointPosition> bp0 = BodyContactPointPosition::create(cp.b0, p);
			std::shared_ptr<BodyContactPointPosition> bp1 = BodyContactPointPosition::create(cp.b1, p);

			float penetration = cp.n_01.tail<3>().dot(p1 - p0); // negative when penetration happens
			// Track max constraint error.
			penetration = std::min(0.0f, penetration);
			// Prevent large corrections and allow slop.
			const float baumgarte = 0.2f;
			const float slop = 0.001f;
			const float max_correction = 0.4f;

			if (penetration > -slop) {
				// no need to correct positions anymore
				continue;
			}

			penetration = std::clamp(baumgarte * (penetration + slop), -max_correction, 0.0f);

			InvDyad inv_I0 = bp0->inv_Ic(); 
			InvDyad inv_I1 = bp1->inv_Ic();

			// positional impulse
			float eff_mass = cp.n_01.dot((inv_I0 + inv_I1) * cp.n_01);
			if (std::abs(eff_mass) < 1E-4) {
				// see velocity solve initialize() for reasoning
				std::cout << "position solve: contact normal direction DoF lost. effective mass = " << eff_mass << ", contact normal = " << cp.n_01.transpose().tail<3>() << std::endl;
				eff_mass = 1E-4;
			}
			
			float lambda = -penetration / eff_mass;
			FVector imp_01 = lambda * cp.n_01; // impulse pointing from 0 to 1

			bp0->apply_positional_impulse(-imp_01);
			bp1->apply_positional_impulse(imp_01);
		}
	}
}

}