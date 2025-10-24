#include "contact_solver.h"

#include <iostream>

using namespace Eigen;

namespace SPD {

void ContactSolver::initialize(
	const std::vector<std::shared_ptr<RigidBody>>& bodies,
	std::shared_ptr<btCollisionDispatcher> dispatcher) {
	int n_man = dispatcher->getNumManifolds();

	// set up velocity constraints
	vcs.resize(n_man);
	for (int m = 0; m < n_man; ++m) {
		btPersistentManifold* manifold = dispatcher->getManifoldByIndexInternal(m);
		const btCollisionObject* obj0 = manifold->getBody0();
		const btCollisionObject* obj1 = manifold->getBody1();
		int id0 = obj0->getUserIndex(); // user index should get set when initializing collision object
		int id1 = obj1->getUserIndex();
		assert(id0 >= 0 && id0 < bodies.size());
		assert(id1 >= 0 && id1 < bodies.size());
		vcs[m].id0 = id0;
		vcs[m].id1 = id1;
		const RigidBody& b0 = *bodies[id0];
		const RigidBody& b1 = *bodies[id1];
		vcs[m].restitution_coeff = std::min(b0.restitution_coeff, b1.restitution_coeff);
		// vcs[m].restitution_coeff = 0.2f;
		vcs[m].friction_coeff = std::sqrt(b0.friction_coeff * b1.friction_coeff);
		// vcs[m].friction_coeff = 0.4f;
		
		int n_cps = manifold->getNumContacts();
		vcs[m].cps.resize(n_cps);
		for (int c = 0; c < n_cps; ++c) {
			const btManifoldPoint& btcp = manifold->getContactPoint(c);
			if (btcp.getLifeTime() == 1) {
				// new contact
				assert(!btcp.m_userPersistentData);
				btcp.m_userPersistentData = new ContactPersistentData();
			}
			assert(btcp.m_userPersistentData);

			Vector3f p = (EV3(btcp.m_positionWorldOnA) + EV3(btcp.m_positionWorldOnB)) * 0.5f;
			VelocityConstraintPoint& cp = vcs[m].cps[c];
			cp.Xortho_0_c = m_transform(Matrix3f::Identity(), Matrix3f::Identity(), p - b0.translation);
			cp.Xortho_1_c = m_transform(Matrix3f::Identity(), Matrix3f::Identity(), p - b1.translation);
			cp.Xortho_c_0 = inverse_transform(cp.Xortho_0_c);
			cp.Xortho_c_1 = inverse_transform(cp.Xortho_1_c);

			MTransform X_c_0 = m_transform(Matrix3f::Identity(), b0.rotation.toRotationMatrix(), b0.translation - p);
			MTransform X_c_1 = m_transform(Matrix3f::Identity(), b1.rotation.toRotationMatrix(), b1.translation - p);

			cp.inv_I0 = b0.type == RigidBody::DynamicType::Dynamic ? 
				transform_inv_dyad2(X_c_0, b0.inv_Ic) : InvDyad(InvDyad::Zero());
			cp.inv_I1 = b1.type == RigidBody::DynamicType::Dynamic ?
				transform_inv_dyad2(X_c_1, b1.inv_Ic) : InvDyad(InvDyad::Zero());

			MTransform Xr_0 = m_transform(Matrix3f::Identity(), b0.rotation.toRotationMatrix(), Vector3f::Zero());
			MTransform Xr_1 = m_transform(Matrix3f::Identity(), b1.rotation.toRotationMatrix(), Vector3f::Zero());

			MVector v0 = cp.Xortho_0_c * b0.v;
			MVector v1 = cp.Xortho_1_c * b1.v;

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
			cp.eff_mass = cp.N_01.transpose() * ((cp.inv_I0.bottomRightCorner(3, 3) + cp.inv_I1.bottomRightCorner(3, 3)) * cp.N_01); // TODO: effective mass matrix has to be orthogal. Try and prove it

			float v_01_n = n_01.dot(v1.tail<3>() - v0.tail<3>());
			cp.v_bias = v_01_n < -restitution_threshold ? v_01_n * vcs[m].restitution_coeff : 0.0f;
		}
	}

	// set up position constraints
	pcs.resize(n_man);
	for (int m = 0; m < n_man; ++m) {
		btPersistentManifold* manifold = dispatcher->getManifoldByIndexInternal(m);
		const btCollisionObject* obj0 = manifold->getBody0();
		const btCollisionObject* obj1 = manifold->getBody1();
		int id0 = obj0->getUserIndex(); // TODO: user index should get set when initializing collision object
		int id1 = obj1->getUserIndex();
		assert(id0 >= 0 && id0 < bodies.size());
		assert(id1 >= 0 && id1 < bodies.size());
		pcs[m].id0 = id0;
		pcs[m].id1 = id1;
		const RigidBody& b0 = *bodies[id0];
		const RigidBody& b1 = *bodies[id1];

		int n_cps = manifold->getNumContacts();
		pcs[m].cps.resize(n_cps);
		for (int c = 0; c < n_cps; ++c) {
			const btManifoldPoint& btcp = manifold->getContactPoint(c);
			pcs[m].cps[c].local_p0 = EV3(btcp.m_localPointA);
			pcs[m].cps[c].local_p1 = EV3(btcp.m_localPointB);
			pcs[m].cps[c].n_01 << Vector3f::Zero(), -EV3(btcp.m_normalWorldOnB).normalized();
		}
	}

	body_map.clear();
	for (auto& vc : vcs) {
		body_map[vc.id0] = bodies[vc.id0];
		body_map[vc.id1] = bodies[vc.id1];
	}
}

void ContactSolver::warm_start() {
	for (VelocityConstraint& vc : vcs) {
		RigidBody& b0 = *body_map[vc.id0];
		RigidBody& b1 = *body_map[vc.id1];
		for (VelocityConstraintPoint& cp : vc.cps) {
			FCoordinates lambda = FCoordinates::Zero(3, 1);
			lambda << (*cp.si_t)(0), (*cp.si_t)(1), *cp.si_n;
			Vector3f imp3_01 = cp.N_01 * lambda;
			FVector imp_01;
			imp_01 << Vector3f::Zero(), imp3_01;
			if (b0.type == RigidBody::DynamicType::Dynamic) {
				MVector dv0 = cp.Xortho_c_0 * (cp.inv_I0 * -imp_01);
				b0.v += dv0;
			}
			if (b1.type == RigidBody::DynamicType::Dynamic) {
				MVector dv1 = cp.Xortho_c_1 * (cp.inv_I1 * imp_01);
				b1.v += dv1;
			}
		}
	}
}

void ContactSolver::solve_velocity() {
	for (VelocityConstraint& vc : vcs) {
		RigidBody& b0 = *body_map[vc.id0];
		RigidBody& b1 = *body_map[vc.id1];
		for (VelocityConstraintPoint& cp : vc.cps) {
			// solve tangential constraints
			MVector v0 = cp.Xortho_0_c * b0.v; // get fresh velocity values
			MVector v1 = cp.Xortho_1_c * b1.v;
			Vector3f v3_01 = (v1 - v0).tail<3>(); // friction only works against linear velocity at contact point, 3x1
			
			F3Subspace t3_01 = cp.N_01.leftCols(2); // 3x2
			FCoordinates vt_01 = t3_01.transpose() * v3_01; // 2x1

			FCoordinates lambda = -cp.eff_mass.topLeftCorner(2, 2).inverse() * vt_01; // 2x1
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
			if (b0.type == RigidBody::DynamicType::Dynamic) {
				MVector dv0 = cp.Xortho_c_0 * (cp.inv_I0 * -imp_01);
				b0.v += dv0;
			}
			if (b1.type == RigidBody::DynamicType::Dynamic) {
				MVector dv1 = cp.Xortho_c_1 * (cp.inv_I1 * imp_01);
				b1.v += dv1;
			}
		}
		for (VelocityConstraintPoint& cp : vc.cps) {
			MVector v0 = cp.Xortho_0_c * b0.v; // get fresh velocity values
			MVector v1 = cp.Xortho_1_c * b1.v;
			MVector v_01 = v1 - v0;
			
			Vector3f n3_01 = cp.N_01.col(2);
			float vn_01 = n3_01.dot(v_01.tail<3>());
			float lambda = -(vn_01 + cp.v_bias) / cp.eff_mass(2, 2);
			// sequential impulse
			float new_impulse = std::max(*cp.si_n + lambda, 0.0f);
			lambda = new_impulse - *cp.si_n;
			*cp.si_n = new_impulse;

			Vector3f imp3_01 = lambda * n3_01; // impulse pointing from 0 to 1
			FVector imp_01;
			imp_01 << Vector3f::Zero(), imp3_01;
			if (b0.type == RigidBody::DynamicType::Dynamic) {
				MVector dv0 = cp.Xortho_c_0 * (cp.inv_I0 * -imp_01);
				b0.v += dv0;
			}
			if (b1.type == RigidBody::DynamicType::Dynamic) {
				MVector dv1 = cp.Xortho_c_1 * (cp.inv_I1 * imp_01);
				b1.v += dv1;
			}
		}
	}
}

void ContactSolver::solve_position() {
	for (PositionConstraint& pc : pcs) {
		for (PositionConstraintPoint& cp : pc.cps) {
			RigidBody& b0 = *body_map[pc.id0];
			RigidBody& b1 = *body_map[pc.id1];
			Matrix3f rot0 = b0.rotation.toRotationMatrix();
			Matrix3f rot1 = b1.rotation.toRotationMatrix();
			Vector3f p0 = rot0 * cp.local_p0 + b0.translation;
			Vector3f p1 = rot1 * cp.local_p1 + b1.translation;
			Vector3f p = (p0 + p1) * 0.5f;
			float penetration = cp.n_01.tail<3>().dot(p1 - p0); // negative when penetration happens
			// Track max constraint error.
			penetration = std::min(0.0f, penetration);
			// Prevent large corrections and allow slop.
			const float baumgarte = 0.1f;
			const float slop = 0.005f;
			const float max_correction = 0.2f;
			penetration = std::clamp(baumgarte * (penetration + slop), -max_correction, 0.0f);

			MTransform X_c_0 = m_transform(Matrix3f::Identity(), rot0, b0.translation - p); // transform from contact point to body 0
			MTransform X_c_1 = m_transform(Matrix3f::Identity(), rot1, b1.translation - p); // transform from contact point to body 1
			InvDyad inv_I0 = b0.type == RigidBody::DynamicType::Static ? InvDyad::Zero() : transform_inv_dyad2(X_c_0, b0.inv_Ic);
			InvDyad inv_I1 = b1.type == RigidBody::DynamicType::Static ? InvDyad::Zero() : transform_inv_dyad2(X_c_1, b1.inv_Ic);

			// positional impulse
			float lambda = -(penetration) / cp.n_01.dot((inv_I0 + inv_I1) * cp.n_01);
			MVector imp_01 = lambda * cp.n_01; // impulse pointing from 0 to 1

			MTransform Xortho_0_c = m_transform(Matrix3f::Identity(), Matrix3f::Identity(), p - b0.translation); // transform from body 0 to contact point
			MTransform Xortho_1_c = m_transform(Matrix3f::Identity(), Matrix3f::Identity(), p - b1.translation); // transform from body 1 to contact point

			if (b0.type == RigidBody::DynamicType::Dynamic) {
				MVector dp = inverse_transform(Xortho_0_c) * (inv_I0 * -imp_01);
				Vector3f d_translation = dp.tail<3>();
				Vector3f d_rotation = dp.head<3>();

				b0.translation += d_translation;
				float d_rotation_norm = d_rotation.norm();
				if (d_rotation_norm > 1e-5) {
					Quaternionf q_rotation(AngleAxisf(d_rotation_norm, d_rotation / d_rotation_norm));
					b0.rotation = q_rotation * b0.rotation;
				}
			}
			if (b1.type == RigidBody::DynamicType::Dynamic) {
				MVector dp = inverse_transform(Xortho_1_c) * (inv_I1 * imp_01);
				Vector3f d_translation = dp.tail<3>();
				Vector3f d_rotation = dp.head<3>();

				b1.translation += d_translation;
				float d_rotation_norm = d_rotation.norm();
				if (d_rotation_norm > 1e-5) {
					Quaternionf q_rotation(AngleAxisf(d_rotation_norm, d_rotation / d_rotation_norm));
					b1.rotation = q_rotation * b1.rotation;
				}
			}
		}
	}
}

}