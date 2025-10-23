#include "contact_solver.h"

#include <iostream>

using namespace Eigen;

namespace SPD {

static Vector3f r_static;

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
		vcs[m].restitution_coeff = 0.9f;
		vcs[m].friction_coeff = std::sqrt(b0.friction_coeff * b1.friction_coeff);
		// vcs[m].friction_coeff = 0.2f;
		
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
			std::cout << "d = " << (p - b0.translation).transpose() << std::endl;
			std::cout << "Xortho_0_c = " << std::endl;
			std::cout << cp.Xortho_0_c << std::endl;
			cp.Xortho_1_c = m_transform(Matrix3f::Identity(), Matrix3f::Identity(), p - b1.translation);

			MTransform X_c_0 = m_transform(Matrix3f::Identity(), b0.rotation.toRotationMatrix(), b0.translation - p);
			MTransform X_c_1 = m_transform(Matrix3f::Identity(), b1.rotation.toRotationMatrix(), b1.translation - p);

			// TODO: temp
			r_static = p - b0.translation;
			std::cout << "r_static = " << r_static.transpose() << std::endl;

			cp.inv_I0 = b0.type == RigidBody::DynamicType::Dynamic ? 
				transform_inv_dyad2(X_c_0, b0.shape->sp_inv_Ic) * b0.inv_mass : InvDyad(InvDyad::Zero());
			cp.inv_I1 = b1.type == RigidBody::DynamicType::Dynamic ?
				transform_inv_dyad2(X_c_1, b1.shape->sp_inv_Ic) * b1.inv_mass : InvDyad(InvDyad::Zero());

			MTransform Xr_0 = m_transform(Matrix3f::Identity(), b0.rotation.toRotationMatrix(), Vector3f::Zero());
			MTransform Xr_1 = m_transform(Matrix3f::Identity(), b1.rotation.toRotationMatrix(), Vector3f::Zero());
			cp.inv_Iortho0 = b0.type == RigidBody::DynamicType::Dynamic ?
				transform_inv_dyad2(Xr_0, b0.shape->sp_inv_Ic) * b0.inv_mass : InvDyad(InvDyad::Zero());
			cp.inv_Iortho1 = b1.type == RigidBody::DynamicType::Dynamic ?
				transform_inv_dyad2(Xr_1, b1.shape->sp_inv_Ic) * b1.inv_mass : InvDyad(InvDyad::Zero());

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
			
			FSubspace N_01 = FSubspace::Zero(6, 3);
			N_01.bottomRows(3) = cp.N_01;
			Unitless eff_mass = N_01.transpose() * ((cp.inv_I0 + cp.inv_I1) * N_01);
			std::cout << "eff_mass = " << std::endl;
			std::cout << eff_mass << std::endl;
			std::cout << "eff_mass inverse = " << std::endl;
			std::cout << eff_mass.inverse() << std::endl;
			std::cout << "eff_mass top left 2x2 corner inverse" << std::endl;
			std::cout << eff_mass.topLeftCorner(2, 2).inverse() << std::endl;

			FSubspace N_01_xy = N_01.leftCols(2);
			Unitless eff_mass_xy = N_01_xy.transpose() * ((cp.inv_I0 + cp.inv_I1) * N_01_xy);
			std::cout << "eff_mass_xy = " << std::endl;
			std::cout << eff_mass_xy << std::endl;
			std::cout << "eff_mass_xy inverse = " << std::endl;
			std::cout << eff_mass_xy.inverse() << std::endl;

			FSubspace N_01_z = N_01.col(2);
			Unitless eff_mass_z = N_01_z.transpose() * ((cp.inv_I0 + cp.inv_I1) * N_01_z);
			std::cout << "eff_mass_z = " << std::endl;
			std::cout << eff_mass_z << std::endl;
			std::cout << "eff_mass_z inverse = " << std::endl;
			std::cout << eff_mass_z.inverse() << std::endl;
			

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
		std::cout << "\twarm start. id0 = " << vc.id0 << ", id1 = " << vc.id1 << std::endl;
		for (VelocityConstraintPoint& cp : vc.cps) {
			std::cout << "\t\tN_01 col0 = " << cp.N_01.col(0).transpose() << std::endl;
			std::cout << "\t\tN_01 col1 = " << cp.N_01.col(1).transpose() << std::endl;
			std::cout << "\t\tN_01 col2 = " << cp.N_01.col(2).transpose() << std::endl;
			FCoordinates lambda = FCoordinates::Zero(3, 1);
			lambda << (*cp.si_t)(0), (*cp.si_t)(1), *cp.si_n;
			std::cout << "\t\tlambda = " << lambda.transpose() << std::endl;
			Vector3f imp3_01 = cp.N_01 * lambda;
			std::cout << "\t\timp3_01 = " << imp3_01.transpose() << std::endl;
			FVector imp_01;
			imp_01 << Vector3f::Zero(), imp3_01;
			if (b0.type == RigidBody::DynamicType::Dynamic) {
				MVector dv0 = cp.inv_Iortho0 * cp.Xortho_0_c.transpose() * (-imp_01);
				MVector dv0_ = inverse_transform(cp.Xortho_0_c) * (cp.inv_I0 * -imp_01);
				std::cout << "\t\tdv0 = " << dv0.transpose() << std::endl;
				std::cout << "\t\tdv0_ = " << dv0_.transpose() << std::endl;
				b0.v += dv0;
				std::cout << "\t\tb0.v = " << b0.v.transpose() << std::endl;
			}
			if (b1.type == RigidBody::DynamicType::Dynamic) {
				MVector dv1 = inverse_transform(cp.Xortho_1_c) * (cp.inv_I1 * imp_01);
				b1.v += dv1;
			}
		}
	}
}

void ContactSolver::solve_velocity() {
	for (VelocityConstraint& vc : vcs) {
		RigidBody& b0 = *body_map[vc.id0];
		RigidBody& b1 = *body_map[vc.id1];
		std::cout << "\t\tsolve velocity. id0 = " << vc.id0 << ", id1 = " << vc.id1 << std::endl;
		for (VelocityConstraintPoint& cp : vc.cps) {
			// solve tangential constraints
			std::cout << "\t\ttangent solve" << std::endl;
			std::cout << "\t\t\tb0.v = " << b0.v.transpose() << std::endl;
			MVector v0 = cp.Xortho_0_c * b0.v; // get fresh velocity values
			MVector v1 = cp.Xortho_1_c * b1.v;
			Vector3f v3_01 = (v1 - v0).tail<3>(); // friction only works against linear velocity at contact point, 3x1
			std::cout << "\t\t\tlinear v = " << v3_01.transpose() << std::endl;
			
			// TODO: temp
			Vector3f omega_com_01 = (b1.v - b0.v).head<3>();
			Vector3f linear_com_01 = (b1.v - b0.v).tail<3>();
			std::cout << "\t\t\tomega = " << omega_com_01.transpose() << std::endl;
			Vector3f linear_produced = omega_com_01.cross(r_static);
			std::cout << "\t\t\tvelocity produced by omega = " << linear_produced.transpose() << std::endl;
			std::cout << "\t\t\tadd it to linear  = " << (linear_produced + linear_com_01).transpose() << std::endl;

			// F3Subspace t3_01 = F3Subspace::Zero(3, 2);
			F3Subspace t3_01 = cp.N_01.leftCols(2); // 3x2
			std::cout << "\t\t\tx = " << t3_01.col(0).transpose() << std::endl;
			std::cout << "\t\t\ty = " << t3_01.col(1).transpose() << std::endl;
			FCoordinates vt_01 = t3_01.transpose() * v3_01; // 2x1
			std::cout << "\t\t\tvelocity components = " << vt_01.transpose() << std::endl;

			// TODO: which expression is correct?
			// Matrix2f inv0 = cp.eff_mass.topLeftCorner(2, 2).inverse();
			// Matrix2f inv1 = cp.eff_mass.inverse().topLeftCorner(2, 2);
			FCoordinates lambda = -cp.eff_mass.topLeftCorner(2, 2).inverse() * vt_01; // 2x1
			// FCoordinates lambda = -cp.eff_mass.inverse().topLeftCorner(2, 2) * vt_01; // 2x1
			
			std::cout << "\t\t\tcomputed lambda = " << lambda.transpose() << std::endl;
			// sequential impulse
			std::cout << "\t\t\tsi_n = " << *cp.si_n << std::endl;
			float max_friction = vc.friction_coeff * *cp.si_n;
			std::cout << "\t\t\tsi_t = " << (*cp.si_t).transpose() << std::endl;
 			FCoordinates new_impulse = *cp.si_t + lambda;
			float new_impulse_norm = new_impulse.norm();
			if (new_impulse_norm > max_friction) {
				new_impulse *= (max_friction / new_impulse_norm);
			}
			lambda = new_impulse - *cp.si_t;
			std::cout << "\t\t\tworking impulse = " << lambda.transpose() << std::endl;
			*cp.si_t = new_impulse;

			Vector3f imp3_01 = t3_01 * lambda; // 3x1
			FVector imp_01;
			imp_01 << Vector3f::Zero(), imp3_01;
			std::cout << "\t\t\timp_01 = " << imp_01.transpose() << std::endl;
			if (b0.type == RigidBody::DynamicType::Dynamic) {
				MVector dv0 = inverse_transform(cp.Xortho_0_c) * (cp.inv_I0 * -imp_01);
				std::cout << "\t\t\tdv0 = " << dv0.transpose() << std::endl;
				b0.v += dv0;
				std::cout << "\t\t\tb0.v = " << b0.v.transpose() << std::endl;
			}
			if (b1.type == RigidBody::DynamicType::Dynamic) {
				MVector dv1 = inverse_transform(cp.Xortho_1_c) * (cp.inv_I1 * imp_01);
				b1.v += dv1;
			}
		}
		for (VelocityConstraintPoint& cp : vc.cps) {
			std::cout << "\t\tnormal solve" << std::endl;
			// solve normal constraints
			std::cout << "\t\t\tb0.v = " << b0.v.transpose() << std::endl;
			MVector v0 = cp.Xortho_0_c * b0.v; // get fresh velocity values
			MVector v1 = cp.Xortho_1_c * b1.v;
			MVector v_01 = v1 - v0;
			
			Vector3f n3_01 = cp.N_01.col(2);
			float vn_01 = n3_01.dot(v_01.tail<3>());
			std::cout << "\t\t\tvn_01 = " << vn_01 << std::endl;
			std::cout << "\t\t\tv_bias = " << cp.v_bias << std::endl;
			float lambda = -(vn_01 + cp.v_bias) / cp.eff_mass(2, 2);
			std::cout << "\t\t\tcomputed lambda = " << lambda << std::endl;
			// sequential impulse
			std::cout << "\t\t\tsi_n = " << *cp.si_n << std::endl;
			float new_impulse = std::max(*cp.si_n + lambda, 0.0f);
			lambda = new_impulse - *cp.si_n;
			std::cout << "\t\t\tworking impulse = " << lambda << std::endl;
			*cp.si_n = new_impulse;

			Vector3f imp3_01 = lambda * n3_01; // impulse pointing from 0 to 1
			FVector imp_01;
			imp_01 << Vector3f::Zero(), imp3_01;
			std::cout << "\t\t\timp_01 = " << imp_01.transpose() << std::endl;
			if (b0.type == RigidBody::DynamicType::Dynamic) {
				MVector dv0 = cp.inv_Iortho0 * cp.Xortho_0_c.transpose()* (-imp_01);
				MVector dv0_ = inverse_transform(cp.Xortho_0_c) * (cp.inv_I0 * -imp_01); //TODO: cp.Xortho_0_c transform is carried out directly and inversely. Redundant
				std::cout << "\t\t\tdv0 = " << dv0.transpose() << std::endl;
				std::cout << "\t\t\tdv0_ = " << dv0_.transpose() << std::endl;
				b0.v += dv0;
				std::cout << "\t\t\tb0.v = " << b0.v.transpose() << std::endl;
			}
			if (b1.type == RigidBody::DynamicType::Dynamic) {
				MVector dv1 = inverse_transform(cp.Xortho_1_c) * (cp.inv_I1 * imp_01);
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
			// TODO: make these parameters constants
			const float baumgarte = 0.1f;
			const float slop = 0.005f;
			const float max_correction = 0.2f;
			penetration = std::clamp(baumgarte * (penetration + slop), -max_correction, 0.0f);

			MTransform X_c_0 = m_transform(Matrix3f::Identity(), rot0, b0.translation - p); // transform from contact point to body 0
			MTransform X_c_1 = m_transform(Matrix3f::Identity(), rot1, b1.translation - p); // transform from contact point to body 1
			InvDyad inv_I0 = b0.type == RigidBody::DynamicType::Static ? InvDyad::Zero() : transform_inv_dyad2(X_c_0, b0.shape->sp_inv_Ic);
			InvDyad inv_I1 = b1.type == RigidBody::DynamicType::Static ? InvDyad::Zero() : transform_inv_dyad2(X_c_1, b1.shape->sp_inv_Ic);

			// positional impulse
			float lambda = -(penetration) / cp.n_01.dot((inv_I0 + inv_I1) * cp.n_01);
			MVector imp_01 = lambda * cp.n_01; // impulse pointing from 0 to 1

			MTransform Xortho_0_c = m_transform(Matrix3f::Identity(), Matrix3f::Identity(), p - b0.translation); // transform from body 0 to contact point
			MTransform Xortho_1_c = m_transform(Matrix3f::Identity(), Matrix3f::Identity(), p - b1.translation); // transform from body 1 to contact point

			if (b0.type == RigidBody::DynamicType::Dynamic) {
				// TODO: take mass into account
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
				// TODO: take mass into account
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