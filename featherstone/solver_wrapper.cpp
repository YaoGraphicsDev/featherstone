#include "solver_wrapper.hpp"
#include <iostream>

using namespace Eigen;

namespace SPD {

std::shared_ptr<BodyContactPointVelocity> BodyContactPointVelocity::create(std::shared_ptr<BodyWrapper> b, const Eigen::Vector3f& pc) {
	if (b->type == BodyWrapper::Type::Rigid) {
		return std::make_shared<RigidBCPVelocity>(b, pc);
	}
	else if (b->type == BodyWrapper::Type::Articulated) {
		return std::make_shared<ArticulatedBCPVelocity>(b, pc);
	}
	else {
		assert(false);
		return nullptr;
	}
}

std::shared_ptr<BodyContactPointPosition> BodyContactPointPosition::create(std::shared_ptr<BodyWrapper> b, const Eigen::Vector3f& pc) {
	if (b->type == BodyWrapper::Type::Rigid) {
		return std::make_shared<RigidBCPPosition>(b, pc);
	}
	else if (b->type == BodyWrapper::Type::Articulated) {
		return std::make_shared<ArticulatedBCPPosition>(b, pc);
	}
	else {
		assert(false);
		return nullptr;
	}
}


RigidBCPVelocity::RigidBCPVelocity(std::shared_ptr<BodyWrapper> b, const Eigen::Vector3f& pc) {
	assert(b->type == BodyWrapper::Type::Rigid);
	bw = std::static_pointer_cast<RigidBodyWrapper>(b);
	if (bw->rb->type == RigidBody::DynamicType::Dynamic) {
		Xortho_com_c = m_transform(Matrix3f::Identity(), Matrix3f::Identity(), pc - bw->rb->translation);
		Xortho_c_com = inverse_transform(Xortho_com_c);
		X_c_com = m_transform(Eigen::Matrix3f::Identity(), bw->rb->rotation.toRotationMatrix(), bw->rb->translation - pc); // from contact point to com
		_inv_Ic = transform_inv_dyad2(X_c_com, bw->rb->inv_I);
	}
	else if (bw->rb->type == RigidBody::DynamicType::Static) {
		_inv_Ic = InvDyad::Zero();
	}
	else {
		assert(false);
	}
}

MVector RigidBCPVelocity::vc() {
	if (bw->rb->type == RigidBody::DynamicType::Dynamic) {
		MVector v0 = Xortho_com_c * bw->rb->v;
		return v0;
	}
	else if (bw->rb->type == RigidBody::DynamicType::Static) {
		return MVector::Zero(6, 1);
	}
	else {
		assert(false);
	}
}

void RigidBCPVelocity::apply_impulse(const FVector& imp_c) {
	if (bw->rb->type == RigidBody::DynamicType::Dynamic) {
		MVector dv = Xortho_c_com * (_inv_Ic * imp_c);
		bw->rb->v += dv;
	}
}


ArticulatedBCPVelocity::ArticulatedBCPVelocity(std::shared_ptr<BodyWrapper> b, const Vector3f& pc) {
	assert(b->type == BodyWrapper::Type::Articulated);
	bw = std::static_pointer_cast<ArticulatedBodyWrapper>(b);
	if (bw->id > 0) {
		MTransform X_w_c = m_transform(Eigen::Matrix3f::Identity(), Eigen::Matrix3f::Identity(), pc); // from world to contact point
		MSubspace J0 = bw->ab->jacobian_0(bw->id);
		Jc = X_w_c * bw->ab->X_0_w * J0;
		inv_Ig = bw->ab->H_inv(Jc.transpose());
		_inv_Ic = Jc * inv_Ig;
	}
	else {
		_inv_Ic = InvDyad::Zero();
	}
}


MVector ArticulatedBCPVelocity::vc() {
	if (bw->id > 0) {
		MCoordinates dq = bw->ab->dq(bw->id);
		assert(Jc.cols() == dq.rows());
		MVector vc = Jc * dq;
		return vc;
	}
	else {
		return MVector::Zero(6, 1);
	}
}

void ArticulatedBCPVelocity::apply_impulse(const FVector& imp_c) {
	if (bw->id > 0) {
		MCoordinates delta_dq = inv_Ig * imp_c;
		bw->ab->apply_delta_dq(delta_dq);
	}
}


RigidBCPPosition::RigidBCPPosition(std::shared_ptr<BodyWrapper> b, const Eigen::Vector3f& pc) {
	assert(b->type == BodyWrapper::Type::Rigid);
	bw = std::static_pointer_cast<RigidBodyWrapper>(b);
	
	if (bw->rb->type == RigidBody::DynamicType::Dynamic) {
		Xortho_c_com = m_transform(Matrix3f::Identity(), Matrix3f::Identity(), bw->rb->translation - pc);
		Matrix3f bases = bw->rb->rotation.toRotationMatrix();
		MTransform X_c_com = m_transform(Matrix3f::Identity(), bases, bw->rb->translation - pc);
		_inv_Ic = transform_inv_dyad2(X_c_com, bw->rb->inv_I);
	}
	else {
		_inv_Ic = InvDyad::Zero();
	}
}

void RigidBCPPosition::apply_positional_impulse(const FVector& imp_c) {
	if (bw->rb->type == RigidBody::DynamicType::Dynamic) {
		MVector dp = Xortho_c_com * (_inv_Ic * imp_c);
		Vector3f d_translation = dp.tail<3>();
		Vector3f d_rotation = dp.head<3>();

		bw->rb->translation += d_translation;
		float d_rotation_norm = d_rotation.norm();
		if (d_rotation_norm > 1e-5) {
			Quaternionf q_rotation(AngleAxisf(d_rotation_norm, d_rotation / d_rotation_norm));
			bw->rb->rotation = q_rotation * bw->rb->rotation;
		}
	}
}

ArticulatedBCPPosition::ArticulatedBCPPosition(std::shared_ptr<BodyWrapper> b, const Eigen::Vector3f& pc) {
	assert(b->type == BodyWrapper::Type::Articulated);
	bw = std::static_pointer_cast<ArticulatedBodyWrapper>(b);
	if (bw->id > 0) {
		bw->ab->compute_H();

		MTransform X_w_c = m_transform(Eigen::Matrix3f::Identity(), Eigen::Matrix3f::Identity(), pc); // from world to contact point
		MSubspace J0 = bw->ab->jacobian_0(bw->id);
		MSubspace Jc = X_w_c * bw->ab->X_0_w * J0; // Jacobian at contact point
		inv_Ig = bw->ab->H_inv(Jc.transpose());
		_inv_Ic = Jc * inv_Ig;
	}
	else {
		_inv_Ic = InvDyad::Zero();
	}
}

void ArticulatedBCPPosition::apply_positional_impulse(const FVector& imp_c) {
	if (bw->id > 0) {
		MCoordinates delta_q = inv_Ig * imp_c;
		bw->ab->apply_delta_q(delta_q);
		bw->ab->move_joints();
		bw->ab->project_position();
	}
}

}