
#pragma once

#include "rigidbody.hpp"
#include "articulatedbody.hpp"

namespace SPD {

struct BodyWrapper {
	enum Type {
		Default,
		Rigid,
		Articulated,
	};
	virtual float restitution_coeff() = 0;
	virtual float friction_coeff() = 0;
	virtual Eigen::Vector3f p_world(Eigen::Vector3f p_local) = 0;
	Type type = Type::Default;
};

struct RigidBodyWrapper : public BodyWrapper {
	RigidBodyWrapper(std::shared_ptr<RigidBody> b) : rb(b) { type = Type::Rigid; }

	virtual float restitution_coeff() override { return rb->restitution_coeff; }

	virtual float friction_coeff() override { return rb->friction_coeff; }

	virtual Eigen::Vector3f p_world(Eigen::Vector3f p_local) override { return rb->rotation * p_local + rb->translation; }

	std::shared_ptr<RigidBody> rb;
};

struct ArticulatedBodyWrapper : public BodyWrapper {
	ArticulatedBodyWrapper(std::shared_ptr<ArticulatedBody> b, int id) : ab(b), id(id) { type = Type::Articulated; }

	virtual float restitution_coeff() override { return ab->bodies[id]->restitution_coeff; }

	virtual float friction_coeff() override { return ab->bodies[id]->friction_coeff; }

	virtual Eigen::Vector3f p_world(Eigen::Vector3f p_local) override { return ab->bodies[id]->rotation * p_local + ab->bodies[id]->translation; }

	std::shared_ptr<ArticulatedBody> ab = nullptr;
	int id = 0;
};

struct BodyContactPointVelocity {

	static std::shared_ptr<BodyContactPointVelocity> create(std::shared_ptr<BodyWrapper> b, const Eigen::Vector3f& pc);
	// velocity at contact point
	virtual MVector vc() = 0;
	virtual InvDyad inv_Ic() = 0;
	virtual void apply_impulse(const FVector& imp_c) = 0;
};

struct RigidBCPVelocity : public BodyContactPointVelocity {

	RigidBCPVelocity(std::shared_ptr<BodyWrapper> b, const Eigen::Vector3f& pc);

	virtual MVector vc() override;

	// inverse inertia at contact point
	virtual InvDyad inv_Ic() override { return _inv_Ic; }
	
	// impulse at contact point
	virtual void apply_impulse(const FVector& imp_c) override;

	std::shared_ptr<RigidBodyWrapper> bw = nullptr;
	MTransform Xortho_com_c; // transforms from com to contact point, no rotation
	MTransform Xortho_c_com; // inverse of the above two
	MTransform X_c_com;
	InvDyad _inv_Ic; // inverse inertia at contact point
};

struct ArticulatedBCPVelocity : public BodyContactPointVelocity {

	ArticulatedBCPVelocity(std::shared_ptr<BodyWrapper> b, const Eigen::Vector3f& pc);
	
	virtual MVector vc() override;

	// inverse inertia at contact point
	virtual InvDyad inv_Ic() override { return _inv_Ic; }

	// impulse at contact point
	virtual void apply_impulse(const FVector& imp_c) override;

	std::shared_ptr<ArticulatedBodyWrapper> bw = nullptr;
	MSubspace Jc; // jacobian at contact point
	JDyad inv_Ig; // generalized inverse inertia, H^-1 J^T -- projection from contact impulse to joint space velocity change
	InvDyad _inv_Ic; // inverse inertia at contact point
};

struct BodyContactPointPosition {

	static std::shared_ptr<BodyContactPointPosition> create(std::shared_ptr<BodyWrapper> b, const Eigen::Vector3f& pc);

	virtual InvDyad inv_Ic() = 0;

	virtual void apply_positional_impulse(const FVector& imp_c) = 0;
};

struct RigidBCPPosition : public BodyContactPointPosition  {
	// pc in word space
	RigidBCPPosition(std::shared_ptr<BodyWrapper> b, const Eigen::Vector3f& pc);

	virtual InvDyad inv_Ic() override { return _inv_Ic; };

	virtual void apply_positional_impulse(const FVector& imp_c) override;

	std::shared_ptr<RigidBodyWrapper> bw;
	MTransform Xortho_c_com;
	InvDyad _inv_Ic;
};

struct ArticulatedBCPPosition : public BodyContactPointPosition {
	// pc in world space
	ArticulatedBCPPosition(std::shared_ptr<BodyWrapper> b, const Eigen::Vector3f& pc);

	virtual InvDyad inv_Ic() override { return _inv_Ic; };

	virtual void apply_positional_impulse(const FVector& imp_c) override;

	struct LoopClosureDisplacement {
		Eigen::AngleAxisf ang_axis;
		Eigen::Vector3f linear;
	};
	// from predecessor to successor
	static LoopClosureDisplacement loop_closure_displacement_ps(std::shared_ptr<ArticulatedBody> ab, int loop_joint_id);

	std::shared_ptr<ArticulatedBodyWrapper> bw = nullptr;
	JDyad inv_Ig; // generalized inverse inertia, H^-1 J^T -- projection from contact impulse to joint space velocity change
	InvDyad _inv_Ic; // inverse inertia at contact point
};

}