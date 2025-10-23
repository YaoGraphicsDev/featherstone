#pragma once

#include "spshapes.hpp"
#include "btBulletCollisionCommon.h"

namespace SPD {

struct RigidBody {
	enum class DynamicType {
		Dynamic,
		Static,
		// Kinematic TODO: kinematic
	};

	struct Config {
		std::shared_ptr<Shape> shape = nullptr;
		Eigen::Quaternionf rotation = Eigen::Quaternionf::Identity();
		Eigen::Vector3f translation = Eigen::Vector3f::Zero();
		DynamicType type = DynamicType::Dynamic;
		float density = 1.0f;
	};

	RigidBody(const Config& config) {
		shape = config.shape;
		rotation = config.rotation;
		translation = config.translation;
		// bases = rotation.toRotationMatrix();
		type = config.type;
		assert(config.density > 0.0f);
		Ic = shape->Ic6 * config.density;
		inv_Ic = Mat66(
			shape->Ic3.inverse(), Eigen::Matrix3f::Zero(),
			Eigen::Matrix3f::Zero(), Eigen::Vector3f::Constant(1.0f / shape->vol).asDiagonal()) / config.density;
		mass = shape->vol * config.density;
		v = MVector::Zero();
		fe = FVector::Zero(); // external force
		linear_damping = 0.05f;
		angular_damping = 0.05f;
		restitution_coeff = 0.5f;
		friction_coeff = 0.5f;
	}

	std::shared_ptr<Shape> shape = nullptr;
	Eigen::Quaternionf rotation = Eigen::Quaternionf::Identity();
	Eigen::Vector3f translation = Eigen::Vector3f::Zero();
	// Eigen::Matrix3f bases = rotation.toRotationMatrix();
	DynamicType type = DynamicType::Static;
	Dyad Ic;
	Dyad inv_Ic;
	float mass;
	// float density = 1.0f;
	// float inv_density = 1.0f / density;

	MVector v = MVector::Zero();
	FVector fe = FVector::Zero(); // external force
	float linear_damping = 0.05f;
	float angular_damping = 0.05f;
	float restitution_coeff = 0.5f;
	float friction_coeff = 0.5f;
};

struct Collider {
	static std::shared_ptr<Collider> create(const RigidBody& rigidbody);

	void update(Eigen::Vector3f translation, Eigen::Quaternionf rotation);

	std::shared_ptr<btCollisionShape> shape;
	std::shared_ptr<btCollisionObject> obj;
};

}