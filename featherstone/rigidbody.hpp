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
		float mass = 1.0f;
	};

	RigidBody(const Config& config) {
		shape = config.shape;
		rotation = config.rotation;
		translation = config.translation;
		// bases = rotation.toRotationMatrix();
		type = config.type;
		mass = config.mass;
		inv_mass = 1.0f / mass;
		v = MVector::Zero();
		// a = MVector::Zero();
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
	float mass = 1.0f;
	float inv_mass = 1.0f / mass;

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