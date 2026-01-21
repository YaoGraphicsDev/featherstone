#pragma once

#include "spshapes.hpp"
#include "btBulletCollisionCommon.h"

namespace SPD {

struct RigidBody {
	enum class DynamicType {
		Dynamic = 0,
		Static,
		// Kinematic TODO: kinematic
	};

	struct Config {
		std::string name = "";
		std::shared_ptr<Shape> shape = nullptr;
		Eigen::Quaternionf rotation = Eigen::Quaternionf::Identity();
		Eigen::Vector3f translation = Eigen::Vector3f::Zero();
		DynamicType type = DynamicType::Dynamic;
		float density = 1.0f;
		float restitution_coeff = 0.3f;
		float friction_coeff = 0.5f;
	};

	RigidBody(const Config& config) {
		name = config.name;
		shape = config.shape;
		rotation = config.rotation;
		translation = config.translation;
		// bases = rotation.toRotationMatrix();
		type = config.type;
		if (type == DynamicType::Dynamic) {
			assert(config.density > 0.0f);
		}
		Dyad Ic = shape->Ic6 * config.density;
		MTransform X_com_o = m_transform(Eigen::Matrix3f::Identity(), Eigen::Matrix3f::Identity(), -shape->com);
		I = transform_dyad(X_com_o, Ic);
		Dyad inv_Ic = Mat66(
			shape->Ic3.inverse(), Eigen::Matrix3f::Zero(),
			Eigen::Matrix3f::Zero(), Eigen::Vector3f::Constant(1.0f / shape->vol).asDiagonal()) / config.density;
		//Dyad inv_Ic = Ic.inverse();
		inv_I = transform_inv_dyad(X_com_o, inv_Ic);
		mass = shape->vol * config.density;
		v = MVector::Zero();
		// fe = FVector::Zero(); // external force
		linear_damping = 0.005f;
		angular_damping = 0.005f;
		restitution_coeff = config.restitution_coeff;
		friction_coeff = config.friction_coeff;
	}

	std::string name = "";
	std::shared_ptr<Shape> shape = nullptr;
	Eigen::Quaternionf rotation = Eigen::Quaternionf::Identity();
	Eigen::Vector3f translation = Eigen::Vector3f::Zero();
	// Eigen::Matrix3f bases = rotation.toRotationMatrix();
	DynamicType type = DynamicType::Static;
	Dyad I; // inertial at origin
	Dyad inv_I; // inertial at origin
	float mass;

	MVector v = MVector::Zero(); // velocity at origin, orthogonal
	// FVector fe = FVector::Zero(); // external force at origin, orthogonal
	float linear_damping = 0.005f;
	float angular_damping = 0.005f;
	float restitution_coeff = 0.3f;
	float friction_coeff = 0.5f;
};

//struct Collider {
//	static std::shared_ptr<Collider> create(const RigidBody& rigidbody);
//
//	void update(Eigen::Vector3f translation, Eigen::Quaternionf rotation);
//
//	std::shared_ptr<btCollisionShape> shape;
//	std::shared_ptr<btCollisionObject> obj;
//};

}