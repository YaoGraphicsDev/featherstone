#pragma once

#include "spvec.hpp"

namespace SPD {

struct Shape {
	Shape() {}
	virtual ~Shape() = default;

	virtual void compute_volumn() {}

	virtual void compute_com() {}

	virtual void compute_inertia() {}

	float vol = 0.0f;
	Eigen::Vector3f com = Eigen::Vector3f::Zero(); // center of mass
	Eigen::Matrix3f Ic3 = Eigen::Matrix3f::Identity(); // inertia tensor about center of mass
	Dyad Ic6 = Dyad::Identity(); // spatial inertia tensor about center of mass
	
	enum class Type {
		Cuboid = 0,
		Cylinder,
		Sphere,
		Default
	};
	Type type = Type::Default;
};

struct Cuboid : public Shape {
	Cuboid(Eigen::Vector3f half_dims) : half_dims(half_dims) {
		compute_volumn();
		compute_com();
		compute_inertia();
		type = Type::Cuboid;
	}

	virtual void compute_volumn() override;

	virtual void compute_com() override;

	virtual void compute_inertia() override;

	Eigen::Vector3f half_dims;
};

}