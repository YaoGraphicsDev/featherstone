#pragma once

#include "spvec.hpp"
#include "glm/gtc/quaternion.hpp"

namespace SPD {

struct Shape {
	Shape() {}
	virtual ~Shape() = default;

	virtual void compute_volumn() {}

	virtual void compute_com() {}

	virtual void compute_inertia() {}

	float vol = 0.0f;
	Eigen::Vector3f com = Eigen::Vector3f::Zero(); // center of mass
	Eigen::Matrix3f Ic = Eigen::Matrix3f::Identity(); // inertia tensor about center of mass
	Dyad sp_Ic = Dyad::Identity(); // spatial inertia tensor about center of mass
};

struct Cuboid : public Shape {
	Cuboid(Eigen::Vector3f half_dims) : half_dims(half_dims) {
		compute_volumn();
		compute_com();
		compute_inertia();
	}

	virtual void compute_volumn() override;

	virtual void compute_com() override;

	virtual void compute_inertia() override;

	Eigen::Vector3f half_dims;
};

}