#include "spshapes.hpp"

namespace SPD {

void Cuboid::compute_volumn() {
	vol = 8.0f * half_dims.x() * half_dims.y() * half_dims.z();
}

void Cuboid::compute_com() {
	com = Eigen::Vector3f::Zero();
}

void Cuboid::compute_inertia() {
	float hx2 = half_dims.x() * half_dims.x();
	float hy2 = half_dims.y() * half_dims.y();
	float hz2 = half_dims.z() * half_dims.z();
	Ic3 = Eigen::Matrix3f::Zero();
	Ic3(0, 0) = vol * (hy2 + hz2) / 3.0f;
	Ic3(1, 1) = vol * (hx2 + hz2) / 3.0f;
	Ic3(2, 2) = vol * (hx2 + hy2) / 3.0f;
	Ic6 = Mat66(
		Ic3, Eigen::Matrix3f::Zero(),
		Eigen::Matrix3f::Zero(), Eigen::Vector3f::Constant(vol).asDiagonal());
}

}