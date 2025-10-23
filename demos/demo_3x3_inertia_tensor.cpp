#include "Eigen/Core"
#include "Eigen/Dense"

#include <iostream>

using namespace Eigen;

const Vector3f half_dims(1.0f, 0.5f, 0.7f);
const float angle = 45.0f / 180.0f * 3.1416f;
const Vector3f axis(1.0f, 1.0f, 1.0f);
const Vector3f t_com(0.295312, 1.08231, -0.620993);

inline Eigen::Matrix3f cross_mat(const Eigen::Vector3f& r) {
	Eigen::Matrix3f M;
	M << 0.0f, -r.z(), r.y(),
		r.z(), 0.0f, -r.x(),
		-r.y(), r.x(), 0.0f;
	return M;
}

int main() {
	float vol = 8.0f * half_dims.x() * half_dims.y() * half_dims.z();
	float hx2 = half_dims.x() * half_dims.x();
	float hy2 = half_dims.y() * half_dims.y();
	float hz2 = half_dims.z() * half_dims.z();
	Matrix3f Ic = Eigen::Matrix3f::Zero();
	Ic(0, 0) = vol * (hy2 + hz2) / 3.0f;
	Ic(1, 1) = vol * (hx2 + hz2) / 3.0f;
	Ic(2, 2) = vol * (hx2 + hy2) / 3.0f;
	std::cout << "Inertia tensor about CoM = " << std::endl;
	std::cout << Ic << std::endl;

	Quaternionf q(AngleAxisf(angle, axis.normalized()));
	Matrix3f E = q.toRotationMatrix();
	std::cout << "E = " << std::endl;
	std::cout << E << std::endl;

	Matrix3f bias0 = vol * (t_com.squaredNorm() * Matrix3f::Identity() - t_com * t_com.transpose());
	std::cout << "bias0 = " << std::endl;;
	std::cout << bias0 << std::endl;
	Matrix3f bias1 = vol * E * cross_mat(t_com) * cross_mat(t_com) * E.transpose();
	std::cout << "bias1 = " << std::endl;
	std::cout << bias1 << std::endl;
	Matrix3f EIET = E * Ic * E.transpose();
	std::cout << "EIET = " << std::endl;
	std::cout << EIET << std::endl;
	std::cout << "EIET inverse = " << std::endl;
	std::cout << EIET.inverse() << std::endl;

	Matrix3f I = E * Ic * E.transpose() + vol * (t_com.squaredNorm() * Matrix3f::Identity() - t_com * t_com.transpose());
	std::cout << "I world = " << std::endl;
	std::cout << I << std::endl;

	return 0;
}