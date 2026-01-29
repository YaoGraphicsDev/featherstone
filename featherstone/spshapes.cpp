#include "spshapes.hpp"
#include <cmath>
#include <iostream>

using namespace Eigen;

namespace SPD {

Cuboid::Cuboid(Eigen::Vector3f half_dims) : Shape() {
	this->half_dims = half_dims;
	this->type = Type::Cuboid;
	this->vol = 8.0f * half_dims.x() * half_dims.y() * half_dims.z();
	this->com = Vector3f::Zero();

	float hx2 = half_dims.x() * half_dims.x();
	float hy2 = half_dims.y() * half_dims.y();
	float hz2 = half_dims.z() * half_dims.z();
	this->Ic3 = Matrix3f::Zero();
	this->Ic3(0, 0) = vol * (hy2 + hz2) / 3.0f;
	this->Ic3(1, 1) = vol * (hx2 + hz2) / 3.0f;
	this->Ic3(2, 2) = vol * (hx2 + hy2) / 3.0f;
	this->Ic6 = Mat66(
		Ic3, Matrix3f::Zero(),
		Matrix3f::Zero(), Vector3f::Constant(vol).asDiagonal());
}

const float Pi = 3.14159265358979323846f;

Sphere::Sphere(float radius) : Shape() {
	this->radius = radius;
	this->type = Type::Sphere;
	this->vol = 4.0f / 3.0f * Pi * radius * radius * radius;
	this->com = Vector3f::Zero();
	this->Ic3 = Vector3f::Constant(0.4f * vol * radius * radius).asDiagonal();
	this->Ic6 = Mat66(
		Ic3, Matrix3f::Zero(),
		Matrix3f::Zero(), Vector3f::Constant(vol).asDiagonal());
}

Cylinder::Cylinder(float r, float h) {
	this->half_dims = Vector2f(r, h * 0.5f);
	float r2 = r * r;
	float h2 = h * h;
	this->type = Type::Cylinder;
	this->vol = Pi * r2 * h;
	this->com = Vector3f::Zero();

	this->Ic3 = Matrix3f::Zero();
	this->Ic3(0, 0) = this->vol * (3.0f * r2 + h2) / 12.0f;
	this->Ic3(2, 2) = this->vol * (3.0f * r2 + h2) / 12.0f;
	this->Ic3(1, 1) = 0.5f * this->vol * r2;
	this->Ic6 = Mat66(
		Ic3, Matrix3f::Zero(),
		Matrix3f::Zero(), Vector3f::Constant(vol).asDiagonal());
}

// signed volume of tetrahedron, with one vertex at origin
inline static float V_tetra(const Vector3f& p1, const Vector3f& p2, const Vector3f& p3) {
	Matrix3f m;
	m.row(0) = p1;
	m.row(1) = p2;
	m.row(2) = p3;

	float v = m.determinant() / 6.0f;
	return v;
}

// inertia tensor of tetrahedron, with one vertex at origin. Resulting inertia tensor relative to origin
static Matrix3f I3_tetra(const Vector3f& p1, const Vector3f& p2, const Vector3f& p3) {
	Matrix3f I = Matrix3f::Zero();
	Matrix3f product;
	std::array<Vector3f, 4> p = { Vector3f::Zero(), p1, p2, p3 };
	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			product = p[i].dot(p[j]) * Matrix3f::Identity() - p[i] * p[j].transpose();
			if (i != j) {
				product *= 0.5f;
			}
			I += product;
		}
	}

	Matrix3f m;
	m.row(0) = p1;
	m.row(1) = p2;
	m.row(2) = p3;

	I = I * V_tetra(p1, p2, p3) / 10.0f;
	return I;
}


ConvexHull::ConvexHull(const float* vertices, uint32_t n_vertices, const uint16_t* indices, uint32_t n_indices) : Shape() {
	positions.resize(n_vertices);
	for (uint32_t i = 0; i < this->positions.size(); ++i) {
		const float* ptr = vertices + i * 3;
		positions[i] << *ptr, *(ptr + 1), *(ptr + 2);
	}

	// compute volume and CoM
	uint32_t n_triangles = n_indices / 3;
	float total_volume = 0.0f;
	Matrix3f total_inertia_tensor = Matrix3f::Zero();
	Vector3f volume_weighted_com = Vector3f::Zero();

	for (uint32_t t = 0; t < n_triangles; ++t) {
		Matrix3f m;
		uint32_t i = t * 3;
		Vector3f& v0 = positions[indices[i]];
		Vector3f& v1 = positions[indices[i + 1]];
		Vector3f& v2 = positions[indices[i + 2]];
		float tetra_volume = V_tetra(v0, v1, v2);
		total_volume += tetra_volume;
		Vector3f tetra_com = (v0 + v1 + v2) / 4.0f;
		volume_weighted_com += tetra_volume * tetra_com;
	}

	if (total_volume <= 0) {
		std::cout << "compound shape volume ( " << total_volume << ") should not be negative.Probably caused by inverted scaling of resources." << std::endl;
		assert(false);
		total_volume = -total_volume;
	}
	this->vol = total_volume;
	this->com = volume_weighted_com / total_volume;
	
	this->Ic3 = Matrix3f::Zero();
	for (uint32_t t = 0; t < n_triangles; ++t) {
		Matrix3f m;
		uint32_t i = t * 3;
		Vector3f& v0 = positions[indices[i]];
		Vector3f& v1 = positions[indices[i + 1]];
		Vector3f& v2 = positions[indices[i + 2]];
		// The inertia tensor of each tetrahedron relative to convex hull CoM
		Matrix3f tetra_Ic3 = I3_tetra(v0 - this->com, v1 - this->com, v2 - this->com); 
		this->Ic3 += tetra_Ic3;
	}
	this->Ic6 = Mat66(
		this->Ic3, Matrix3f::Zero(),
		Matrix3f::Zero(), Vector3f::Constant(vol).asDiagonal());

	this->type = Type::ConvexHull;
}

CompoundShape::CompoundShape(const std::vector<Composition>& compositions) : Shape() {

	this->compositions = compositions;
	this->type = Type::Compound;

	float total_volume = 0.0f;
	// Matrix3f total_inertia_tensor = Matrix3f::Zero();
	Vector3f volume_weighted_com = Vector3f::Zero();
	for (const Composition& comp : compositions) {
		total_volume += comp.shape->vol;
		volume_weighted_com += comp.shape->vol * (comp.shape->com + comp.translation);
	}

	if (total_volume <= 0) {
		std::cout << "compound shape volume ( " << total_volume << ") should not be negative.Probably caused by inverted scaling of resources." << std::endl;
		assert(false);
		total_volume = -total_volume;
	}
	this->vol = total_volume;
	this->com = volume_weighted_com / total_volume;
	
	Dyad total_Ic6 = Dyad::Zero();
	for (const Composition& comp : compositions) {
		// auto shape = comp.shape;
		MTransform X_com_compcom = m_transform(Matrix3f::Identity(), comp.rotation.toRotationMatrix(), comp.shape->com + comp.translation - this->com);
		total_Ic6 += transform_dyad2(X_com_compcom, comp.shape->Ic6);
	}
	this->Ic6 = total_Ic6;

	// conceptually, Ic6 should take the diagonal form
	// the following assertions essentially say that each off-diagonal insignificant element, having a non-zero value due to numerical error,
	// should be at least 5 magnitudes smaller than volume
	//assert(std::abs(Ic6.topRightCorner<3, 3>().determinant() / vol) < 3e-15);
	//assert(std::abs(Ic6.bottomLeftCorner<3, 3>().determinant() / vol) < 3e-15);
	//assert(std::abs((Ic6.bottomRightCorner<3, 3>() - Matrix3f(Vector3f::Constant(vol).asDiagonal())).determinant() / vol) < 3e-15);
	
	this->Ic3 = Ic6.topLeftCorner<3, 3>();
}

}