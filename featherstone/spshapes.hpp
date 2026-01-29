#pragma once

#include "spvec.hpp"

namespace SPD {

struct Shape {
	Shape() {}
	virtual ~Shape() = default;

	float vol = 0.0f; // volume
	Eigen::Vector3f com = Eigen::Vector3f::Zero(); // center of mass
	Eigen::Matrix3f Ic3 = Eigen::Matrix3f::Zero(); // inertia tensor about center of mass
	Dyad Ic6 = Dyad::Zero(); // spatial inertia tensor about center of mass
	
	enum class Type {
		Cuboid = 0,
		Cylinder,
		Sphere,
		ConvexHull,
		Compound,
		Default
	};
	Type type = Type::Default;
};

struct Cuboid : public Shape {
	Cuboid(Eigen::Vector3f half_dims);

	Eigen::Vector3f half_dims;
};

struct Sphere : public Shape {
	Sphere(float radius);
	
	float radius;
};

struct Cylinder : public Shape {
	Cylinder(float r, float h);

	Eigen::Vector2f half_dims;
};

struct ConvexHull : public Shape {
	ConvexHull(const float* vertices, uint32_t n_vertices, const uint16_t* indices, uint32_t n_indices);

	std::vector<Eigen::Vector3f> positions;
};

struct CompoundShape : public Shape {
	struct Composition {
		std::shared_ptr<Shape> shape;
		Eigen::Vector3f translation;
		Eigen::Quaternionf rotation;
	};
	CompoundShape(const std::vector<Composition>& compositions);

	std::vector<Composition> compositions;
};

}