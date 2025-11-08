#include "Eigen/Core"
#include "Eigen/Dense"
#include "spshapes.hpp"

#include <iostream>

using namespace Eigen;
using namespace SPD;

void generate_box(Vector3f half_dims, std::vector<float>& vb, std::vector<uint16_t>& ib)
{
	vb.clear();
	ib.clear();

	const float hw = half_dims.x();
	const float hh = half_dims.y();
	const float hl = half_dims.z();

	// 8 corners of the box
	const float vertices[8][3] = {
		{-hw, -hh, -hl}, // 0
		{ hw, -hh, -hl}, // 1
		{ hw,  hh, -hl}, // 2
		{-hw,  hh, -hl}, // 3
		{-hw, -hh,  hl}, // 4
		{ hw, -hh,  hl}, // 5
		{ hw,  hh,  hl}, // 6
		{-hw,  hh,  hl}  // 7
	};

	// Copy to vertex buffer (positions only)
	for (int i = 0; i < 8; ++i) {
		vb.push_back(vertices[i][0]);
		vb.push_back(vertices[i][1]);
		vb.push_back(vertices[i][2]);
	}

	// 12 triangles (36 indices)
	const uint16_t indices[36] = {
		// Front face
		4, 5, 6,  4, 6, 7,
		// Back face
		0, 2, 1,  0, 3, 2,
		// Left face
		0, 7, 3,  0, 4, 7,
		// Right face
		1, 2, 6,  1, 6, 5,
		// Top face
		3, 7, 6,  3, 6, 2,
		// Bottom face
		0, 1, 5,  0, 5, 4
	};

	ib.insert(ib.end(), std::begin(indices), std::end(indices));
}

void move_vertices(Quaternionf rot, Vector3f trans, std::vector<float>& vb) {
	int n_vertices = vb.size() / 3;
	for (int i = 0; i < n_vertices; ++i) {
		Vector3f v;
		float& x = vb[3 * i];
		float& y = vb[3 * i + 1];
		float& z = vb[3 * i + 2];
		v << x, y, z;
		v = rot * v + trans;
		x = v.x();
		y = v.y();
		z = v.z();
	}
}

Vector3f box_half_dim(1.4f, 1.0f, 0.8f);

int main() {
	std::shared_ptr<Cuboid> cube = std::make_shared<Cuboid>(box_half_dim);
	
	std::vector<float> vb;
	std::vector<uint16_t> ib;
	generate_box(box_half_dim, vb, ib);
	std::shared_ptr<ConvexHull> hull = std::make_shared<ConvexHull>(vb.data(), vb.size() / 3, ib.data(), ib.size());

	std::cout << "=================== Test Convex Hull ==================" << std::endl;
	std::cout << "Cuboid: " << std::endl;
	std::cout << "\tvolume = " << cube->vol << std::endl;
	std::cout << "\tCoM = " << cube->com.transpose() << std::endl;
	std::cout << "\tIc3 = " << std::endl;
	std::cout << cube->Ic3 << std::endl;
	std::cout << "\tIc6 = " << std::endl;
	std::cout << cube->Ic6 << std::endl;

	std::cout << "\nConvexHull: " << std::endl;
	std::cout << "\tvolume = " << hull->vol << std::endl;
	std::cout << "\tCoM = " << hull->com.transpose() << std::endl;
	std::cout << "\tIc3 = " << std::endl;
	std::cout << hull->Ic3 << std::endl;
	std::cout << "\tIc6 = " << std::endl;
	std::cout << hull->Ic6 << std::endl;

	Quaternionf rotation = Quaternionf(AngleAxisf(30.0f / 180.0f * 3.14159265f, Vector3f(1.0f, 1.0f, 0.5f).normalized()));
	Vector3f translation = Vector3f(0.7f, 3.2f, 0.1f);
	// Vector3f translation = Vector3f::Zero();
	Matrix3f base = rotation.toRotationMatrix();
	Dyad I6_cube = transform_dyad2(m_transform(Matrix3f::Identity(), base, translation), cube->Ic6);
	move_vertices(rotation, translation, vb);
	hull.reset(new ConvexHull(vb.data(), vb.size() / 3, ib.data(), ib.size()));
	Dyad I6_hull = transform_dyad(m_transform(Matrix3f::Identity(), Matrix3f::Identity(), -translation), hull->Ic6);

	std::cout << "Apply rotation and translation" << std::endl;

	std::cout << "\nCuboid I6 = " << std::endl;
	std::cout << I6_cube << std::endl;

	std::cout << "\nConvexHull I6 = " << std::endl;
	std::cout << I6_hull << std::endl;

	std::cout << "\nConvecHull CoM = " << hull->com.transpose() << std::endl;
	std::cout << "\nConvexHull volume = " << hull->vol << std::endl;

	
	std::cout << "=================== Test Compound Shape ==================" << std::endl;
	generate_box(box_half_dim, vb, ib);
	hull.reset(new ConvexHull(vb.data(), vb.size() / 3, ib.data(), ib.size()));
	std::vector<CompoundShape::Composition> comps = {
		{hull, translation + rotation * Vector3f(box_half_dim.x(), box_half_dim.y(), box_half_dim.z()), rotation },
		{hull, translation + rotation * Vector3f(box_half_dim.x(), box_half_dim.y(), -box_half_dim.z()), rotation },
		{hull, translation + rotation * Vector3f(box_half_dim.x(), -box_half_dim.y(), box_half_dim.z()), rotation },
		{hull, translation + rotation * Vector3f(box_half_dim.x(), -box_half_dim.y(), -box_half_dim.z()), rotation },
		{hull, translation + rotation * Vector3f(-box_half_dim.x(), box_half_dim.y(), box_half_dim.z()), rotation },
		{hull, translation + rotation * Vector3f(-box_half_dim.x(), box_half_dim.y(), -box_half_dim.z()), rotation },
		{hull, translation + rotation * Vector3f(-box_half_dim.x(), -box_half_dim.y(), box_half_dim.z()), rotation },
		{hull, translation + rotation * Vector3f(-box_half_dim.x(), -box_half_dim.y(), -box_half_dim.z()), rotation },
	};
	std::shared_ptr<CompoundShape> cs = std::make_shared<CompoundShape>(comps);
	std::cout << "Compound Ic6 = " << std::endl;
	std::cout << cs->Ic6 << std::endl;
	//std::cout << "Compound Ic3 = " << std::endl;
	//std::cout << cs->Ic3 << std::endl;

	cube.reset(new Cuboid(box_half_dim * 2.0f));
	std::cout << "\nBox Ic6 = " << std::endl;
	std::cout << transform_dyad2(m_transform(Matrix3f::Identity(), base, Vector3f::Zero()), cube->Ic6) << std::endl;
	//std::cout << "Box Ic3 = " << std::endl;
	//std::cout << cube->Ic3 << std::endl;

	return 0;
}