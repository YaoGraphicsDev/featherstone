#pragma once

#include "Eigen/Core"
#include "Eigen/Dense"

// spatial vector based dynamics
namespace SPD {

template<int Col>
using Matrix6 = Eigen::Matrix<float, 6, Col>;
typedef Matrix6<1> MVector;
typedef Matrix6<1> FVector;
typedef Matrix6<6> MTransform;
typedef Matrix6<6> FTransform;
typedef Matrix6<6> Dyad; // Dyadic that maps motion space vector to force space
typedef Eigen::Matrix<float, 6, Eigen::Dynamic, 0, 6, 6> MSubspace;
typedef Eigen::Matrix<float, 6, Eigen::Dynamic, 0, 6, 6> FSubspace;
typedef Eigen::Matrix<float, Eigen::Dynamic, 1, 0, 6, 1> MCoordinates;
typedef Eigen::Matrix<float, Eigen::Dynamic, 1, 0, 6, 1> FCoordinates;
typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> JDyad; // Joint space dyad. A typical instance being the joint space inertia matrix H
typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> GPower; // Generalized power. Dot product between force and motion. Power and JDyad are in the same space.
typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> Unitless; // Generalized power. Dot product between force and motion. Power and JDyad are in the same space.


inline Eigen::Matrix3f cross_mat(const Eigen::Vector3f& r) {
	Eigen::Matrix3f M;
	M << 0.0f, -r.z(), r.y(),
		r.z(), 0.0f, -r.x(),
		-r.y(), r.x(), 0.0f;
	return M;
}

inline Matrix6<6> Mat66(
	const Eigen::Matrix3f& m00, const Eigen::Matrix3f& m01,
	const Eigen::Matrix3f& m10, const Eigen::Matrix3f& m11)
{
	Matrix6<6> mat = Matrix6<6>::Zero(6, 6);

	mat.topLeftCorner(3, 3) = m00;
	mat.topRightCorner(3, 3) = m01;
	mat.bottomLeftCorner(3, 3) = m10;
	mat.bottomRightCorner(3, 3) = m11;

	return mat;
}

// motion space coordinate transform from space A to B
inline MTransform m_transform(const Eigen::Matrix3f& base_a, const Eigen::Matrix3f& base_b, const Eigen::Vector3f& ab) {
	Eigen::Matrix3f e = base_b.transpose() * base_a;
	Eigen::Matrix3f rc = cross_mat(ab);
	return Mat66(
		e, Eigen::Matrix3f::Zero(),
		-e * rc, e);
}

inline MTransform inverse_transform(const MTransform& m) {
	assert(m.rows() == 6 && m.cols() == 6);
	Eigen::Matrix3f et = m.topLeftCorner(3, 3).transpose();
	Eigen::Matrix3f rc = -et * m.bottomLeftCorner(3, 3);
	return Mat66(
		et, Eigen::Matrix3f::Zero(),
		rc * et, et);
}

inline MTransform transpose_transform(const MTransform& m) {
	assert(m.rows() == 6 && m.cols() == 6);
	Eigen::Matrix3f et = m.topLeftCorner(3, 3).transpose();
	return Mat66(
		et, m.bottomLeftCorner(3, 3).transpose(),
		Eigen::Matrix3f::Zero(), et
	);
}

inline FTransform dual_transform(const MTransform& m) {
	assert(m.rows() == 6 && m.cols() == 6);
	FTransform f = m;
	f.topRightCorner(3, 3) = f.bottomLeftCorner(3, 3);
	f.bottomLeftCorner(3, 3) = Eigen::Matrix3f::Zero();
	return f;
}

inline MTransform derivative_cross(const MVector& v) {
	Eigen::Matrix3f w = cross_mat(v.head<3>());
	Eigen::Matrix3f vo = cross_mat(v.tail<3>());
	return Mat66(
		w, Eigen::Matrix3f::Zero(),
		vo, w);
}

inline Dyad transform_dyad(const MTransform& X_a_b, const Dyad& Ia) {
	return dual_transform(X_a_b) * Ia * inverse_transform(X_a_b);
}

inline Dyad transform_dyad2(const MTransform& X_b_a, const Dyad& Ia) {
	return transpose_transform(X_b_a) * Ia * X_b_a;
}

struct BlockAccess {
	typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> MType;

	BlockAccess(const std::vector<int>& block_sizes) : BlockAccess(block_sizes, block_sizes) {}

	BlockAccess(const std::vector<int>& row_block_sizes, const std::vector<int>& col_block_sizes) {
		// Calculate cumulative offsets for rows and columns
		std::vector<int> row_offsets = offsets(row_block_sizes);
		std::vector<int> col_offsets = offsets(col_block_sizes);

		int num_row_blocks = row_block_sizes.size();
		int num_col_blocks = col_block_sizes.size();
		blocks.resize(num_row_blocks, std::vector<BlockRange>(num_col_blocks));

		// Create block ranges for all combinations
		for (int i = 0; i < num_row_blocks; ++i) {
			for (int j = 0; j < num_col_blocks; ++j) {
				blocks[i][j] = BlockRange{
					row_offsets[i],
					col_offsets[j],
					row_block_sizes[i],
					col_block_sizes[j]
				};
			}
		}
	}

	Eigen::Block<MType> block(MType& M, int row, int col) {
		BlockRange r = blocks[row][col];
		assert(r.start_row + r.rows <= M.rows());
		assert(r.start_col + r.cols <= M.cols());
		return M.block(r.start_row, r.start_col, r.rows, r.cols);
	}

	int total_rows() {
		return blocks.back().back().start_row + blocks.back().back().rows;
	}

	int total_cols() {
		return blocks.back().back().start_col + blocks.back().back().cols;
	}

	std::vector<int> offsets(const std::vector<int>& sizes) {
		if (sizes.empty()) {
			return {};
		}

		std::vector<int> offsets(sizes.size(), 0);
		for (size_t i = 0; i < sizes.size() - 1; ++i) {
			offsets[i + 1] = offsets[i] + sizes[i];
		}
		return offsets;
	}


	struct BlockRange {
		int start_row;
		int start_col;
		int rows;
		int cols;
	};
	std::vector<std::vector<BlockRange>> blocks;
	
};

};