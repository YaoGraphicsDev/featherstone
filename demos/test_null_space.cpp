#include "spvec.hpp"

#include <iostream>

using namespace Eigen;
using namespace SPD;

int main() {
	Unitless C(6, 2);
	C <<
		1, 0,
		0, 1,
		0, 0,
		0.45, 0,
		0, 3,
		0, 0;
	Unitless N = null_space_QR(C.transpose());
	std::cout << "C^T null space basis = " << std::endl;
	std::cout << N << std::endl;
	std::cout << "C^T N = " << std::endl;
	std::cout << C.transpose() * N << std::endl;

	N = ortho_complement_QR(C);
	std::cout << "C orthogonal complementary = " << std::endl;
	std::cout << N << std::endl;
	std::cout << "C^T N = " << std::endl;
	std::cout << C.transpose() * N << std::endl;

	std::cout << "N^T N = " << std::endl;
	std::cout << N.transpose() * N << std::endl;

	return 0;
}