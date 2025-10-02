#include "spdynamics.hpp"

#include <iostream>
#include <queue>

namespace SPD {

void ArticulatedBody::move_constraints(float dt) {
	for (int i = 1; i < joints.size(); ++i) {
		std::shared_ptr<Constraint> j = joints[i];

		// semi-implicit Euler
		j->dq += j->ddq * dt;
		j->q += j->dq * dt;

		// TODO: update X_J0_J1
		if (j->type == ConstraintType::Revolute) {
			j->X_J0_J1 = m_transform(
				Eigen::Matrix3f::Identity(),
				Eigen::AngleAxisf(j->q[0], Eigen::Vector3f::UnitZ()).toRotationMatrix(),
				Eigen::Vector3f::Zero());
		}
		else if (j->type == ConstraintType::Prismatic) {
			j->X_J0_J1 = m_transform(
				Eigen::Matrix3f::Identity(),
				Eigen::Matrix3f::Identity(),
				Eigen::Vector3f::UnitZ() * j->q[0]);
		}
		else {
			assert(false);
		}

		int Li = lambda[i];
		std::shared_ptr<const Constraint> Ji = joints[i];
		if (Li == 0) {
			X_Li_[i] = Ji->X_J0_J1 * Ji->X_0_J0;
		}
		else {
			std::shared_ptr<Constraint> JLi = joints[Li];
			X_Li_[i] = Ji->X_J0_J1 * Ji->X_0_J0 * inverse_transform(JLi->X_1_J1);
		}
		X_0_[i] = X_Li_[i] * X_0_[Li];
	}
}

void ArticulatedBody::solve_ddq() {
	FCoordinates tau;
	MCoordinates C;
	for (int i = 1; i < joints.size(); ++i) {
		std::shared_ptr<const Constraint> j = joints[i];
		tau.conservativeResize(tau.size() + j->taue.size());
		tau.tail(j->taue.size()) = j->taue;
		C.conservativeResize(C.size() + j->bias.size());
		C.tail(j->bias.size()) = j->bias;
	}
	assert(tau.rows() == H.rows() && C.rows() == H.rows());

	MCoordinates ddq = H.llt().solve(tau - C);

	int count = 0;
	for (int i = 1; i < joints.size(); ++i) {
		std::shared_ptr<Constraint> j = joints[i];
		j->ddq = ddq.segment(count, j->ddq.rows());
		count += j->ddq.rows();
	}
}

//void ArticulatedBody::apply_velo_friction() {
//	for (int i = 1; i < joints.size(); ++i) {
//		if (joints[i]->type == ConstraintType::Revolute) {
//			joints[i]->taue = joints[i]->dq * -0.1f;
//		}
//	}
//}

void ArticulatedBody::step(float dt) {
	move_constraints(dt);
	move_bodies();

	compute_bias_RNEA();
	compute_H();

	// apply_velo_friction();
	solve_ddq();

}

std::shared_ptr<ArticulatedBody::Body> ArticulatedBody::add_body(std::shared_ptr<Shape> shape, Eigen::Quaternionf rotation, Eigen::Vector3f translation) {
	std::shared_ptr<Body> body = std::make_shared<Body>();
	body->shape = shape;
	body->rotation = rotation;
	body->translation = translation;
	body->bases = rotation.toRotationMatrix();
	bodies.push_back(body);
	return body;
}

std::shared_ptr<ArticulatedBody::Constraint> ArticulatedBody::add_constraint(
	ConstraintType type,
	std::shared_ptr<Body> b0,
	std::shared_ptr<Body> b1,
	Eigen::Matrix3f base0,
	Eigen::Vector3f trans0) {

	assert(b0 && b1);
	std::shared_ptr<Constraint> c = std::make_shared<Constraint>();
	c->type = type;
	c->b0 = b0;
	c->b1 = b1;
	c->bb0 = base0;
	c->bt0 = trans0;
	Eigen::Matrix3f joint_bases_world = b0->bases * base0;
	c->bb1 = b1->bases.transpose() * joint_bases_world;
	Eigen::Vector3f joint_translation_world = b0->bases * trans0 + b0->translation;
	c->bt1 = b1->bases.transpose() * (joint_translation_world - b1->translation);
	c->X_0_J0 = m_transform(Eigen::Matrix3f::Identity(), c->bb0, c->bt0);
	c->X_1_J1 = m_transform(Eigen::Matrix3f::Identity(), c->bb1, c->bt1);
	c->X_J0_J1 = MTransform::Identity();
	if (type == ConstraintType::Revolute) {
		c->S.resize(6, 1);
		c->S << 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f;
		c->q.resize(1, 1);
		c->q << 0.0f;
		c->dq.resize(1, 1);
		c->dq << 0.0f;
		c->ddq.resize(1, 1);
		c->ddq << 0.0f;
		c->bias.resize(1, 1);
		c->bias << 0.0f;
		c->taue.resize(1, 1);
		c->taue << 0.0f;
	} else if(type == ConstraintType::Prismatic) {
		c->S.resize(6, 1);
		c->S << 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f;
		c->q.resize(1, 1);
		c->q << 0.0f;
		c->dq.resize(1, 1);
		c->dq << 0.0f;
		c->ddq.resize(1, 1);
		c->ddq << 0.0f;
		c->bias.resize(1, 1);
		c->bias << 0.0f;
		c->taue.resize(1, 1);
		c->taue << 0.0f;
	}
	joints.push_back(c);

	assert(!b1->parent_joint);
	b0->children_joints.push_back(c);
	b1->parent_joint = c;

	// Transform from C (com space) to J (joint space)
	MTransform X_C_J = m_transform(Eigen::Matrix3f::Identity(), c->bb1, c->bt1);
	b1->I = std::move(transform_dyad(X_C_J, b1->shape->sp_Ic));//  dual_transform(X_C_J) * b1->shape->sp_Ic* inverse_transform(X_C_J);

	return c;
}

void ArticulatedBody::set_constraint_status(
	std::shared_ptr<ArticulatedBody::Constraint> c,
	std::initializer_list<float> ql,
	std::initializer_list<float> dql,
	std::initializer_list<float> ddql) {
	assert(false); // TODO: set dq as next frame values
	if (c->type == ConstraintType::Ball) {
		// do not use Euler angles to calculate rotation for ball joints
		assert(false);
	}
	else if (c->type == ConstraintType::Revolute) {
		if (ql.size() >= 1) {
			c->q[0] = *ql.begin();
		}
		if (dql.size() >= 1) {
			c->dq[0] = *dql.begin();
		}
		if (ddql.size() >= 1) {
			c->ddq[0] = *ddql.begin();
		}
		c->X_J0_J1 = m_transform(
			Eigen::Matrix3f::Identity(),
			Eigen::AngleAxisf(c->q[0], Eigen::Vector3f::UnitZ()).toRotationMatrix(),
			Eigen::Vector3f::Zero());
	}
	else if (c->type == ConstraintType::Prismatic) {
		if (ql.size() >= 1) {
			c->q[0] = *ql.begin();
		}
		if (dql.size() >= 1) {
			c->dq[0] = *dql.begin();
		}
		if (ddql.size() >= 1) {
			c->ddq[0] = *ddql.begin();
		}
		c->X_J0_J1 = m_transform(
			Eigen::Matrix3f::Identity(),
			Eigen::Matrix3f::Identity(),
			Eigen::Vector3f::UnitZ() * c->q[0]);
	}
}

bool ArticulatedBody::build_tree() {
	// breadth first
	std::vector<std::shared_ptr<Body>> sorted_bodies;
	std::queue<std::shared_ptr<Body>> q;
	std::queue<int> lambda_q;
	q.push(bodies.front());
	while (!q.empty()) {
		std::shared_ptr<Body> b = q.front();
		sorted_bodies.push_back(b);
		q.pop();
		for (std::shared_ptr<Constraint> c : b->children_joints) {
			q.push(c->b1);
		}
	}
	
	assert(sorted_bodies.size() == bodies.size());
	bodies = std::move(sorted_bodies);
	for (int i = 0; i < bodies.size(); ++i) {
		bodies[i]->id = i;
	}
	
	std::vector<std::shared_ptr<Constraint>> sorted_joints(bodies.size(), nullptr);
	for (int i = 1; i < bodies.size(); ++i) {
		if (!bodies[i]->parent_joint) {
			// this body is not connected to the tree
			assert(false);
			return false;
		}
		sorted_joints[i] = bodies[i]->parent_joint;
	}
	joints = std::move(sorted_joints);

	// build flat tree structures
	// lambda -- parent body
	for (auto bi = bodies.begin(); bi < bodies.end(); ++bi) {
		if ((*bi)->id == 0) {
			lambda.push_back(-1);
			continue;
		}
		lambda.push_back((*bi)->parent_joint->b0->id);
	}

	// mu -- children bodies
	mu.resize(lambda.size());
	for (int i = bodies.size() - 1; i >= 1; --i) {
		mu[lambda[i]].insert(i);
	}

	// nu -- all subtree bodies
	nu.resize(mu.size());
	for (int i = bodies.size() - 1; i >= 1; --i) {
		nu[i].insert(i);
		nu[lambda[i]].insert(nu[i].begin(), nu[i].end());
	}

	X_0_.resize(bodies.size());
	X_0_[0] = MTransform::Identity();
	X_Li_.resize(bodies.size());
	X_Li_[0] = MTransform::Identity();

	// Figure out the dimensions and block configurations of H
	configure_H();

	return true;
}

void ArticulatedBody::configure_H() {
	int n_blocks = joints.size() - 1;
	H_blocks.resize(n_blocks);
	int H_size = 0;
	for (int i = 0; i < n_blocks; ++i) {
		H_blocks[i].resize(n_blocks);
		H_blocks[i][i].rows = joints[i + 1]->S.cols();
		H_blocks[i][i].cols = H_blocks[i][i].rows;
		H_size += joints[i + 1]->S.cols();
	}
	for (int i = 0; i < n_blocks; ++i) {
		for (int j = 0; j < n_blocks; ++j) {
			if (i == j) {
				continue;
			}
			H_blocks[i][j].rows = H_blocks[i][i].rows;
			H_blocks[i][j].cols = H_blocks[j][j].cols;
		}
	}
	for (int i = 0; i < n_blocks; ++i) {
		for (int j = 0; j < n_blocks; ++j) {
			if (i == 0) {
				H_blocks[i][j].start_row = 0;
			}
			else {
				H_blocks[i][j].start_row = H_blocks[i - 1][j].start_row + H_blocks[i - 1][j].rows;
			}
			if (j == 0) {
				H_blocks[i][j].start_col = 0;
			}
			else {
				H_blocks[i][j].start_col = H_blocks[i][j - 1].start_col + H_blocks[i][j - 1].cols;
			}
		}
	}
	H = JDyad::Zero(H_size, H_size);
}

Eigen::Ref<JDyad> ArticulatedBody::H_block(int row, int col) {
	HBlockRange r = H_blocks[row - 1][col - 1];
	return H.block(r.start_row, r.start_col, r.rows, r.cols);
}


void ArticulatedBody::move_bodies() {
	for (int i = 1; i < bodies.size(); ++i) {
		auto cb = bodies[i]; // child body
		auto pj = cb->parent_joint; // parent joint
		auto pb = pj->b0; // parent body

		Eigen::Matrix3f joint_bases0_world = pb->bases * pj->bb0;
		Eigen::Matrix3f joint_bases1_world = joint_bases0_world;
		if (pj->type == ConstraintType::Revolute) {
			joint_bases1_world = Eigen::AngleAxisf(pj->q(0, 0), joint_bases0_world.col(2)).toRotationMatrix() * joint_bases0_world;
		}
		else if (pj->type == ConstraintType::Prismatic) {
			joint_bases1_world = joint_bases0_world;
		}
		else {
			assert(false);
		}

		Eigen::Vector3f joint_translation0_world = pb->bases * pj->bt0 + pb->translation;
		Eigen::Vector3f joint_translation1_world = joint_translation0_world;
		if (pj->type == ConstraintType::Revolute) {
			joint_translation1_world = joint_translation0_world;
		}
		else if (pj->type == ConstraintType::Prismatic) {
			joint_translation1_world = joint_bases0_world.col(2) * pj->q(0, 0) + joint_translation0_world;
		}
		else {
			assert(false);
		}

		// body 1 bases in joint space 1
		Eigen::Matrix3f body1_bases_joint1 = pj->bb1.transpose();
		Eigen::Matrix3f body1_bases_world =  joint_bases1_world * body1_bases_joint1;
		cb->bases = body1_bases_world;
		cb->rotation = Eigen::Quaternionf(body1_bases_world);
		cb->rotation.normalize();

		Eigen::Vector3f body1_translation_joint1 = -body1_bases_joint1 * pj->bt1;
		Eigen::Vector3f body1_translation_world = joint_translation1_world + joint_bases1_world * body1_translation_joint1;
		cb->translation = body1_translation_world;
	}

	for (int i = 1; i < bodies.size(); ++i) {
		int Li = lambda[i];
		std::shared_ptr<Body> Bi = bodies[i];
		std::shared_ptr<const Body> BLi = bodies[Li];
		std::shared_ptr<const Constraint> Ji = joints[i];

		MVector& vi = Bi->v;
		MVector& ai = Bi->a;
		const MVector& vLi = BLi->v;
		const MVector& aLi = BLi->a;
		const MSubspace& Si = Ji->S;
		const MCoordinates& dqi = Ji->dq;
		const MCoordinates& ddqi = Ji->ddq;
		MTransform vci = std::move(derivative_cross(vi));

		vi = X_Li_[i] * vLi + Si * dqi;
		ai = X_Li_[i] * aLi + Si * ddqi + vci * Si * dqi;
	}

	return;
}

void ArticulatedBody::compute_bias_RNEA() {
	MVector v0 = MVector::Zero();
	MVector aw;
	aw << Eigen::Vector3f::Zero(), -gravity;
	MTransform X_W_0 = m_transform(Eigen::Matrix3f::Identity(), bodies[0]->bases, bodies[0]->translation);// from world space to body 0 space

	std::vector<MVector> a(bodies.size());
	a[0] = X_W_0 * aw;

	std::vector<FVector> f(bodies.size());
	f[0] = FVector::Zero();
	for (int i = 1; i < bodies.size(); ++i) {
		int Li = lambda[i];
		std::shared_ptr<const Body> Bi = bodies[i];
		std::shared_ptr<const Body> BLi = bodies[Li];
		std::shared_ptr<const Constraint> Ji = joints[i];

		const MVector& vi = Bi->v;
		const MVector& vLi = BLi->v;
		const MSubspace& Si = Ji->S;
		const MCoordinates& dqi = Ji->dq;
		// MCoordinates& ddqi = Ji->ddq;
		MTransform vci = std::move(derivative_cross(vi));

		a[i] = X_Li_[i] * a[Li] /* + Si * ddqi */ + vci * Si * dqi;

		const Dyad& Ii = Bi->I;
		f[i] = Ii * a[i] + dual_transform(vci) * Ii * vi - dual_transform(X_0_[i]) * Bi->fe0;
	}
	for (int i = bodies.size() - 1; i >= 1; --i) {
		joints[i]->bias = joints[i]->S.transpose() * f[i];
		int Li = lambda[i];
		if (Li != 0) {
			f[Li] += transpose_transform(X_Li_[i]) * f[i];
		}
	}
}

void ArticulatedBody::compute_H() {
	std::vector<Dyad> Ic(bodies.size(), Dyad::Zero());
	for (int i = 1; i < bodies.size(); ++i) {
		Ic[i] = bodies[i]->I;
	}
	for (int i = bodies.size() - 1; i >= 1; --i) {
		int Li = lambda[i];
		if (Li != 0) {
			Ic[Li] += transform_dyad2(X_Li_[i], Ic[i]);
		}

		MSubspace& Si = joints[i]->S;
		FVector F = Ic[i] * Si;
		H_block(i, i) = Si.transpose() * F;
		// TODO: why is the first element of H zero?
		int j = i;
		while (lambda[j] != 0) {
			F = X_Li_[j].transpose() * F;
			j = lambda[j];
			H_block(i, j) = F.transpose() * joints[j]->S;
			H_block(j, i) = H_block(i, j).transpose();
		}
	} 
}

}