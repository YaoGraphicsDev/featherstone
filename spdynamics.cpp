#include "spdynamics.hpp"

#include <iostream>
#include <queue>

namespace SPD {

void ArticulatedBody::move_constraints(float dt) {
	for (int i = 1; i < tree_joints.size(); ++i) {
		std::shared_ptr<Constraint> j = tree_joints[i];

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
		std::shared_ptr<const Constraint> Ji = tree_joints[i];
		if (Li == 0) {
			X_Li_[i] = Ji->X_J0_J1 * Ji->X_0_J0;
		}
		else {
			std::shared_ptr<Constraint> JLi = tree_joints[Li];
			X_Li_[i] = Ji->X_J0_J1 * Ji->X_0_J0 * inverse_transform(JLi->X_1_J1);
		}
		X_0_[i] = X_Li_[i] * X_0_[Li];
	}
}

void ArticulatedBody::joint_damping() {
	for (int i = 1; i < tree_joints.size(); ++i) {
		tree_joints[i]->taue -= tree_joints[i]->dq * 0.8f;
	}
	// TODO: add effective damping to loop joints
}

void ArticulatedBody::clear_joint_forces() {
	for (int i = 1; i < tree_joints.size(); ++i) {
		tree_joints[i]->taue.setZero();
	}
}

void ArticulatedBody::solve_ddq() {
	// build tau and C
	FCoordinates tau;
	MCoordinates C;
	for (int i = 1; i < tree_joints.size(); ++i) {
		std::shared_ptr<const Constraint> j = tree_joints[i];
		tau.conservativeResize(tau.size() + j->taue.size());
		tau.tail(j->taue.size()) = j->taue;
		C.conservativeResize(C.size() + j->bias.size());
		C.tail(j->bias.size()) = j->bias;
	}
	//std::cout << "tau = " << std::endl;
	//std::cout << tau << std::endl;
	assert(tau.rows() == H.rows() && C.rows() == H.rows());

	// compute loop joint force variale lambda
	Unitless lambda = Unitless::Zero(K.rows(), 1);
	if (!loop_joints.empty()) {
		Unitless KHi = K * H.inverse();
		Unitless A = KHi * K.transpose();
		Unitless inv_A = A.completeOrthogonalDecomposition().pseudoInverse();
		Unitless b = _k - KHi * (tau - C);
		lambda = inv_A * b;
	}


	MCoordinates ddq = MCoordinates::Zero(H.cols(), 1);
	if (loop_joints.empty()) {
		ddq = H.llt().solve(tau - C);
	}
	else {
		ddq = H.llt().solve(tau - C + K.transpose() * lambda);
	}

	int count = 0;
	for (int i = 1; i < tree_joints.size(); ++i) {
		std::shared_ptr<Constraint> j = tree_joints[i];
		j->ddq = ddq.segment(count, j->ddq.rows());
		count += j->ddq.rows();
	}
}

void ArticulatedBody::step(float dt) {
	move_constraints(dt);
	move_bodies();

	compute_bias_RNEA();
	compute_H();
	compute_K_k();

	joint_damping();
	solve_ddq();
	clear_joint_forces();
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
	c->S = &S.at(type);
	c->T = &T.at(type);
	c->Ta = &Ta.at(type);
	if (type == ConstraintType::Revolute) {
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
	tree_joints.push_back(c);

	b0->children_joints.push_back(c);
	b1->parent_joints.push_back(c);

	// Transform from C (com space) to J (joint space)
	MTransform X_C_J = m_transform(Eigen::Matrix3f::Identity(), c->bb1, c->bt1);
	if (b1->shape) {
		b1->I = std::move(transform_dyad(X_C_J, b1->shape->sp_Ic));//  dual_transform(X_C_J) * b1->shape->sp_Ic* inverse_transform(X_C_J);
	}

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
	q.push(bodies.front());
	std::set<Body*> visited;
	visited.insert(bodies.front().get());
	while (!q.empty()) {
		std::shared_ptr<Body> b = q.front();
		sorted_bodies.push_back(b);
		q.pop();
		for (std::shared_ptr<Constraint> c : b->children_joints) {
			if (visited.find(c->b1.get()) == visited.end()) {
				// body not visited yet, no loop
				q.push(c->b1);
				visited.insert(c->b1.get());
			}
			else {
				// loop
				// 1. store loop joint 2. remove parent joint
				loop_joints.push_back(c);
				c->b1->parent_joints.remove_if([&](std::shared_ptr<Constraint> j) {
					return j.get() == c.get();
				});
			}
		}
	}

	assert(sorted_bodies.size() == bodies.size());
	bodies = std::move(sorted_bodies);
	for (int i = 0; i < bodies.size(); ++i) {
		bodies[i]->id = i;
	}
	
	std::vector<std::shared_ptr<Constraint>> sorted_joints(bodies.size(), nullptr);
	for (int i = 1; i < bodies.size(); ++i) {
		if (bodies[i]->parent_joints.empty()) {
			// this body is not connected to the tree
			assert(false);
			return false;
		}
		sorted_joints[i] = *bodies[i]->parent_joints.begin();
	}
	tree_joints = std::move(sorted_joints);

	// build flat tree structures
	// lambda -- parent body
	for (auto bi = bodies.begin(); bi < bodies.end(); ++bi) {
		if ((*bi)->id == 0) {
			assert((*bi)->parent_joints.size() == 0);
			lambda.push_back(-1);
			continue;
		}
		assert((*bi)->parent_joints.size() == 1);
		lambda.push_back((*(*bi)->parent_joints.begin())->b0->id);
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

	// configure Xp and Xs
	for (auto& j : loop_joints) {
		std::shared_ptr<Body> pb = j->b0; // predecessor body
		if (pb->id == 0) {
			// loop joint's predecessor is base
			XP.push_back(j->X_0_J0);
		}
		else {
			std::shared_ptr<Constraint> pj = tree_joints[pb->id]; // joint supporting predecessor body
			XP.push_back(j->X_0_J0 * inverse_transform(pj->X_1_J1));
		}
	}
	for (auto& j : loop_joints) {
		std::shared_ptr<Body> sb = j->b1; // successor body
		if (sb->id == 0) {
			// loop joint's successor is base
			XS.push_back(j->X_1_J1);
		}
		else {
			std::shared_ptr<Constraint> pj = tree_joints[sb->id]; // joint supporting predecessor body
			XS.push_back(j->X_1_J1 * inverse_transform(pj->X_1_J1));
		}
	}

	// velocity product acceleration
	a_vp.resize(bodies.size());

	// configure H
	std::vector<int> H_block_sizes;
	for (int i = 1; i < tree_joints.size(); ++i) {
		H_block_sizes.push_back(tree_joints[i]->S->cols());
	}
	H_acc = std::make_shared<BlockAccess>(H_block_sizes);
	H = JDyad::Zero(H_acc->total_rows(), H_acc->total_cols());
	
	// configure K
	std::vector<int> row_block_sizes;
	for (auto& j : loop_joints) {
		row_block_sizes.push_back(j->T->cols());
	}
	K_acc = std::make_shared<BlockAccess>(row_block_sizes, H_block_sizes);
	K = GPower::Zero(K_acc->total_rows(), K_acc->total_cols());

	// configure k
	k_acc = std::make_shared<BlockAccess>(row_block_sizes, std::vector<int>{1});
	_k = GPower::Zero(k_acc->total_rows(), k_acc->total_cols());

	return true;
}

void ArticulatedBody::move_bodies() {
	for (int i = 1; i < bodies.size(); ++i) {
		auto cb = bodies[i]; // child body
		auto pj = *cb->parent_joints.begin(); // parent joint
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
		std::shared_ptr<const Constraint> Ji = tree_joints[i];

		MVector& vi = Bi->v;
		MVector& ai = Bi->a;
		const MVector& vLi = BLi->v;
		const MVector& aLi = BLi->a;
		const MSubspace& Si = *(Ji->S);
		const MCoordinates& dqi = Ji->dq;
		const MCoordinates& ddqi = Ji->ddq;
		MTransform vci = std::move(derivative_cross(vi));

		vi = X_Li_[i] * vLi + Si * dqi;
		ai = X_Li_[i] * aLi + Si * ddqi + vci * Si * dqi;
	}

	return;
}

// treats loop joint active force as external force
// loop joint constraint force will be taken care of by the acceleration constraint
void ArticulatedBody::compute_bias_RNEA() {
	MVector v0 = MVector::Zero();
	MVector aw;
	aw << Eigen::Vector3f::Zero(), -gravity;
	MTransform X_W_0 = m_transform(Eigen::Matrix3f::Identity(), bodies[0]->bases, bodies[0]->translation);// from world space to body 0 space

	// std::vector<MVector> a(bodies.size());
	a_vp[0] = X_W_0 * aw;

	std::vector<FVector> f(bodies.size());
	f[0] = FVector::Zero();
	for (int i = 1; i < bodies.size(); ++i) {
		int Li = lambda[i];
		std::shared_ptr<const Body> Bi = bodies[i];
		std::shared_ptr<const Body> BLi = bodies[Li];
		std::shared_ptr<const Constraint> Ji = tree_joints[i];

		const MVector& vi = Bi->v;
		const MVector& vLi = BLi->v;
		const MSubspace& Si = *(Ji->S);
		const MCoordinates& dqi = Ji->dq;
		// MCoordinates& ddqi = Ji->ddq;
		MTransform vci = std::move(derivative_cross(vi));

		// a[i] = X_Li_[i] * a[Li] + Si * ddqi + vci * Si * dqi; // The OG acceleration iteration calculation
		a_vp[i] = X_Li_[i] * a_vp[Li] + vci * Si * dqi;

		const Dyad& Ii = Bi->I;
		f[i] = Ii * a_vp[i] + dual_transform(vci) * Ii * vi - dual_transform(X_0_[i]) * Bi->fe0;
	}
	// account for loop joint active force
	for (int k = 0; k < loop_joints.size(); ++k) {
		std::shared_ptr<Constraint> l = loop_joints[k];
		FVector fa = transpose_transform(XS[k]) * *(l->Ta) * l->taue; // conceptually it should be the dual of the inverse of XS
		//std::cout << XS[k] << std::endl;
		//std::cout << l->Ta->transpose() << std::endl;
		//std::cout << l->taue.transpose() << std::endl;
		int pk = l->b0->id;
		int sk = l->b1->id;
		f[sk] -= fa;
 		f[pk] += dual_transform(X_0_[pk]) * transpose_transform(X_0_[sk]) * fa;
	}

	for (int i = bodies.size() - 1; i >= 1; --i) {
		tree_joints[i]->bias = tree_joints[i]->S->transpose() * f[i];
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

		const MSubspace& Si = *tree_joints[i]->S;
		FVector F = Ic[i] * Si;
		H_acc->block(H, i - 1, i - 1) = Si.transpose() * F;
		int j = i;
		while (lambda[j] != 0) {
			F = X_Li_[j].transpose() * F;
			j = lambda[j];
			H_acc->block(H, i - 1, j - 1) = F.transpose() * *tree_joints[j]->S;
			H_acc->block(H, j - 1, i - 1) = H_acc->block(H, i - 1, j - 1).transpose();
		}
	} 
}

GPower ArticulatedBody::compute_delta(ConstraintType type, const MTransform& X) {
	GPower delta;
	delta.resize(T.at(type).cols(), 1);
	if (type == ConstraintType::Prismatic) {
		delta <<
			(X(1, 2) - X(2, 1)) * 0.5f,
			(X(2, 0) - X(0, 2)) * 0.5f,
			(X(0, 1) - X(1, 0)) * 0.5f,
			X(4, 2),
			-X(3, 2);
	}
	else if (type == ConstraintType::Revolute) {
		delta <<
			X(1, 2),
			-X(0, 2),
			X(4, 2),
			-X(3, 2),
			X(3, 0) * X(1, 0) + X(3, 1) * X(1, 1);
	}
	else {
		assert(false);
	}
	return delta;
}

void ArticulatedBody::compute_K_k() {
	K.setZero();
	for (int k = 0; k < loop_joints.size(); ++k) {
		int pk = loop_joints[k]->b0->id;
		int sk = loop_joints[k]->b1->id;
		const MTransform& X_0_pk = X_0_[pk]; // transform from base to loop joint k's predecessor
		const MTransform& X_0_sk = X_0_[sk]; // transform from base to loop joint k's successor
		MTransform Xp = XP[k] * X_0_pk;
		MTransform Xs = XS[k] * X_0_sk;
		MTransform X_pk_0 = std::move(inverse_transform(X_0_pk));
		MTransform X_sk_0 = std::move(inverse_transform(X_0_sk));
		MVector vp = X_pk_0 * bodies[pk]->v;
		MVector vs = X_sk_0 * bodies[sk]->v;
		MVector ap = X_pk_0 * a_vp[pk];
		MVector as = X_sk_0 * a_vp[sk];
		GPower delta = compute_delta(loop_joints[k]->type, Xs * inverse_transform(Xp));
		FSubspace T = transpose_transform(Xs) * *loop_joints[k]->T;
		GPower kstab = -2.0f * alpha * T.transpose() * (vs - vp) - beta * beta * delta;
		k_acc->block(_k, k, 0) = -T.transpose() * (as - ap + derivative_cross(vs) * vp) + kstab;
		int i = pk;
		int j = sk;
		while (i != j) {
			if (i > j) {
				K_acc->block(K, k, i - 1) = -T.transpose() * inverse_transform(X_0_[i]) * (*tree_joints[i]->S);
				i = lambda[i];
			}
			else {
				K_acc->block(K, k, j - 1) = T.transpose() * inverse_transform(X_0_[j]) * (*tree_joints[j]->S);
				j = lambda[j];
			}
		}
	}
}

Eigen::Matrix<float, 6, Eigen::Dynamic, 0, 6, 6> ArticulatedBody::subspace(std::initializer_list<std::array<float, 6>> columns) {
	if (columns.size() == 0) {
		return Eigen::Matrix<float, 6, Eigen::Dynamic, 0, 6, 6>::Zero(6, 0);
	}

	Eigen::Matrix<float, 6, Eigen::Dynamic, 0, 6, 6> mat(6, columns.size());

	int col_index = 0;
	for (const auto& column : columns) {
		for (int row = 0; row < 6; ++row) {
			mat(row, col_index) = column[row];
		}
		col_index++;
	}

	return mat;
}

}