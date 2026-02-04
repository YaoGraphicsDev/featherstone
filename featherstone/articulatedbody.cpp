#include "articulatedbody.hpp"

#include <iostream>
#include <queue>

namespace SPD {

// loop closure joint Baumguarte position correction parameters
const float alpha = 0.5f;
const float beta = 0.5f;

// spherical joint gyro damping
const float spherical_damping_coeff = 0.3f;

inline Eigen::Matrix3f so3_exp(const Eigen::Vector3f& w) {
	float theta = w.norm();
	if (theta < 1e-6f)
		return Eigen::Matrix3f::Identity();
	Eigen::Vector3f axis = w / theta;
	return Eigen::AngleAxisf(theta, axis).toRotationMatrix();
}

inline Eigen::Vector3f so3_log(const Eigen::Matrix3f& R) {
	Eigen::AngleAxisf aa(R);
	float theta = aa.angle();
	if (theta < 1e-6f)
		return Eigen::Vector3f::Zero();
	return aa.axis() * theta;
}

void ArticulatedBody::move_joints() {
	for (int i = 1; i < tree_joints.size(); ++i) {
		std::shared_ptr<Joint> j = tree_joints[i];

		// TODO: update X_J0_J1
		if (j->type == JointType::Revolute) {
			j->X_J0_J1 = m_transform(
				Eigen::Matrix3f::Identity(),
				Eigen::AngleAxisf(j->q[0], Eigen::Vector3f::UnitZ()).toRotationMatrix(),
				Eigen::Vector3f::Zero());
		}
		else if (j->type == JointType::Prismatic) {
			j->X_J0_J1 = m_transform(
				Eigen::Matrix3f::Identity(),
				Eigen::Matrix3f::Identity(),
				Eigen::Vector3f::UnitZ() * j->q[0]);
		}
		else if (j->type == JointType::Cylindrical) {
			j->X_J0_J1 = m_transform(
				Eigen::Matrix3f::Identity(),
				Eigen::AngleAxisf(j->q[0], Eigen::Vector3f::UnitZ()).toRotationMatrix(),
				Eigen::Vector3f::UnitZ() * j->q[1]);
		}
		else if (j->type == JointType::Spherical) {
			Eigen::Matrix3f rot = j->qs.toRotationMatrix();
			j->X_J0_J1 = m_transform(
				Eigen::Matrix3f::Identity(),
				rot,
				Eigen::Vector3f::Zero());
		}
		else {
			assert(false);
		}

		int Li = lambda[i];
		std::shared_ptr<const Joint> Ji = tree_joints[i];
		if (Li == 0) {
			X_Li_[i] = Ji->X_J0_J1 * Ji->X_0_J0;
		}
		else {
			std::shared_ptr<Joint> JLi = tree_joints[Li];
			X_Li_[i] = Ji->X_J0_J1 * Ji->X_0_J0 * inverse_transform(JLi->X_1_J1);
		}
		X_0_[i] = X_Li_[i] * X_0_[Li];
	}
}

void ArticulatedBody::solve_ddq(float dt) {
	// build tau and C
	FCoordinates tau;
	FCoordinates C;
	for (int i = 1; i < tree_joints.size(); ++i) {
		std::shared_ptr<const Joint> j = tree_joints[i];
		tau.conservativeResize(tau.size() + j->taue.size());
		tau.tail(j->taue.size()) = j->taue;
		C.conservativeResize(C.size() + j->bias.size());
		C.tail(j->bias.size()) = j->bias;
	}

	assert(tau.rows() == H.rows() && C.rows() == H.rows());
	
	FCoordinates tC = tau - C ; // tau - C
	if (enable_springs) {
		tC = tC /* + Ks * q0*/ /*q0 always start at 0*/ - (Ds + dt * Ks) * dq(true) - Ks * q(true);
	}

	auto KH_inv = [&](const Eigen::LLT<JDyad>& H_llt, const GPower& K) -> Unitless { // K * H^-1
		return (H_llt.solve(K.transpose())).transpose();
	};
	// loop joint force variale lambda
	Unitless lambda = Unitless::Zero(K.rows(), 1);
	if (!loop_joints.empty()) {
		if (constraints.empty()) {
			Unitless KHi;
			if (!enable_springs) {
				KHi = KH_inv(H_llt, K);
			}
			else {
				KHi = KH_inv(H_spring_llt, K);
			}
			Unitless A = KHi * K.transpose();
			Unitless b = _k - KHi * tC;
			 InvOrPinvSolver A_inv(A);
			 lambda = A_inv.solve(b);
		}
		else {
			Unitless KHi;
			if (!enable_springs) {
				KHi = KH_inv(H_reduced_llt, K_reduced);
			}
			else {
				KHi = KH_inv(H_spring_reduced_llt, K_reduced);
			}
			Unitless A = KHi * K_reduced.transpose();
			Unitless b = /*ZT **/ _k - KHi * (ZT * tC);
			InvOrPinvSolver A_inv(A);
			lambda = A_inv.solve(b);
		}
	}


	MCoordinates ddq = MCoordinates::Zero(H.cols(), 1);

	if (constraints.empty()) {
		if (loop_joints.empty()) {
			if (!enable_springs) {
				ddq = H_llt.solve(tC);
			}
			else {
				ddq = H_spring_llt.solve(tC);
			}
		}
		else {
			if (!enable_springs) {
				ddq = H_llt.solve(tC + K.transpose() * lambda);
			}
			else {
				ddq = H_spring_llt.solve(tC + K.transpose() * lambda);
			}
		}
	}
	else {
		MCoordinates ddq_reduced;
		if (loop_joints.empty()) {
			if (!enable_springs) {
				ddq_reduced = H_reduced_llt.solve(ZT * tC);
			}
			else {
				ddq_reduced = H_spring_reduced_llt.solve(ZT * tC);
			}
		}
		else {
			if (!enable_springs) {
				ddq_reduced = H_reduced_llt.solve(ZT * tC + (K * Z).transpose() * lambda);
			}
			else {
				ddq_reduced = H_spring_reduced_llt.solve(ZT * tC + (K * Z).transpose() * lambda);
			}
		}
		
		// recover full ddq from reduced
		ddq = Z * ddq_reduced;
	}

	int count = 0;
	for (int i = 1; i < tree_joints.size(); ++i) {
		std::shared_ptr<Joint> j = tree_joints[i];
		j->ddq = ddq.segment(count, j->ddq.rows());
		count += j->ddq.rows();
	}
}

void ArticulatedBody::integrate_velocity(float dt) {
	project_velocity();
	compute_bias_RNEA();
	if (enable_springs) {
		compute_H_spring(dt);
	}
	else {
		compute_H();
	}
	if (!loop_joints.empty()) {
		compute_K_k();
	}
	solve_ddq(dt);
	for (int i = 1; i < tree_joints.size(); ++i) {
		std::shared_ptr<Joint> j = tree_joints[i];
		j->dq += j->ddq * dt;
	}

	// TODO: this should be useless. As contact solver only uses joint space velocity 
	// and will also project velocity after it's done iterating
	// project_velocity();
}

void ArticulatedBody::integrate_position(float dt) {
	for (int i = 1; i < tree_joints.size(); ++i) {
		std::shared_ptr<Joint> j = tree_joints[i];
		if (j->type == JointType::Spherical) {
			Eigen::Vector3f w = j->dq * dt;
			float theta = w.norm();
			Eigen::Quaternionf dqrot = (theta < 1e-6f)
				? Eigen::Quaternionf::Identity()
				: Eigen::Quaternionf(Eigen::AngleAxisf(theta, w / theta));
			j->qs = (j->qs * dqrot).normalized();
			j->q = so3_log(j->qs.toRotationMatrix());
		}
		else {
			j->q += j->dq * dt;
		}
	}
	move_joints();
	project_position();
}

void ArticulatedBody::project_velocity() {
	for (int i = 1; i < bodies.size(); ++i) {
		int Li = lambda[i];
		std::shared_ptr<Body> Bi = bodies[i];
		std::shared_ptr<const Body> BLi = bodies[Li];
		std::shared_ptr<const Joint> Ji = tree_joints[i];

		MVector& vi = Bi->v;
		// MVector& ai = Bi->a;
		const MVector& vLi = BLi->v;
		// const MVector& aLi = BLi->a;
		const MSubspace& Si = *(Ji->S);
		const MCoordinates& dqi = Ji->dq;
		// const MCoordinates& ddqi = Ji->ddq;
		// MTransform vci = std::move(derivative_cross(vi));

		vi = X_Li_[i] * vLi + Si * dqi;
		// ai = X_Li_[i] * aLi + Si * ddqi + vci * Si * dqi;
	}
}

std::shared_ptr<ArticulatedBody::Body> ArticulatedBody::add_body(const RigidBody& rb) {
	std::shared_ptr<Body> body = std::make_shared<Body>();
	body->name = rb.name;
	body->shape = rb.shape;
	body->rotation = rb.rotation;
	body->translation = rb.translation;
	body->bases = rb.rotation.toRotationMatrix();
	body->restitution_coeff = rb.restitution_coeff;
	body->friction_coeff = rb.friction_coeff;
	body->mass = rb.mass;
	bodies.push_back(body);
	return body;
}

std::shared_ptr<ArticulatedBody::Joint> ArticulatedBody::add_joint(
	std::string name,
	JointType type,
	size_t id0,
	size_t id1,
	Eigen::Matrix3f base0,
	Eigen::Vector3f trans0,
	bool disable_collision,
	std::vector<SpringParam> spring_params) {
	return add_joint(name, type, bodies[id0], bodies[id1], base0, trans0, disable_collision, spring_params);
}

std::shared_ptr<ArticulatedBody::Joint> ArticulatedBody::add_joint(
	std::string name,
	JointType type,
	std::shared_ptr<Body> b0,
	std::shared_ptr<Body> b1,
	Eigen::Matrix3f base0,
	Eigen::Vector3f trans0,
	bool disable_collision,
	std::vector<SpringParam> spring_params) {

	assert(b0 && b1);
	std::shared_ptr<Joint> c = std::make_shared<Joint>();
	c->name = name;
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

	if (type == JointType::Revolute ||
		type == JointType::Prismatic) {
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
	else if (type == JointType::Cylindrical) {
		c->q.resize(2, 1);
		c->q << 0.0f, 0.0f;
		c->dq.resize(2, 1);
		c->dq << 0.0f, 0.0f;
		c->ddq.resize(2, 1);
		c->ddq << 0.0f, 0.0f;
		c->bias.resize(2, 1);
		c->bias << 0.0f, 0.0f;
		c->taue.resize(2, 1);
		c->taue << 0.0f, 0.0f;
	}
	else if (type == JointType::Spherical) {
		c->q.resize(3, 1);  
		c->q << 0.0f, 0.0f, 0.0f;
		c->qs = Eigen::Quaternionf::Identity();
		c->dq.resize(3, 1);
		c->dq << 0.0f, 0.0f, 0.0f;
		c->ddq.resize(3, 1);
		c->ddq << 0.0f, 0.0f, 0.0f;
		c->bias.resize(3, 1);
		c->bias << 0.0f, 0.0f, 0.0f;
		c->taue.resize(3, 1);
		c->taue << 0.0f, 0.0f, 0.0f;
	}
	else {
		assert(false);
	}
	c->disable_collision = disable_collision;

	if (!spring_params.empty()) {
		c->enable_spring = true;
		c->ks = MCoordinates::Zero(c->ddq.size(), 1);
		c->ds = MCoordinates::Zero(c->ddq.size(), 1);
		for (int i = 0; i < std::min((size_t)c->ks.rows(), spring_params.size()); ++i) {
			c->ks(i) = std::abs(spring_params[i].k);
			c->ds(i) = std::abs(spring_params[i].d);
		}
	}

	tree_joints.push_back(c);

	b0->children_joints.push_back(c);
	b1->parent_joints.push_back(c);

	// Transform from com to J (joint space)
	MTransform X_com_J = m_transform(Eigen::Matrix3f::Identity(), c->bb1, c->bt1 - b1->shape->com);
	if (b1->shape) {
		b1->I = transform_dyad(X_com_J, b1->shape->Ic6) * b1->mass / b1->shape->vol;//  dual_transform(X_C_J) * b1->shape->sp_Ic* inverse_transform(X_C_J);
	}

	return c;
}

std::shared_ptr<ArticulatedBody::Joint> ArticulatedBody::get_joint(const std::string& name) const {
	for (auto j : tree_joints) {
		if (!j) {
			continue;
		}
		if (j->name == name) {
			return j;
		}
	}
	for (auto j : loop_joints) {
		if (j->name == name) {
			return j;
		}
	}
	return nullptr;
}

std::shared_ptr<ArticulatedBody::Body> ArticulatedBody::get_body(const std::string& name) const {
	for (auto b : bodies) {
		if (b->name == name) {
			return b;
		}
	}
	return nullptr;
}

std::shared_ptr<ArticulatedBody::Constraint> ArticulatedBody::add_constraint(std::string name, std::shared_ptr<Joint> j0, std::shared_ptr<Joint> j1, float r01) {
	if (j0->S->cols() != 1 || j1->S->cols() != 1) {
		// Does not work on joints that have more than 1 dimensional motion subspace
		assert(false);
		return nullptr;
	}

	std::shared_ptr<Constraint> c = std::make_shared<Constraint>();
	c->name = name;
	c->j0 = j0;
	c->msubspace_col0 = 0;
	c->j1 = j1;
	c->msubspace_col1 = 0;
	c->r01 = r01;

	constraints.push_back(c);
	return c;
}

std::shared_ptr<ArticulatedBody::Constraint> ArticulatedBody::add_constraint(std::string name,
	std::shared_ptr<Joint> j0, int msubspace_col0,
	std::shared_ptr<Joint> j1, int msubspace_col1, float r01) {
	if (j0->type == JointType::Spherical || j1->type == JointType::Spherical) {
		std::cout << "Constrained spherical joint untested. What are you trying to achieve by constraining spherical joint anyway?" << std::endl;
		assert(false);
		return nullptr;
	}
	if (j0->S->cols() <= msubspace_col0 || j1->S->cols() <= msubspace_col1) {
		assert(false);
		return nullptr;
	}

	std::shared_ptr<Constraint> c = std::make_shared<Constraint>();
	c->name = name;
	c->j0 = j0;
	c->msubspace_col0 = msubspace_col0;
	c->j1 = j1;
	c->msubspace_col1 = msubspace_col1;
	c->r01 = r01;

	constraints.push_back(c);
	return c;
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
		for (std::shared_ptr<Joint> c : b->children_joints) {
			if (visited.find(c->b1.get()) == visited.end()) {
				// body not visited yet, no loop
				q.push(c->b1);
				visited.insert(c->b1.get());
			}
			else {
				// loop
				// 1. store loop joint 2. remove parent joint
				loop_joints.push_back(c);
				c->b1->parent_joints.remove_if([&](std::shared_ptr<Joint> j) {
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
	
	std::vector<std::shared_ptr<Joint>> sorted_joints(bodies.size(), nullptr);
	for (int i = 1; i < bodies.size(); ++i) {
		if (bodies[i]->parent_joints.empty()) {
			// this body is not connected to the tree
			assert(false);
			return false;
		}
		sorted_joints[i] = *bodies[i]->parent_joints.begin();
	}
	tree_joints = std::move(sorted_joints);
	for (int i = 1; i < tree_joints.size(); ++i) {
		tree_joints[i]->id = i;
	}
	for (int i = 0; i < loop_joints.size(); ++i) {
		loop_joints[i]->id = -1;
	}
	

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

	X_w_0 = m_transform(Eigen::Matrix3f::Identity(), bodies[0]->bases, bodies[0]->translation);
	X_0_w = inverse_transform(X_w_0);
	X_0_.resize(bodies.size());
	X_0_[0] = MTransform::Identity();
	X_Li_.resize(bodies.size());
	X_Li_[0] = MTransform::Identity();

	move_joints();

	// configure Xp and Xs
	for (auto& j : loop_joints) {
		std::shared_ptr<Body> pb = j->b0; // predecessor body
		if (pb->id == 0) {
			// loop joint's predecessor is base
			XP.push_back(j->X_0_J0);
		}
		else {
			std::shared_ptr<Joint> pj = tree_joints[pb->id]; // joint supporting predecessor body
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
			std::shared_ptr<Joint> pj = tree_joints[sb->id]; // joint supporting predecessor body
			XS.push_back(j->X_1_J1 * inverse_transform(pj->X_1_J1));
		}
	}
	// sequential impulse
	for (auto& j : loop_joints) {
		SI.push_back(FCoordinates::Zero(j->T->cols(), 1));
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
	
	if (!loop_joints.empty()) {
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
	}

	// configure spring matrices
	for (auto j : loop_joints) {
		if (j->enable_spring) {
			std::cout << "spring configuration on loop closure joint [" << j->name << "] will be ignored" << std::endl;
			assert(false);
		}
	}

	Ks = Unitless::Zero(H.rows(), H.cols());
	Ds = Unitless::Zero(H.rows(), H.cols());
	for (int i = 1; i < tree_joints.size(); ++i) {
		auto j = tree_joints[i];
		if (j->enable_spring) {
			auto& block = H_acc->blocks[i - 1][i - 1];
			assert(j->ks.rows() == block.cols);
			assert(j->ds.rows() == block.cols);
			Ks.diagonal().segment(block.start_col, block.cols) = j->ks;
			Ds.diagonal().segment(block.start_col, block.cols) = j->ds;
			enable_springs |= true;
		}
		// mitigate gyro drift when there is no damping or damping too small
		if (j->type == JointType::Spherical) {
			assert(j->b1->id > 0); // after building the dynamic tree, the child body of a tree joint is guaranteed not to be base
			float damping_mass = j->b1->mass;
			auto& block = H_acc->blocks[i - 1][i - 1];
			Ds.diagonal().segment(block.start_col, block.cols) 
				= Ds.diagonal().segment(block.start_col, block.cols).cwiseMax(damping_mass * spherical_damping_coeff);
			enable_springs |= true;
		}
	}

	// configure constraints
	for (std::shared_ptr<Constraint> c : constraints) {
		c->C = MCoordinates::Zero(H.cols(), 1);
		int j0_id = c->j0->id;
		int j1_id = c->j1->id;
		if (j0_id < 0 || j1_id < 0) {
			if (j0_id < 0) {
				std::cout << "constraint does not work on loop joint [" << c->j0->name << "]" << std::endl;
				assert(false);
			}
			else {
				std::cout << "constraint does not work on loop joint [" << c->j1->name << "]" << std::endl;
				assert(false);
			}
		}
		else {
			c->C(H_acc->blocks[j0_id - 1][j0_id - 1].start_col + c->msubspace_col0) = 1.0f;
			c->C(H_acc->blocks[j1_id - 1][j1_id - 1].start_col + c->msubspace_col1) = c->r01;
		}
	}

	if (!constraints.empty()) {
		// build constraint matrix
		Unitless C = Unitless::Zero(H.cols(), constraints.size());
		for (int i = 0; i < constraints.size(); ++i) {
			C.col(i) = constraints[i]->C;
		}

		// compute ortho complement of constraint matrix
		Z = ortho_complement_QR(C);
		ZT = Z.transpose();

		// If Z has 0 columns, that means all dofs are lost, fully constrained
		if (Z.cols() == 0) {
			std::cout << "mechanism fully constrained. No DoF left." << std::endl;
			assert(false);
		}
	}

	return true;
}

void ArticulatedBody::project_position() {
	for (int i = 1; i < bodies.size(); ++i) {
		auto cb = bodies[i]; // child body
		auto pj = *cb->parent_joints.begin(); // parent joint
		auto pb = pj->b0; // parent body

		Eigen::Matrix3f joint_bases0_world = pb->bases * pj->bb0;
		Eigen::Matrix3f joint_bases1_world = joint_bases0_world;
		if (pj->type == JointType::Revolute || pj->type == JointType::Cylindrical) {
			joint_bases1_world = Eigen::AngleAxisf(pj->q(0, 0), joint_bases0_world.col(2)).toRotationMatrix() * joint_bases0_world;
		}
		else if (pj->type == JointType::Prismatic) {
			joint_bases1_world = joint_bases0_world;
		}
		else if (pj->type == JointType::Spherical) {
			// Eigen::Matrix3f rot = so3_exp(pj->q);         // joint0 -> joint1 in joint0 coords
			Eigen::Matrix3f rot = pj->qs.toRotationMatrix();
			joint_bases1_world = joint_bases0_world * rot;
		}
		else {
			assert(false);
		}

		Eigen::Vector3f joint_translation0_world = pb->bases * pj->bt0 + pb->translation;
		Eigen::Vector3f joint_translation1_world = joint_translation0_world;
		if (pj->type == JointType::Revolute || pj->type == JointType::Spherical) {
			joint_translation1_world = joint_translation0_world;
		}
		else if (pj->type == JointType::Prismatic) {
			joint_translation1_world = joint_bases0_world.col(2) * pj->q(0, 0) + joint_translation0_world;
		}
		else if (pj->type == JointType::Cylindrical) {
			joint_translation1_world = joint_bases0_world.col(2) * pj->q(1, 0) + joint_translation0_world;
		}
		else {
			assert(false);
		}

		// body 1 bases in joint space 1
		Eigen::Matrix3f body1_bases_joint1 = pj->bb1.transpose();
		Eigen::Matrix3f body1_bases_world =  joint_bases1_world * body1_bases_joint1;
		cb->rotation = Eigen::Quaternionf(body1_bases_world);
		cb->rotation.normalize();
		cb->bases = cb->rotation.toRotationMatrix();

		Eigen::Vector3f body1_translation_joint1 = -body1_bases_joint1 * pj->bt1;
		Eigen::Vector3f body1_translation_world = joint_translation1_world + joint_bases1_world * body1_translation_joint1;
		cb->translation = body1_translation_world;
	}

	return;
}


MSubspace ArticulatedBody::jacobian_0(size_t body_id) {
	MSubspace J0 = MSubspace::Zero(6, H.rows());

	if (body_id == 0 || body_id >= bodies.size()) {
		assert(false);
		return J0;
	}
	assert(H.rows() == H.cols());
	assert(body_id <= H_acc->blocks.size());
	size_t i = body_id;
	while (i != 0) {
		const MSubspace& s = *tree_joints[i]->S;
		int start_col = H_acc->blocks[i - 1][i - 1].start_col;
		assert(H_acc->blocks[i - 1][i - 1].cols == s.cols());
		J0.middleCols(start_col, s.cols()) = inverse_transform(X_0_[i]) * s;
		i = lambda[i];
	}

	if (!constraints.empty()) {
		J0 = J0 * Z;
	}

	return J0;
}

MCoordinates ArticulatedBody::q(bool full_stack) {
	MCoordinates q_chain = MCoordinates::Zero(H.rows());

	// unlike the Jacobian
	// we need a full chain of dq because constrained joints may not be on the tree to the root
	for (int i = 1; i < tree_joints.size(); ++i) {
		int offset = H_acc->blocks[i - 1][i - 1].start_row;
		int n = H_acc->blocks[i - 1][i - 1].rows;
		q_chain.segment(offset, n) = tree_joints[i]->q;
	}

	if (!constraints.empty() && !full_stack) {
		q_chain = ZT * q_chain;
	}

	return q_chain;
}

MCoordinates ArticulatedBody::dq(bool full_stack) {
	MCoordinates dq_chain = MCoordinates::Zero(H.rows());

	// unlike the Jacobian
	// we need a full chain of dq because constrained joints may not be on the tree to the root
	for (int i = 1; i < tree_joints.size(); ++i) {
		int offset = H_acc->blocks[i - 1][i - 1].start_row;
		int n = H_acc->blocks[i - 1][i - 1].rows;
		dq_chain.segment(offset, n) = tree_joints[i]->dq;
	}

	if (!constraints.empty() && !full_stack) {
		dq_chain = ZT * dq_chain;
	}

	return dq_chain;
}

void ArticulatedBody::apply_delta_dq(MCoordinates delta_dq) {
	if (!constraints.empty()) {
		assert(delta_dq.rows() == H_reduced.cols());
		delta_dq = Z * delta_dq;
	}

	assert(delta_dq.rows() == H.cols());
	for (int i = 1; i < tree_joints.size(); ++i) {
		int offset = H_acc->blocks[i - 1][i - 1].start_col;
		int n = H_acc->blocks[i - 1][i - 1].cols;
		assert(n == tree_joints[i]->dq.size());
		tree_joints[i]->dq += delta_dq.segment(offset, n);
	}
}

void ArticulatedBody::apply_delta_q(MCoordinates delta_q) {
	if (!constraints.empty()) {
		assert(delta_q.rows() == H_reduced.cols());
		delta_q = Z * delta_q;
	}

	assert(delta_q.rows() == H.cols());
	for (int i = 1; i < tree_joints.size(); ++i) {
		int offset = H_acc->blocks[i - 1][i - 1].start_col;
		int n = H_acc->blocks[i - 1][i - 1].cols;
		assert(n == tree_joints[i]->q.size());

		auto j = tree_joints[i];
		if (j->type == JointType::Spherical) {
			Eigen::Quaternionf qs_inc(so3_exp(delta_q.segment(offset, n)));
			j->qs = j->qs * qs_inc;
			
			
			//Eigen::Vector3f dtheta = delta_q.segment(offset, n);
			//Eigen::Matrix3f rot = so3_exp(j->q);
			//Eigen::Matrix3f rot_inc = so3_exp(dtheta);
			//assert(false); // check order
			//rot = rot_inc * rot;
			j->q = so3_log(j->qs.toRotationMatrix());
		}
		else {
			j->q += delta_q.segment(offset, n);
		}
	}
}

JDyad ArticulatedBody::H_inv(const Unitless& J) {
	if (constraints.empty()) {
		return H_llt.solve(J);
	}
	else {
		return H_reduced_llt.solve(J);
	}
}

// treats loop joint active force as external force
// loop joint constraint force will be taken care of by the acceleration constraint
// Same applies to constraints and as constraints do not provide active force, they do not participate in RNEA
void ArticulatedBody::compute_bias_RNEA() {
	MVector v0 = MVector::Zero();
	MVector aw;
	aw << Eigen::Vector3f::Zero(), -gravity;
	// MTransform X_W_0 = m_transform(Eigen::Matrix3f::Identity(), bodies[0]->bases, bodies[0]->translation);// from world space to body 0 space

	// std::vector<MVector> a(bodies.size());
	a_vp[0] = X_w_0 * aw;

	std::vector<FVector> f(bodies.size());
	f[0] = FVector::Zero();
	for (int i = 1; i < bodies.size(); ++i) {
		int Li = lambda[i];
		std::shared_ptr<const Body> Bi = bodies[i];
		std::shared_ptr<const Body> BLi = bodies[Li];
		std::shared_ptr<const Joint> Ji = tree_joints[i];

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
		std::shared_ptr<Joint> l = loop_joints[k];
		FVector fa = transpose_transform(XS[k]) * *(l->Ta) * l->taue; // conceptually it should be the dual of the inverse of XS
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

void ArticulatedBody::compute_H_spring(float dt) {
	compute_H();
	assert(enable_springs);
	H_spring = H + dt * dt * Ks + dt * Ds;
	H_spring_llt = H_spring.llt();
	assert(H_spring_llt.info() == Eigen::Success);
	if (!constraints.empty()) {
		H_spring_reduced = ZT * H_spring * Z;
		H_spring_reduced_llt = H_spring_reduced.llt();
		assert(H_spring_reduced_llt.info() == Eigen::Success);
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
		Unitless F = Ic[i] * Si;
		H_acc->block(H, i - 1, i - 1) = Si.transpose() * F;
		int j = i;
		while (lambda[j] != 0) {
			F = X_Li_[j].transpose() * F;
			j = lambda[j];
			H_acc->block(H, i - 1, j - 1) = F.transpose() * *tree_joints[j]->S;
			H_acc->block(H, j - 1, i - 1) = H_acc->block(H, i - 1, j - 1).transpose();
		}
	}

	H_llt = H.llt();
	assert(H_llt.info() == Eigen::Success);

	if (!constraints.empty()) {
		H_reduced = ZT * H * Z;
		H_reduced_llt = H_reduced.llt();
		assert(H_reduced_llt.info() == Eigen::Success);
	}
}

GPower ArticulatedBody::compute_delta(JointType type, const MTransform& X) {
	GPower delta;
	delta.resize(T.at(type).cols(), 1);
	if (type == JointType::Prismatic) {
		delta <<
			(X(1, 2) - X(2, 1)) * 0.5f,
			(X(2, 0) - X(0, 2)) * 0.5f,
			(X(0, 1) - X(1, 0)) * 0.5f,
			X(4, 2),
			-X(3, 2);
	}
	else if (type == JointType::Revolute) {
		delta <<
			X(1, 2),
			-X(0, 2),
			X(4, 2),
			-X(3, 2),
			X(3, 0) * X(1, 0) + X(3, 1) * X(1, 1);
	}
	else if (type == JointType::Cylindrical) {
		delta <<
			X(1, 2),
			-X(0, 2),
			X(4, 2),
			-X(3, 2);
	}
	else if (type == JointType::Spherical) {
		delta <<
			X(4, 0) * X(2, 0) + X(4, 1) * X(2, 1) + X(4, 2) * X(2, 2),
			X(5, 0) * X(0, 0) + X(5, 1) * X(0, 1) + X(5, 2) * X(0, 2),
			X(3, 0) * X(1, 0) + X(3, 1) * X(1, 1) + X(3, 2) * X(1, 2);
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

	if (!constraints.empty()) {
		// K_reduced = ZT * K * Z;
		K_reduced = K * Z;
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