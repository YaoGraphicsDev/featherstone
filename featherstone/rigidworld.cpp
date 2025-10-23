#include "rigidworld.h"
#include "math_utils.h"

#include "BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h"

#include <algorithm>
#include <iostream>

namespace SPD {

using namespace Eigen;

std::shared_ptr<RigidWorld::Collider> RigidWorld::Collider::create(const RigidBody& rigidbody, int user_id) {
	btCollisionShape* shape = nullptr;
	if (rigidbody.shape->type == Shape::Type::Cuboid) {
		const Cuboid* box = static_cast<Cuboid*>(rigidbody.shape.get());
		shape = new btBoxShape(btv3(box->half_dims));
	}
	else {
		assert(false);
	}

	btCollisionObject* obj = new btCollisionObject();
	obj->setCollisionShape(shape);
	btTransform transform(
		btquat(rigidbody.rotation),
		btv3(rigidbody.translation));
	obj->setWorldTransform(transform);
	obj->setUserIndex(user_id);

	std::shared_ptr<Collider> collider = std::make_shared<Collider>();
	collider->shape.reset(shape);
	collider->obj.reset(obj);
	return collider;
}

void RigidWorld::Collider::update(Eigen::Vector3f translation, Eigen::Quaternionf rotation) {
	btTransform transform(
		btquat(rotation),
		btv3(translation));
	obj->setWorldTransform(transform);
}

RigidWorld::RigidWorld(Eigen::Vector3f gravity) {
	this->gravity = gravity;

	btDefaultCollisionConfiguration* config = new btDefaultCollisionConfiguration();
	btCollisionDispatcher* dispatcher = new btCollisionDispatcher(config );
	btDbvtBroadphase* broadphase = new btDbvtBroadphase();
	btCollisionWorld* world = new btCollisionWorld(dispatcher, broadphase, config);

	collision_world.reset(new CollisionWorld);
	collision_world->config.reset(config);
	collision_world->dispatcher.reset(dispatcher);
	collision_world->broadphase.reset(broadphase);
	collision_world->world.reset(world);

	contact_solver.reset(new ContactSolver());
}

RigidWorld::~RigidWorld() {
	for (auto c : colliders) {
		collision_world->world->removeCollisionObject(c->obj.get());
	}
	gContactDestroyedCallback = nullptr;
	collision_world->world.reset();
	collision_world->broadphase.reset();
	collision_world->dispatcher.reset();
	collision_world->config.reset();
}

void RigidWorld::add_body(std::shared_ptr<RigidBody> body) {
	if (!body->shape || body->shape->type == Shape::Type::Default) {
		return;
	}

	bodies.push_back(body);
	std::shared_ptr<Collider> c = Collider::create(*body, bodies.size() - 1);
	colliders.push_back(c);
	collision_world->world->addCollisionObject(c->obj.get());
}

static int step_count = 0;

void RigidWorld::step(float dt) {
	std::cout << "step = " << step_count++ << std::endl;

	collide();
	integrate_velocity(dt);

	contact_solver->initialize(bodies, collision_world->dispatcher);
	contact_solver->warm_start();
	for (uint32_t i = 0; i < max_velocity_solve_iterations; ++i) {
		std::cout << "\titeration = " << i << std::endl;
		contact_solver->solve_velocity();
	}

	integrate_position(dt);

	for (uint32_t i = 0; i < max_position_solve_iterations; ++i) {
		contact_solver->solve_position();
	}
}

// parameters in world space
static FVector force_on_com(Vector3f f, Vector3f fp, Vector3f com, Matrix3f com_bases) {
	//Vector3f n = f.cross(-fp);
	//FVector f6;
	//f6 << n, f;
	//FTransform t = dual_transform(m_transform(Matrix3f::Identity(), com_bases, com));
	//f6 = t * f6;
	//return f6;

	Vector3f torque = f.cross(com - fp);
	torque = com_bases.inverse() * torque;
	Vector3f force = com_bases.inverse() * f;

	FTransform t = dual_transform(m_transform(Matrix3f::Identity(), com_bases, com - fp));
	FVector f6;
	f6 << Vector3f::Zero(), f;
	f6 = t * f6;
	//// temp
	//f6.head<3>() = Vector3f::Zero();
 	return f6;
}

void RigidWorld::collide() {
	for (int i = 0; i < bodies.size(); ++i) {
		RigidBody& b = *bodies[i];
		Collider& c = *colliders[i];
		if (b.type == RigidBody::DynamicType::Static) {
			continue;
		}
		c.update(b.translation, b.rotation);
		collision_world->world->updateSingleAabb(c.obj.get());
	}

	collision_world->world->performDiscreteCollisionDetection();
}

void RigidWorld::integrate_velocity(float dt) {
	for (auto b : bodies) {
		if (b->type == RigidBody::DynamicType::Static) {
			continue;
		}

		FVector g6;
		g6 << Vector3f::Zero(), gravity;
		// g6 = dual_transform(m_transform(Matrix3f::Identity(), b->bases, Vector3f::Zero())) * g6;
		FVector fe_com = b->fe + g6 * b->mass;
		b->fe = FVector::Zero();

		MTransform Xr = m_transform(Matrix3f::Identity(), b->rotation.toRotationMatrix(), Vector3f::Zero()); // a rotation to accomodate inertia's rotation
		Dyad I = transform_dyad2(Xr, b->shape->sp_Ic);
		// FVector bias = dual_transform(derivative_cross(b->v)) * (b->mass * b->shape->sp_Ic) * b->v;
		FVector bias = dual_transform(derivative_cross(b->v)) * (b->mass * I) * b->v;
		MVector a = transform_inv_dyad2(Xr, b->shape->sp_inv_Ic) * b->inv_mass * (fe_com - bias);
		// MTransform X_ortho = m_transform(b->bases, Matrix3f::Identity(), Vector3f::Zero());
		MVector v_ortho = /*X_ortho */ b->v;
		MVector a_ortho = /*X_ortho */ a;

		Vector3f v_ang = v_ortho.head<3>();
		Vector3f v_linear = v_ortho.tail<3>();
		Vector3f a_ang = a_ortho.head<3>();
		Vector3f a_linear = a_ortho.tail<3>() + cross_mat(v_ang) * v_linear;

		// implicit euler on ortho linear velocity
		v_linear += a_linear * dt;
		v_linear *= std::pow(1.0f - b->linear_damping, dt); // linear damping

		// implicit euler on ortho angular velocity
		v_ang += a_ang * dt;
		v_ang *= std::pow(1.0f - b->angular_damping, dt); // angular damping

		v_ortho << v_ang, v_linear;
		b->v = /*inverse_transform(X_ortho) */ v_ortho;
	}
}

void RigidWorld::integrate_position(float dt) {
	// TODO: for an accurate integration, see line 1103 of btDiscretePhysicsWorld.cpp predictIntegratedTransform
	for (auto b : bodies) {
		// implicit euler on ortho linear velocity
		Vector3f v_linear = b->v.tail<3>();
		b->translation += v_linear * dt;

		// implicit euler on ortho angular velocity
		Vector3f v_ang = b->v.head<3>();
		Quaternionf q = b->rotation;
		Quaternionf dq = q * Quaternionf(0.0f, v_ang.x(), v_ang.y(), v_ang.z());
		dq.coeffs() *= 0.5f;
		Quaternionf dqdt;
		dqdt.coeffs() = dq.coeffs() * dt;
		q.coeffs() = q.coeffs() + dqdt.coeffs();
		b->rotation = q;
		b->rotation.normalize();
	}
}

// return a negative value if penetration happens
float RigidWorld::new_penetration(
	const RigidBody& b0, Vector3f local_p0,
	const RigidBody& b1, Vector3f local_p1,
	Vector3f n_01) {
	Vector3f p0 = b0.rotation.toRotationMatrix() * local_p0 + b0.translation;
	Vector3f p1 = b1.rotation.toRotationMatrix() * local_p1 + b1.translation;
	float proj = n_01.dot((p0 - p1));
	return -proj;
}

Eigen::Vector3f RigidWorld::find_penetration(const btCollisionObject* obj0, const btCollisionObject* obj1) {
	//btCollisionObjectWrapper obj0_wrap(nullptr, obj0->getCollisionShape(), obj0, obj0->getWorldTransform(), -1, -1);
	//btCollisionObjectWrapper obj1_wrap(nullptr, obj1->getCollisionShape(), obj1, obj1->getWorldTransform(), -1, -1);
	//// Get the collision algorithm for this pair
	//btCollisionAlgorithm* algo = collision_world->dispatcher->findAlgorithm(&obj0_wrap, &obj1_wrap, nullptr, BT_CONTACT_POINT_ALGORITHMS);
	//assert(algo);
	//// Create temporary manifold storage
	//btManifoldResult manifold_result(&obj0_wrap, &obj1_wrap);
	//btDispatcherInfo dispatchInfo;
	//dispatchInfo.m_dispatchFunc = btDispatcherInfo::DISPATCH_DISCRETE;
	////dispatchInfo.m_timeOfImpact = 1.0f;
	//dispatchInfo.m_useContinuous = false;
	////dispatchInfo.m_debugDraw = nullptr;
	////dispatchInfo.m_enableSatConvex = false;
	////dispatchInfo.m_enableSPU = true;
	////dispatchInfo.m_useEpa = true;
	//dispatchInfo.m_allowedCcdPenetration = 0.0f;
	//// Perform full narrowphase collision detection
	//algo->processCollision(&obj0_wrap, &obj1_wrap, dispatchInfo, &manifold_result);
	//btPersistentManifold* manifold = manifold_result.getPersistentManifold();

	btGjkEpaPenetrationDepthSolver epaSolver;
	btVoronoiSimplexSolver simplexSolver;

	const btConvexShape* convex0 = nullptr;
	const btConvexShape* convex1 = nullptr;
	const btCollisionShape* shape0 = obj0->getCollisionShape();
	const btCollisionShape* shape1 = obj1->getCollisionShape();
	if (shape0->isConvex()) {
		convex0 = static_cast<const btConvexShape*>(shape0);
	}
	else {
		assert(false);
		return Vector3f::Zero();
	}
	if (shape1->isConvex()) {
		convex1 = static_cast<const btConvexShape*>(shape1);
	}
	else {
		assert(false);
		return Vector3f::Zero();
	}

	if (!convex0 || !convex1) {
		// One or both shapes are not convex
		assert(false);
		return Vector3f::Zero();
	}
	const btTransform& transform0 = obj0->getWorldTransform();
	const btTransform& transform1 = obj1->getWorldTransform();

	btVector3 penetration_vec;
	btVector3 witness0, witness1;

	if (epaSolver.calcPenDepth(
		simplexSolver, convex0, convex1,
		transform0, transform1,
		penetration_vec, witness0, witness1, nullptr)) {
		Vector3f vec = EV3(witness0) - EV3(witness1); // TODO: is this a good direction to solve penetration? probably not
		return vec;
	}
	else {
		return Vector3f::Zero();
	}
}

}