#include "rigidworld.h"
#include "math_utils.h"

#include "BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h"

#include <algorithm>
#include <iostream>

namespace SPD {

using namespace Eigen;

inline btCollisionShape* btbox(std::shared_ptr<Shape> shape) {
	const Cuboid* box = static_cast<Cuboid*>(shape.get());
	return new btBoxShape(btv3(box->half_dims));
}

inline btCollisionShape* btsphere(std::shared_ptr<Shape> shape) {
	const Sphere* sp = static_cast<Sphere*>(shape.get());
	return new btSphereShape(sp->radius);
}

inline btCollisionShape* btconvexhull(std::shared_ptr<Shape> shape) {
	const ConvexHull* ch = static_cast<ConvexHull*>(shape.get());
	btConvexHullShape* btshape = new btConvexHullShape();
	for (const Vector3f& p : ch->positions) {
		btshape->addPoint(btv3(p));
	}
	btshape->setMargin(0.0f);
	return btshape;
}

inline btCollisionShape* btcompound(std::shared_ptr<Shape> shape) {
	const CompoundShape* cs = static_cast<CompoundShape*>(shape.get());
	btCompoundShape* btshape = new btCompoundShape(true, cs->compositions.size());
	for (const CompoundShape::Composition& comp : cs->compositions) {
		if (comp.shape->type == Shape::Type::Cuboid) {
			btshape->addChildShape(bttrans(comp.rotation, comp.translation), btbox(comp.shape));
		}
		else if (comp.shape->type == Shape::Type::ConvexHull) {
			btshape->addChildShape(bttrans(comp.rotation, comp.translation), btconvexhull(comp.shape));
		}
		else if (comp.shape->type == Shape::Type::Sphere) {
			btshape->addChildShape(bttrans(comp.rotation, comp.translation), btsphere(comp.shape));
		}
		else {
			assert(false);
		}
	}
	return btshape;
}

std::shared_ptr<RigidWorld::Collider> RigidWorld::Collider::create(const RigidBody& rigidbody, int user_id) {
	btCollisionShape* shape = nullptr;
	if (rigidbody.shape->type == Shape::Type::Cuboid) shape = btbox(rigidbody.shape);
	else if (rigidbody.shape->type == Shape::Type::Sphere) shape = btsphere(rigidbody.shape);
	else if (rigidbody.shape->type == Shape::Type::ConvexHull) shape = btconvexhull(rigidbody.shape);
	else if (rigidbody.shape->type == Shape::Type::Compound) shape = btcompound(rigidbody.shape);
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
	if (!body || !body->shape || body->shape->type == Shape::Type::Default) {
		return;
	}

	bodies.push_back(body);
	std::shared_ptr<Collider> c = Collider::create(*body, bodies.size() - 1);
	colliders.push_back(c);
	collision_world->world->addCollisionObject(c->obj.get());
}

static int step_count = 0;

void RigidWorld::step(float dt) {
	collide();
	integrate_velocity(dt);

	contact_solver->initialize(bodies, collision_world->dispatcher);
	contact_solver->warm_start();
	for (uint32_t i = 0; i < max_velocity_solve_iterations; ++i) {
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
		FVector fe_com = b->fe + g6 * b->mass;
		b->fe = FVector::Zero();

		MTransform Xr = m_transform(Matrix3f::Identity(), b->rotation.toRotationMatrix(), Vector3f::Zero()); // a rotation to accomodate inertia's rotation
		Dyad I = transform_dyad2(Xr, b->I);
		FVector bias = dual_transform(derivative_cross(b->v)) * I * b->v;
		MVector a = transform_inv_dyad2(Xr, b->inv_I) * (fe_com - bias);
		MVector v_ortho = b->v;
		MVector a_ortho = a;

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
		
		// TODO:
		// follows this answer https://math.stackexchange.com/a/5035902
		//Vector3f v_ang = b->v.head<3>();
		//Quaternionf q = b->rotation;
		//Quaternionf dq = q * Quaternionf(0.0f, v_ang.x(), v_ang.y(), v_ang.z());
		//dq.coeffs() *= 0.5f;
		//Quaternionf dqdt;
		//dqdt.coeffs() = dq.coeffs() * dt;
		//q.coeffs() = q.coeffs() + dqdt.coeffs();
		//b->rotation = q;
		//b->rotation.normalize();

		// calculation given by deepseek. Works great, but doesnt align with the stackoverflow answer
		//Vector3f v_ang = b->v.head<3>();
		//Quaternionf omega_q(0.0f, v_ang.x(), v_ang.y(), v_ang.z());
		//Quaternionf dq = (omega_q * b->rotation);
		//dq.coeffs() *= 0.5f * dt;
		//b->rotation.coeffs() += dq.coeffs();
		//b->rotation.normalize();

		// Not accurate but the the easiest to understand
		Vector3f v_ang = b->v.head<3>();
		float angle = v_ang.norm() * dt;
		if (angle > 1e-8f) {
			Vector3f axis = v_ang.normalized();
			Quaternionf delta_q(AngleAxisf(angle, axis));
			b->rotation = delta_q * b->rotation;
		}
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

}