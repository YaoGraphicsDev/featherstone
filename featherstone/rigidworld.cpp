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
	obj->setUserIndex2(-1); // articulated body indicator. -1 indicated it is not an art body

	std::shared_ptr<Collider> collider = std::make_shared<Collider>();
	collider->shape.reset(shape);
	collider->obj.reset(obj);
	return collider;
}

std::vector<std::shared_ptr<RigidWorld::Collider>> RigidWorld::Collider::create(const ArticulatedBody& artbody, int user_id) {
	std::vector<std::shared_ptr<RigidWorld::Collider>> art_colliders;
	for (int i = 0; i < artbody.bodies.size(); ++i) {
		auto& body = artbody.bodies[i];
		btCollisionShape* shape = nullptr;
		assert(body);
		if (!body->shape) {
			continue;
		}
		if (body->shape->type == Shape::Type::Cuboid) shape = btbox(body->shape);
		else if (body->shape->type == Shape::Type::Sphere) shape = btsphere(body->shape);
		else if (body->shape->type == Shape::Type::ConvexHull) shape = btconvexhull(body->shape);
		else if (body->shape->type == Shape::Type::Compound) shape = btcompound(body->shape);
		else {
			assert(false);
		}

		btCollisionObject* obj = new btCollisionObject();
		obj->setCollisionShape(shape);
		btTransform transform(
			btquat(body->rotation),
			btv3(body->translation));
		obj->setWorldTransform(transform);
		obj->setUserIndex(user_id);
		obj->setUserIndex2(i);

		std::shared_ptr<Collider> collider = std::make_shared<Collider>();
		collider->shape.reset(shape);
		collider->obj.reset(obj);
		art_colliders.push_back(collider);
	}
	return art_colliders;
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
	for (auto c : rigid_colliders) {
		collision_world->world->removeCollisionObject(c->obj.get());
	}
	for (auto& c : art_colliders) {
		for (auto cc : c) {
			collision_world->world->removeCollisionObject(cc->obj.get());
		}
	}
	gContactDestroyedCallback = nullptr;
	collision_world->world.reset();
	collision_world->broadphase.reset();
	collision_world->dispatcher.reset();
	collision_world->config.reset();
}

void RigidWorld::add_body(std::shared_ptr<RigidBody> rigidbody) {
	if (!rigidbody || !rigidbody->shape || rigidbody->shape->type == Shape::Type::Default) {
		return;
	}

	rigidbodies.push_back(rigidbody);
	std::shared_ptr<Collider> c = Collider::create(*rigidbody, rigidbodies.size() - 1);
	rigid_colliders.push_back(c);
	collision_world->world->addCollisionObject(c->obj.get());
}

void RigidWorld::add_body(std::shared_ptr<ArticulatedBody> artbody) {
	if (artbody->bodies.empty()) {
		assert(false);
		return;
	}

	artbodies.push_back(artbody);
	artbody->set_gravity(this->gravity);
	std::vector<std::shared_ptr<Collider>> cs = std::move(Collider::create(*artbody, artbodies.size() - 1));
	art_colliders.push_back(cs);
	for (std::shared_ptr<Collider> c : cs) {
		collision_world->world->addCollisionObject(c->obj.get());
	}
}

static int step_count = 0;

void RigidWorld::step(float dt) {
	collide();
	integrate_velocity(dt);

	contact_solver->initialize(rigidbodies, artbodies, collision_world->dispatcher);
	contact_solver->warm_start();
	for (uint32_t i = 0; i < max_velocity_solve_iterations; ++i) {
		contact_solver->solve_velocity();
	}
	contact_solver->project_velocity();

	integrate_position(dt);

	for (uint32_t i = 0; i < max_position_solve_iterations; ++i) {
		contact_solver->solve_position();
	}
}

void RigidWorld::collide() {
	for (int i = 0; i < rigidbodies.size(); ++i) {
		RigidBody& b = *rigidbodies[i];
		Collider& c = *rigid_colliders[i];
		if (b.type == RigidBody::DynamicType::Static) {
			continue;
		}
		c.update(b.translation, b.rotation);
		collision_world->world->updateSingleAabb(c.obj.get());
	}

	for (int i = 0; i < artbodies.size(); ++i) {
		ArticulatedBody& art_b = *artbodies[i];
		std::vector<std::shared_ptr<Collider>>& art_c = art_colliders[i];
		for (int j = 0; j < art_b.bodies.size(); ++j) {
			art_c[j]->update(art_b.bodies[j]->translation, art_b.bodies[j]->rotation);
			collision_world->world->updateSingleAabb(art_c[j]->obj.get());
		}
	}

	collision_world->world->performDiscreteCollisionDetection();
}

void RigidWorld::integrate_velocity(float dt) {
	for (auto b : rigidbodies) {
		if (b->type == RigidBody::DynamicType::Static) {
			continue;
		}

		FVector g6_com;
		g6_com << Vector3f::Zero(), gravity;
		FVector fe_o = dual_transform(m_transform(Matrix3f::Identity(), Matrix3f::Identity(), b->rotation * -b->shape->com)) * g6_com * b->mass;

		// Accomodates rotation
		MTransform Xr = m_transform(Matrix3f::Identity(), b->rotation.toRotationMatrix(), Vector3f::Zero());
		Dyad I = transform_dyad2(Xr, b->I);
		FVector bias = dual_transform(derivative_cross(b->v)) * I * b->v;
		MVector a = transform_inv_dyad2(Xr, b->inv_I) * (fe_o - bias);
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

	for (auto ab : artbodies) {
		ab->integrate_velocity(dt);
	}
}

void RigidWorld::integrate_position(float dt) {
	// TODO: for an accurate integration, see line 1103 of btDiscretePhysicsWorld.cpp predictIntegratedTransform
	for (auto b : rigidbodies) {
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

	// TODO integrate articulated body positions
	for (auto ab : artbodies) {
		ab->integrate_position(dt);
	}
}

// return a negative value if penetration happens
//float RigidWorld::new_penetration(
//	const RigidBody& b0, Vector3f local_p0,
//	const RigidBody& b1, Vector3f local_p1,
//	Vector3f n_01) {
//	Vector3f p0 = b0.rotation.toRotationMatrix() * local_p0 + b0.translation;
//	Vector3f p1 = b1.rotation.toRotationMatrix() * local_p1 + b1.translation;
//	float proj = n_01.dot((p0 - p1));
//	return -proj;
//}

}