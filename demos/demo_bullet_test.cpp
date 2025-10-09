#include "btBulletCollisionCommon.h"
#include <iostream>

int main() {
    // Collision configuration and dispatcher
    btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
    btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);

    // Broadphase
    btDbvtBroadphase* broadphase = new btDbvtBroadphase();

    // Create collision world
    btCollisionWorld* collisionWorld = new btCollisionWorld(dispatcher, broadphase, collisionConfiguration);

    // Create two box collision shapes
    btBoxShape* boxShape1 = new btBoxShape(btVector3(1, 1, 1));
    btBoxShape* boxShape2 = new btBoxShape(btVector3(1, 1, 1));

    // Create collision objects
    btCollisionObject* obj1 = new btCollisionObject();
    btCollisionObject* obj2 = new btCollisionObject();

    obj1->setCollisionShape(boxShape1);
    obj2->setCollisionShape(boxShape2);

    // Position objects to collide
    btTransform transform1, transform2;
    transform1.setIdentity();
    transform2.setIdentity();

    transform1.setOrigin(btVector3(0, 0, 0));
    transform2.setOrigin(btVector3(1.5, 0, 0)); // Partially overlapping

    obj1->setWorldTransform(transform1);
    obj2->setWorldTransform(transform2);

    // Add objects to world
    collisionWorld->addCollisionObject(obj1);
    collisionWorld->addCollisionObject(obj2);

    // Perform collision detection
    collisionWorld->performDiscreteCollisionDetection();

    // Get collision manifolds
    int numManifolds = collisionWorld->getDispatcher()->getNumManifolds();
    std::cout << "Number of manifolds: " << numManifolds << std::endl;

    for (int i = 0; i < numManifolds; i++) {
        btPersistentManifold* manifold = collisionWorld->getDispatcher()->getManifoldByIndexInternal(i);

        if (manifold->getNumContacts() > 0) {
            const btCollisionObject* objA = static_cast<const btCollisionObject*>(manifold->getBody0());
            const btCollisionObject* objB = static_cast<const btCollisionObject*>(manifold->getBody1());

            std::cout << "Collision detected! Number of contact points: " << manifold->getNumContacts() << std::endl;

            // Print contact point details
            for (int j = 0; j < manifold->getNumContacts(); j++) {
                btManifoldPoint& pt = manifold->getContactPoint(j);
                std::cout << "  Contact " << j << ":" << std::endl;
                std::cout << "    Position A: " << pt.getPositionWorldOnA().x() << ", "
                    << pt.getPositionWorldOnA().y() << ", "
                    << pt.getPositionWorldOnA().z() << std::endl;
                std::cout << "    Position B: " << pt.getPositionWorldOnB().x() << ", "
                    << pt.getPositionWorldOnB().y() << ", "
                    << pt.getPositionWorldOnB().z() << std::endl;
                std::cout << "    Normal: " << pt.m_normalWorldOnB.x() << ", "
                    << pt.m_normalWorldOnB.y() << ", "
                    << pt.m_normalWorldOnB.z() << std::endl;
                std::cout << "    Distance: " << pt.getDistance() << std::endl;
                std::cout << "    Depth: " << pt.getDistance() << std::endl;
            }
        }
    }

    // Cleanup
    delete collisionWorld;
    delete broadphase;
    delete dispatcher;
    delete collisionConfiguration;
    delete boxShape1;
    delete boxShape2;
    delete obj1;
    delete obj2;

    return 0;
}