#include "btBulletCollisionCommon.h"
#include <iostream>

int main() {
    btCollisionConfiguration* config = new btDefaultCollisionConfiguration();
    btCollisionDispatcher* dispatcher = new btCollisionDispatcher(config);
    btBroadphaseInterface* broadphase = new btDbvtBroadphase();
    btCollisionWorld* world = new btCollisionWorld(dispatcher, broadphase, config);

    btCollisionObject* box = new btCollisionObject();
    box->setCollisionShape(new btBoxShape(btVector3(1.0f, 1.0f, 1.0f)));
    box->setWorldTransform(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 5, 0)));
    world->addCollisionObject(box);

    btCollisionObject* box = new btCollisionObject();
    box->setCollisionShape(new btBoxShape(btVector3(5.0f, 1.5f, 5.0f)));
    world->addCollisionObject(box);

    for (int frame = 0; frame < 100; ++frame)
    {
        // Move sphere manually
        btTransform t = sphere->getWorldTransform();
        t.setOrigin(t.getOrigin() + btVector3(0, -0.1, 0));
        std::cout << "sphere center at " << t.getOrigin().y() << ", bottom at " << t.getOrigin().y() - 1.0f  << ", top at " << t.getOrigin().y() + 1.0f << std::endl;
        sphere->setWorldTransform(t);
        world->updateSingleAabb(sphere);

        world->performDiscreteCollisionDetection();

        int numManifolds = dispatcher->getNumManifolds();
        if (numManifolds == 0) {
            std::cout << "no contact frame " << frame << std::endl;
        }
        else {
            std::cout << "contact" << std::endl;
        }
        for (int i = 0; i < numManifolds; i++) {
            btPersistentManifold* manifold = dispatcher->getManifoldByIndexInternal(i);
            const btCollisionObject* A = manifold->getBody0();
            const btCollisionObject* B = manifold->getBody1();

            for (int i = 0; i < manifold->getNumContacts(); ++i) {
                const btManifoldPoint& cp = manifold->getContactPoint(i);
                float penetration = -cp.getDistance();
                btVector3 pa = cp.m_positionWorldOnA;
                btVector3 pb = cp.m_positionWorldOnB;
                btVector3 n = cp.m_normalWorldOnB;

                std::cout << "\t" << "contact point " << i << ", penetration = " << cp.getDistance()
                    << ", pA = " << cp.m_positionWorldOnA.x() << ", " << cp.m_positionWorldOnA.y() << ", " << cp.m_positionWorldOnA.z()
                    << ", pB = " << cp.m_positionWorldOnB.x() << ", " << cp.m_positionWorldOnB.y() << ", " << cp.m_positionWorldOnB.z()
                    << ", nB = " << cp.m_normalWorldOnB.x() << ", " << cp.m_normalWorldOnB.y() << ", " << cp.m_normalWorldOnB.z()
                    << std::endl;
            }
            //if (manifold->getNumContacts() > 0) {
            //    const btManifoldPoint& cp = manifold->getContactPoint(0);
            //    printf("Penetration: %f\n", cp.getDistance());
            //}
        }
    }
}