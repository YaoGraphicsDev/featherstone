#include "rigidworld.h"
#include "rigidworld_renderer.h"

#include <iostream>

using namespace SPD;
using namespace Eigen;

glm::quat q(Quaternionf q) {
    return glm::quat(q.w(), q.x(), q.y(), q.z());
}

glm::vec3 v3(Vector3f v) {
    return glm::vec3(v.x(), v.y(), v.z());
}

glm::mat3 m3(Matrix3f M) {
    glm::mat3 m;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            m[j][i] = M(i, j);
        }
    }
    return m;
}

static FVector force_on_com(Vector3f f, Vector3f fp, Vector3f com, Matrix3f com_bases) {
    Vector3f torque = f.cross(com - fp);
    torque = com_bases.inverse() * torque;
    Vector3f force = com_bases.inverse() * f;

    FTransform t = dual_transform(m_transform(Matrix3f::Identity(), com_bases, com - fp));
    FVector f6;
    f6 << Vector3f::Zero(), f;
    f6 = t * f6;

    std::cout << "torque = " << torque.transpose() << std::endl;
    std::cout << "force = " << force.transpose() << std::endl;
    std::cout << f6.transpose() << std::endl;
    
    return f6;
}

int main() {
    std::shared_ptr<RigidWorld> world = std::make_shared<RigidWorld>();
    std::shared_ptr<RigidBody> box;
    std::shared_ptr<RigidBody> ground;
    {
        RigidBody::Config config;
        config.shape.reset(new Cuboid(Vector3f(1.0f, 0.5f, 0.7f)));
        float radian = 45.0f / 180.0f * 3.1416f;
        config.rotation = Quaternionf(AngleAxisf(radian, Vector3f(1.0f, 1.0f, 1.0f).normalized()));
        config.translation = Vector3f(2.0f, 8.0f, 3.0f);
        box.reset(new RigidBody(config));
        world->add_body(box);
    }
    {
        RigidBody::Config config;
        config.shape.reset(new Cuboid(Vector3f(15.0f, 1.0f, 15.0f)));
        config.translation = Vector3f(0.0f, -1.0f, 0.0f);
        config.type = RigidBody::DynamicType::Static;
        ground.reset(new RigidBody(config));
        world->add_body(ground);
    }

    RigidWorldRenderer::Config renderer_config;
    renderer_config.world_aabb = { glm::vec3(-10.0f, -1.0f, -10.0f), glm::vec3(10.0f, 15.0f, 10.0f) };
    RigidWorldRenderer renderer(renderer_config);

    size_t k1 = renderer.add_body(RigidWorldRenderer::Shape::Cuboid, v3(std::dynamic_pointer_cast<SPD::Cuboid>(box->shape)->half_dims));
    size_t k2 = renderer.add_body(RigidWorldRenderer::Shape::Cuboid, v3(std::dynamic_pointer_cast<SPD::Cuboid>(ground->shape)->half_dims));

    float total_time = 0.0f;
    bool force_applied = false;
    auto update_world = [&](float frame_dt, size_t frame_id) {
        //total_time += frame_dt;
        //if (total_time > 1.0f && !force_applied) {
        //    box->fe_com = force_on_com(Vector3f(100.0f, 0.0f, 50.0f), Vector3f(2.0f, 0.0f, 0.0f), box->translation, Matrix3f::Identity());
        //    force_applied = true;
        //}

        world->step(0.01667f);

        renderer.update_body(k1, q(box->rotation), v3(box->translation));
        renderer.update_body(k2, q(ground->rotation), v3(ground->translation));
    };

    renderer.run(update_world, nullptr);

    return 0;
}