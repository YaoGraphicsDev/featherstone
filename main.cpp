#include "spdynamics.hpp"
#include "rigidworld_renderer.h"

#include <iostream>

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

int main(void)
{
    float hx = 0.2f;
    float hy = 1.2f;
    float hz = 0.15f;
    std::shared_ptr<SPD::Cuboid> cube = std::make_shared<SPD::Cuboid>(Vector3f(hx, hy, hz));

    float gap = 0.1f;
    float height = 3.0f;

    SPD::ArticulatedBody art(nullptr, Quaternionf::Identity(), Vector3f(0.0f, height, 0.0f));
    art.set_gravity(Vector3f(0.0f, -10.0f, 0.0f));
    auto b1 = art.add_body(
        cube,
        Quaternionf::Identity(),
        { 0.0f, hy + height, 0.0f });
    auto b2 = art.add_body(
        cube,
        Quaternionf(AngleAxisf(glm::radians(90.0f), -Vector3f::UnitZ())),
        { hy, 2.0f * hy + height, -(2.0f * hz + gap) });
    auto b3 = art.add_body(
        cube,
        Quaternionf::Identity(),
        { 2.0f * hy, hy + height, -(4.0f * hz + 2.0f * gap) });
    auto j1 = art.add_constraint(SPD::ArticulatedBody::ConstraintType::Revolute,
        art.base(), b1,
        Matrix3f::Identity(),
        Vector3f(0.0f, -gap, 0.0f));
    auto j2 = art.add_constraint(SPD::ArticulatedBody::ConstraintType::Revolute,
        b1, b2,
        Matrix3f::Identity(),
        Vector3f(0.0f, hy, -(hz + 0.5f * gap)));
    auto j3 = art.add_constraint(SPD::ArticulatedBody::ConstraintType::Revolute,
        b2, b3,
        Matrix3f::Identity(),
        Vector3f(0.0f, hy, -(hz + 0.5f * gap)));

    // The joint that creates loop
    auto j4 = art.add_constraint(SPD::ArticulatedBody::ConstraintType::Prismatic,
        art.base(), b3,
        Quaternionf(AngleAxisf(glm::radians(90.0f), Vector3f::UnitY())).toRotationMatrix(),
        Vector3f(2.0f * hy, -gap, -(4.0f * hz + 2.0 * gap)));

    art.build_tree();
    
    // renderer
    RigidWorldRenderer::Config renderer_config;
    renderer_config.world_aabb = { glm::vec3(-10.0f, -5.0f, -10.0f), glm::vec3(10.0f, 15.0f, 10.0f) };
    RigidWorldRenderer renderer(renderer_config);

    size_t k1 = renderer.add_body(RigidWorldRenderer::Shape::Cuboid, v3(std::dynamic_pointer_cast<SPD::Cuboid>(b1->shape)->half_dims));
    size_t k2 = renderer.add_body(RigidWorldRenderer::Shape::Cuboid, v3(std::dynamic_pointer_cast<SPD::Cuboid>(b2->shape)->half_dims));
    size_t k3 = renderer.add_body(RigidWorldRenderer::Shape::Cuboid, v3(std::dynamic_pointer_cast<SPD::Cuboid>(b3->shape)->half_dims));
    size_t ground = renderer.add_body(RigidWorldRenderer::Shape::Cuboid, { 5.0f, 0.5f, 5.0f }, glm::identity<glm::quat>(), { 0.0f, -0.5f, 0.0f });

    auto update_world = [&](float frame_dt, size_t frame_id) {
        // art.set_constraint_status(j1, { glm::sin((float)frame_id / 120.0f * glm::two_pi<float>()) });
        // art.set_constraint_status(j2, { (float)frame_id / 30.0f });

        // TODO: RNEA should be tailored to accept zero joint acceleration
        //art.RNEA();
        //art.compute_H();

        //std::cout << "j1 tau = " << j1->tau << std::endl;
        //std::cout << "j2 tau = " << j2->tau << std::endl;
        //std::cout << (b2->bases * j2->bt1).x() * 10.0f * b2->shape->vol << std::endl;

        //art.move_bodies();

        j4->taue(0, 0) = 50.0f;

        art.step(1.0f / 120.0f);

        renderer.update_body(k1, q(b1->rotation), v3(b1->translation));
        renderer.update_body(k2, q(b2->rotation), v3(b2->translation));
        renderer.update_body(k3, q(b3->rotation), v3(b3->translation));
    };

    auto draw_joint = [&](std::shared_ptr<SPD::ArticulatedBody::Constraint> c) {
        renderer.draw_bases(v3(c->b0->translation + c->b0->bases * c->bt0), m3(c->b0->bases * c->bb0), 0.5f);
        renderer.draw_bases(v3(c->b1->translation + c->b1->bases * c->bt1), m3(c->b1->bases * c->bb1), 0.5f);
    };

    auto debug_draw = [&](float frame_dt, size_t frame_id) {
        // renderer.draw_bases(glm::vec3(0.0f), glm::mat3(1.0f), 1.0f);
        draw_joint(j1);
        draw_joint(j2);
        draw_joint(j3);
        draw_joint(j4);
    };

    //RigidWorldRenderer::Options run_options;
    //// run_options.show_light_config = true;
    //run_options.movable_light = true;
    renderer.run(update_world, debug_draw);

    return 0;
}


