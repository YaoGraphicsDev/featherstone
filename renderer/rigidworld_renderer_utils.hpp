#pragma once
#include "glm/glm.hpp"
#include "raylib.h"
#include "primitives.h"

// Fit AABB into OBB space
static OBB FitAABBToOBB(AABB aabb, glm::mat3 bases) {
    glm::mat3 inv_bases = glm::transpose(bases);

    glm::vec3 aabb_verts[8] = {
        {aabb.min.x, aabb.min.y, aabb.min.z},
        {aabb.max.x, aabb.min.y, aabb.min.z},
        {aabb.min.x, aabb.max.y, aabb.min.z},
        {aabb.max.x, aabb.max.y, aabb.min.z},
        {aabb.min.x, aabb.min.y, aabb.max.z},
        {aabb.max.x, aabb.min.y, aabb.max.z},
        {aabb.min.x, aabb.max.y, aabb.max.z},
        {aabb.max.x, aabb.max.y, aabb.max.z}
    };

    // Transform vertices to OBB space
    glm::vec3 obb_min = { FLT_MAX, FLT_MAX, FLT_MAX };
    glm::vec3 obb_max = { -FLT_MAX, -FLT_MAX, -FLT_MAX };

    for (int i = 0; i < 8; i++) {
        glm::vec3 transformed_vert = inv_bases * aabb_verts[i];

        // in obb space
        obb_min = glm::min(transformed_vert, obb_min);
        obb_max = glm::max(transformed_vert, obb_max);
    }

    return { bases * ((obb_min + obb_max) * 0.5f), (obb_max - obb_min) * 0.5f, bases };
}

// fit the entirety of world aabb inside an obb. Determine the width of light's orthogonal projection
static OBB DirectionalLightOBB(AABB world_aabb, glm::vec3 light_dir) {
    glm::vec3 up(0.0f, 1.0f, 0.0f);
    glm::vec3 front = glm::normalize(light_dir);
    glm::vec3 right = glm::cross(front, up);
    if (glm::length(right) <= 10E-5) {
        right = glm::vec3(1.0f, 0.0f, 0.0f);
    }
    right = glm::normalize(right);
    up = glm::normalize(glm::cross(right, front));
    glm::mat3 bases(up, right, front);
    return FitAABBToOBB(world_aabb, bases);
}

static inline Vector3 V3(glm::vec3 v) {
    return { v.x, v.y, v.z };
}

static inline glm::vec3 v3(Vector3 v) {
    return { v.x, v.y, v.z };
}

