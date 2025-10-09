#pragma once

#include "glm/glm.hpp"

struct AABB {
    glm::vec3 min;
    glm::vec3 max;
};

struct OBB {
    glm::vec3 center;
    glm::vec3 half_dims;
    glm::mat3 bases;
};