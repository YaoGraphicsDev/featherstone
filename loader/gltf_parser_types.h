#pragma once

#include "Eigen/Core"

struct PVec2f {
    float x;
    float y;
};
static_assert(sizeof(PVec2f) == 2 * sizeof(float), "Padding detected!");

inline Eigen::Vector2f EV2(const PVec2f& pvec2) {
    return Eigen::Vector2f(pvec2.x, pvec2.y);
}

struct PVec3f {
    float x;
    float y;
    float z;
};
static_assert(sizeof(PVec3f) == 3 * sizeof(float), "Padding detected!");

inline Eigen::Vector3f EV3(const PVec3f& pvec3) {
    return Eigen::Vector3f(pvec3.x, pvec3.y, pvec3.z);
}
inline PVec3f PV3(const Eigen::Vector3f& evec3) {
    return { evec3.x(), evec3.y(), evec3.z() };
}

struct PVec4f {
    float x;
    float y;
    float z;
    float w;
};
static_assert(sizeof(PVec4f) == 4 * sizeof(float), "Padding detected!");

inline Eigen::Vector4f EV4(const PVec4f& pvec4) {
    return Eigen::Vector4f(pvec4.x, pvec4.y, pvec4.z, pvec4.w);
}