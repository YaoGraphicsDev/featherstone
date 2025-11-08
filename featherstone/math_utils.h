#pragma once

#include "Eigen/Core"
#include "Eigen/Dense"

#include "LinearMath/btVector3.h"
#include "LinearMath/btQuaternion.h"
#include "LinearMath/btTransform.h"

namespace SPD {

static inline Eigen::Vector3f EV3(btVector3 btv3) {
	return Eigen::Vector3f(btv3.x(), btv3.y(), btv3.z());
}

static inline btVector3 btv3(Eigen::Vector3f EV3) {
	return btVector3(EV3.x(), EV3.y(), EV3.z());
}

static inline btQuaternion btquat(Eigen::Quaternionf EQuat) {
	return btQuaternion(EQuat.x(), EQuat.y(), EQuat.z(), EQuat.w());
}

static inline btTransform bttrans(Eigen::Quaternionf EQuat, Eigen::Vector3f EV3) {
	return btTransform(btquat(EQuat), btv3(EV3));
}

static inline Eigen::Quaternionf EQuat(btQuaternion btquat) {
	return Eigen::Quaternionf(btquat.w(), btquat.x(), btquat.y(), btquat.z());
}

}