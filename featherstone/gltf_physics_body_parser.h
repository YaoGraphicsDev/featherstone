#pragma once

#include "gltf_physics_types.h"

bool load_gltf_physics_world(
	const std::string& filename,
	std::vector<RigidBodyDescriptor>& rigids,
	std::vector<ArticulatedBodyDescriptor>& articulates);