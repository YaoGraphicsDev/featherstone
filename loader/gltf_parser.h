#pragma once

#include "gltf_scene.h"

#include <string>

enum GLTFParseOption {
	Default = 0,
	BlenderExport,
};
bool load_gltf(const std::string& filename, Scene& scene, GLTFParseOption opt);
