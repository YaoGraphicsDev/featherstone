#pragma once

#include "gltf_scene.h"

#include <string>

bool load_gltf(
	const std::string& filename,
	SceneGraph& graph,
	SceneGraphFlatRefs& graph_refs);
