#pragma once

#include "gltf_parser_types.h"

template<typename T>
struct GltfElementTraits;

template<>
struct GltfElementTraits<PVec2f> {
	static constexpr int n_components = 2;
	static constexpr int gltf_type = TINYGLTF_TYPE_VEC2;
	static constexpr int gltf_component_type = TINYGLTF_COMPONENT_TYPE_FLOAT;
};

template<>
struct GltfElementTraits<PVec3f> {
	static constexpr int n_components = 3;
	static constexpr int gltf_type = TINYGLTF_TYPE_VEC3;
	static constexpr int gltf_component_type = TINYGLTF_COMPONENT_TYPE_FLOAT;
};

template<>
struct GltfElementTraits<PVec4f> {
	static constexpr int n_components = 4;
	static constexpr int gltf_type = TINYGLTF_TYPE_VEC4;
	static constexpr int gltf_component_type = TINYGLTF_COMPONENT_TYPE_FLOAT;
};

template<>
struct GltfElementTraits<uint32_t> {
	static constexpr int n_components = 1;
	static constexpr int gltf_type = TINYGLTF_TYPE_SCALAR;
	static constexpr int gltf_component_type = TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT;
};

template<>
struct GltfElementTraits<uint16_t> {
	static constexpr int n_components = 1;
	static constexpr int gltf_type = TINYGLTF_TYPE_SCALAR;
	static constexpr int gltf_component_type = TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT;
};

template<typename T>
struct TypeReflect;

template<>
struct TypeReflect<PVec2f> {
	static constexpr const char* name = "PVec2f";
};

template<>
struct TypeReflect<PVec3f> {
	static constexpr const char* name = "PVec3f";
};

template<>
struct TypeReflect<PVec4f> {
	static constexpr const char* name = "PVec4f";
};

template<>
struct TypeReflect<uint32_t> {
	static constexpr const char* name = "uint32_t";
};

template<>
struct TypeReflect<uint16_t> {
	static constexpr const char* name = "uint16_t";
};