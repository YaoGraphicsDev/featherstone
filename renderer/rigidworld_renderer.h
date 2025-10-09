#pragma once

#include <vector>
#include <map>
#include <functional>

#include "primitives.h"
#include "raylib.h"
#include "glm/gtc/quaternion.hpp"

class RigidWorldRenderer {
public:
	struct Config {
		int screen_width = 1440;
		int screen_height = 900;
		Camera3D cam = {
			{ 10.0f, 10.0f, 10.0f }, // position
			{ 0.0f, 0.0f, 0.0f }, // target
			{ 0.0f, 1.0f, 0.0f }, // up
			{ 45.0f }, // fov
			CAMERA_PERSPECTIVE, // projection
		};
		glm::vec3 light_dir = { 0.35f, -1.0f, -0.35f };
		int fps = 60;
		// An aabb that tightly bounds the entire physical world. For the purpose of computing light space projection
		AABB world_aabb = {
			{-15.0f, -15.0f, -15.0f },
			{ 15.0f, 15.0f, 15.0f } 
		};
	};

	RigidWorldRenderer(Config config);

	~RigidWorldRenderer();

	enum class Shape {
		Cuboid,
		Cylinder,
		Sphere,
		Cone
	};
	// return handle
	size_t add_body(Shape shape, glm::vec3 half_dims, glm::quat rot = glm::identity<glm::quat>(), glm::vec3 trans = glm::vec3(0.0f));

	void update_body(size_t key, glm::quat rotation, glm::vec3 translation);

	void draw_bases(glm::vec3 origin, glm::mat3 bases, float length);

	struct Options {
		bool show_coordinate_gizmo = true;
		bool show_axis = true;
		bool show_light_config = false;
		bool movable_light = false;
	};
	// callback function passes render frame dt as parameter. DO NOT use it as time interval for physics world
	void run(
		std::function<void(float frame_dt, size_t frame_id)> update_world_cb,
		std::function<void(float frame_dt, size_t frame_id)> draw_3d_aid_cb = nullptr,
		Options opts = Options());

private:
	
	void draw_scene();

	void build_models();

	void draw_wireframe_aabb(AABB aabb, Color color);

	void draw_wireframe_obb(OBB obb, Color color);

	void draw_arrow_3d(Vector3 start, Vector3 end, Color color, float girth, float head_ratio = 0.33f);

	void draw_sphere(Vector3 center, float radius, Color color);

	glm::vec3 rotate_light(glm::vec3 light_dir, float dt, float radps = 0.6f);

	void update_light(AABB world_aabb, glm::vec3 light_dir);

	void draw_coordinate_gizmo();

	void UpdateCameraFreeRoam(Camera3D* camera, float moveSpeed, float mouseSensitivity);

	std::map<Shape, Model> _phong_models; // material type: phong lighitng model + shadow
	std::map<Shape, Model> _default_models; // material type: mono color default material

	const std::vector<Color> _palette = { RED, GREEN, BLUE, YELLOW, MAGENTA };

	struct Body {
		Shape shape;
		glm::quat rotation;
		glm::vec3 translation;
		glm::vec3 half_dims;
		Model* model;
		Color color;
	};
	std::vector<Body> _bodies;

	Camera3D _camera;
	Camera3D _lightCam;
	Shader _shader;
	RenderTexture2D _shadowmap;

	int _lightVPLoc;
	int _lightDirLoc;
	int _shadowmapLoc;
	int _frame_id;
	
	AABB _world_aabb;
	OBB _light_obb;
};