#include <iostream>
#include <string>
#include <fstream>
#include "rigidworld_renderer_utils.hpp"
#include "raymath.h"
#include "rlgl.h"
#include "json.hpp" // nlohmann json

#include "rigidworld_renderer.h"

#define GLM_ENABLE_EXPERIMENTAL
#include "glm/gtx/quaternion.hpp"

#define PURE_RED        CLITERAL(Color){ 255, 0, 0, 255 }     // Red
#define PURE_GREEN        CLITERAL(Color){ 0, 255, 0, 255 }     // Red
#define PURE_BLUE        CLITERAL(Color){ 0, 0, 255, 255 }     // Red

const int shadowmap_resolution = 2048;

RenderTexture2D LoadShadowmapRenderTexture(int width, int height)
{
    RenderTexture2D target = { 0 };

    target.id = rlLoadFramebuffer(); // Load an empty framebuffer
    target.texture.width = width;
    target.texture.height = height;

    if (target.id > 0)
    {
        rlEnableFramebuffer(target.id);

        // Create depth texture
        // We don't need a color texture for the shadowmap
        target.depth.id = rlLoadTextureDepth(width, height, false);
        target.depth.width = width;
        target.depth.height = height;
        target.depth.format = 19;       //DEPTH_COMPONENT_24BIT?
        target.depth.mipmaps = 1;

        // Attach depth texture to FBO
        rlFramebufferAttach(target.id, target.depth.id, RL_ATTACHMENT_DEPTH, RL_ATTACHMENT_TEXTURE2D, 0);

        // Check if fbo is complete with attachments (valid)
        if (rlFramebufferComplete(target.id)) TRACELOG(LOG_INFO, "FBO: [ID %i] Framebuffer object created successfully", target.id);

        rlDisableFramebuffer();
    }
    else TRACELOG(LOG_WARNING, "FBO: Framebuffer object can not be created");

    return target;
}

void UnloadShadowmapRenderTexture(RenderTexture2D target)
{
    if (target.id > 0)
    {
        // NOTE: Depth texture/renderbuffer is automatically
        // queried and deleted before deleting framebuffer
        rlUnloadFramebuffer(target.id);
    }
}

RigidWorldRenderer::RigidWorldRenderer(Config config) {
    _config_json_name = config.app_name + "_renderer_config.json";

    Config saved_config;
    if (load_config_json(saved_config)) {
        // if saved config exists, overwrite config parameter
        config = saved_config;
    }

    _camera = config.cam;

    SetConfigFlags(FLAG_MSAA_4X_HINT);
    SetConfigFlags(FLAG_WINDOW_RESIZABLE);
    InitWindow(config.screen_width, config.screen_height, "Rigid World Renderer");

    build_shaders(config);
    build_textures();
    build_models();

    // shadowmap
    _shadowmap = LoadShadowmapRenderTexture(shadowmap_resolution, shadowmap_resolution);

    update_light(config.world_aabb, config.light_dir);

    SetTargetFPS(config.fps);
    _frame_id = 0;

    _world_aabb = config.world_aabb;
}

RigidWorldRenderer::~RigidWorldRenderer() {
    destroy_models();
    destroy_bodies();
    destroy_textures();
    destroy_shaders();
    UnloadShadowmapRenderTexture(_shadowmap);

    save_config_json();

    CloseWindow();
}

void RigidWorldRenderer::save_config_json() {
    // std::string filename = "./rigidworld_renderer_config.json";

    auto v3_to_json = [](float x, float y, float z) {
        return nlohmann::json::array({ x, y, z });
    };

    nlohmann::json j;

    // Window / perf
    j["screen_width"] = GetScreenWidth();
    j["screen_height"] = GetScreenHeight();

    // raylib does not expose target FPS; GetFPS() is "current measured FPS".
    // If you want the target FPS, store it as a member and serialize that instead.
    j["fps"] = 60.0f;

    // Camera
    j["cam"] = {
        {"position",   v3_to_json(_camera.position.x, _camera.position.y, _camera.position.z)},
        {"target",     v3_to_json(_camera.target.x,   _camera.target.y,   _camera.target.z)},
        {"up",         v3_to_json(_camera.up.x,       _camera.up.y,       _camera.up.z)},
        {"fovy",       _camera.fovy},
        {"projection", _camera.projection}
    };

    // Light direction: your runtime light dir lives in _light_obb.bases[2]
    {
        glm::vec3 light_dir = _light_obb.bases[2];
        j["light_dir"] = v3_to_json(light_dir.x, light_dir.y, light_dir.z);
    }

    // World AABB
    j["world_aabb"] = {
        {"min", v3_to_json(_world_aabb.min.x, _world_aabb.min.y, _world_aabb.min.z)},
        {"max", v3_to_json(_world_aabb.max.x, _world_aabb.max.y, _world_aabb.max.z)}
    };

    std::ofstream os(_config_json_name, std::ios::out | std::ios::trunc);
    if (!os.is_open()) {
        std::cout << "renderer failed to open " << _config_json_name << " for writing" << std::endl;
        return;
    }
    os << j.dump(4) << std::endl;
}

bool RigidWorldRenderer::load_config_json(Config& cfg) {
    // std::string filename = "./rigidworld_renderer_config.json";

    // Helpers
    auto read_vec3_array = [](const nlohmann::json& a, float& x, float& y, float& z) -> bool {
        if (!a.is_array() || a.size() != 3) return false;
        if (!a[0].is_number() || !a[1].is_number() || !a[2].is_number()) return false;
        x = a[0].get<float>();
        y = a[1].get<float>();
        z = a[2].get<float>();
        return true;
    };

    std::ifstream is(_config_json_name);
    if (!is.is_open()) {
        return false; // No saved config is not an error
    }

    nlohmann::json j;
    is >> j;
    if (!j.is_object())
        return false;

    // Screen
    if (j.contains("screen_width") && j["screen_width"].is_number_integer())
        cfg.screen_width = j["screen_width"].get<int>();
    if (j.contains("screen_height") && j["screen_height"].is_number_integer())
        cfg.screen_height = j["screen_height"].get<int>();

    // FPS
    if (j.contains("fps") && j["fps"].is_number_integer())
        cfg.fps = j["fps"].get<int>();

    // Camera
    if (j.contains("cam") && j["cam"].is_object()) {
        const auto& jc = j["cam"];

        if (jc.contains("position")) {
            float x, y, z;
            if (read_vec3_array(jc["position"], x, y, z)) cfg.cam.position = { x, y, z };
        }
        if (jc.contains("target")) {
            float x, y, z;
            if (read_vec3_array(jc["target"], x, y, z)) cfg.cam.target = { x, y, z };
        }
        if (jc.contains("up")) {
            float x, y, z;
            if (read_vec3_array(jc["up"], x, y, z)) cfg.cam.up = { x, y, z };
        }
        if (jc.contains("fovy") && jc["fovy"].is_number())
            cfg.cam.fovy = jc["fovy"].get<float>();
        if (jc.contains("projection") && jc["projection"].is_number_integer())
            cfg.cam.projection = jc["projection"].get<int>();
    }

    // Light dir
    if (j.contains("light_dir")) {
        float x, y, z;
        if (read_vec3_array(j["light_dir"], x, y, z)) cfg.light_dir = { x, y, z };
    }

    // World AABB
    if (j.contains("world_aabb") && j["world_aabb"].is_object()) {
        const auto& wa = j["world_aabb"];
        if (wa.contains("min")) {
            float x, y, z;
            if (read_vec3_array(wa["min"], x, y, z)) cfg.world_aabb.min = { x, y, z };
        }
        if (wa.contains("max")) {
            float x, y, z;
            if (read_vec3_array(wa["max"], x, y, z)) cfg.world_aabb.max = { x, y, z };
        }
    }

    return true;
}


static Mesh move_and_reupload_mesh(Mesh old_mesh, glm::vec3 translation, glm::quat rotation) {
    Mesh new_mesh = { 0 };
    new_mesh.vertexCount = old_mesh.vertexCount;
    new_mesh.triangleCount = old_mesh.triangleCount;

    {
        new_mesh.vertices = (float*)RL_MALLOC(new_mesh.vertexCount * 3 * sizeof(float));
        memcpy(new_mesh.vertices, old_mesh.vertices, new_mesh.vertexCount * 3 * sizeof(float));
        glm::vec3* v3p = reinterpret_cast<glm::vec3*>(new_mesh.vertices);
        for (int i = 0; i < new_mesh.vertexCount; ++i) {
            *v3p = rotation * *v3p + translation;
            ++v3p;
        }
    }

    if (old_mesh.texcoords) {
        new_mesh.texcoords = (float*)RL_MALLOC(new_mesh.vertexCount * 2 * sizeof(float));
        memcpy(new_mesh.texcoords, old_mesh.texcoords, new_mesh.vertexCount * 2 * sizeof(float));
    }

    if (old_mesh.normals) {
        new_mesh.normals = (float*)RL_MALLOC(new_mesh.vertexCount * 3 * sizeof(float));
        memcpy(new_mesh.normals, old_mesh.normals, new_mesh.vertexCount * 3 * sizeof(float));
        glm::vec3* v3p = reinterpret_cast<glm::vec3*>(new_mesh.normals);
        for (int i = 0; i < new_mesh.vertexCount; ++i) {
            *v3p = rotation * *v3p;
            ++v3p;
        }
    }

    if (old_mesh.indices) {
        // assuming this is a triangle mesh
        int index_count = new_mesh.triangleCount * 3;
        new_mesh.indices = (uint16_t*)RL_MALLOC(index_count * sizeof(uint16_t));
        memcpy(new_mesh.indices, old_mesh.indices, index_count * sizeof(uint16_t));
    }

    UnloadMesh(old_mesh);
    UploadMesh(&new_mesh, false);
    return new_mesh;
}

Mesh RigidWorldRenderer::build_mesh(
    Shape shape,
    glm::vec3 half_dim,
    glm::vec3 trans,
    glm::quat rot) {
    if (shape == Shape::Cuboid) {
        Mesh mesh = GenMeshCube(half_dim.x * 2.0f, half_dim.y * 2.0f, half_dim.z * 2.0f);
        Mesh new_mesh = move_and_reupload_mesh(mesh, trans, rot);
        return new_mesh;
    }
    else if (shape == Shape::Cylinder) {
        Mesh mesh = GenMeshCylinder(half_dim.x, half_dim.y * 2.0f, 6); // Generated cylinder sits on y=0 plane. Move it downwars so that half of it goes under y=0
        Mesh new_mesh = move_and_reupload_mesh(mesh, rot * glm::vec3(0.0f, -half_dim.y, 0.0f) + trans, rot);
        return new_mesh;
    }
    else if (shape == Shape::Sphere) {
        Mesh mesh = GenMeshSphere(half_dim.x * 1.01f, 8, 8);
        Mesh new_mesh = move_and_reupload_mesh(mesh, trans, rot);
        return new_mesh;
    }
    else {
        assert(false);
        return Mesh();
    }
}

Mesh RigidWorldRenderer::build_mesh(
    const std::vector<glm::vec3>& positions,
    const std::vector<glm::vec3>& normals,
    const std::vector<glm::vec2>& uvs,
    const std::vector<uint16_t>& indices) {

    int vertex_count = positions.size();
    int index_count = indices.size();
    Mesh mesh = { 0 };
    mesh.vertexCount = vertex_count;
    mesh.triangleCount = index_count / 3;

    mesh.vertices = (float*)RL_MALLOC(vertex_count * 3 * sizeof(float));
    memcpy(mesh.vertices, positions.data(), vertex_count * 3 * sizeof(float));

    if (!uvs.empty()) {
        assert(uvs.size() == vertex_count);
        mesh.texcoords = (float*)RL_MALLOC(vertex_count * 2 * sizeof(float));
        memcpy(mesh.texcoords, uvs.data(), vertex_count * 2 * sizeof(float));
    }

    if (!normals.empty()) {
        assert(normals.size() == vertex_count);
        mesh.normals = (float*)RL_MALLOC(vertex_count * 3 * sizeof(float));
        memcpy(mesh.normals, normals.data(), vertex_count * 3 * sizeof(float));
    }

    mesh.indices = (uint16_t*)RL_MALLOC(index_count * sizeof(uint16_t));
    memcpy(mesh.indices, indices.data(), index_count * sizeof(uint16_t));

    UploadMesh(&mesh, false);
    return mesh;
}

size_t RigidWorldRenderer::add_body(
    const std::vector<Mesh>& collider_meshes,
    const Mesh& renderable_mesh,
    glm::vec3 trans,
    glm::quat rot) {

    Body body;
    body.rotation = rot;
    body.translation = trans;

    body.renderable = LoadModelFromMesh(renderable_mesh);
    body.renderable.materials[0].shader = _shader;
    body.renderable.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = _checked_tex;
    
    for (const Mesh& m : collider_meshes) {
        body.collider.push_back(LoadModelFromMesh(m));
    }

    static int color_id = 0;
    body.color = _palette[color_id];
    color_id = ++color_id % _palette.size();
    _bodies.push_back(body);
    return _bodies.size() - 1;  
}

void RigidWorldRenderer::update_body(size_t key, glm::quat rotation, glm::vec3 translation) {
    assert(key < _bodies.size());
    if (key >= _bodies.size()) {
        return;
    }

    _bodies[key].rotation = rotation;
    _bodies[key].translation = translation;
}

void RigidWorldRenderer::draw_bases(glm::vec3 origin, glm::mat3 bases, float length) {
    DrawLine3D(V3(origin), V3(origin + bases[0] * length), PURE_RED);
    DrawLine3D(V3(origin), V3(origin + bases[1] * length), PURE_GREEN);
    DrawLine3D(V3(origin), V3(origin + bases[2] * length), PURE_BLUE);
    draw_sphere(V3(origin), length * 0.2f, MAGENTA);
}

#define COLLIDER_VISIBLE 1
#define RENDERABLE_VISIBLE 2

void RigidWorldRenderer::run(
    std::function<void(float frame_dt, size_t frame_id)> update_world_cb,
    std::function<void(float frame_dt, size_t frame_id)> draw_3d_aid_cb,
    Options opts) {

    static bool _camera_free_roam = false;
    static int _collider_renderable_visibility = COLLIDER_VISIBLE | RENDERABLE_VISIBLE;

    // Main game loop
    while (!WindowShouldClose())    // Detect window close button or ESC key
    {
        float dt = GetFrameTime();
        if (update_world_cb) {
            update_world_cb(dt, _frame_id++);
        }

        SetShaderValue(_shader, _shader.locs[SHADER_LOC_VECTOR_VIEW], &_camera.position, SHADER_UNIFORM_VEC3);

        if (_camera_free_roam) {
            UpdateCameraFreeRoam(&_camera, 5.0f, 0.1f);
        }

        glm::vec3 light_dir = _light_obb.bases[2];

        if (opts.movable_light) {
            light_dir = rotate_light(light_dir, dt);
        }

        update_light(_world_aabb, light_dir);
        // _lightCam.position = Vector3Scale(lightDir, -15.0f);
        SetShaderValue(_shader, _lightDirLoc, &light_dir, SHADER_UNIFORM_VEC3);

        BeginDrawing();
        BeginTextureMode(_shadowmap);
        ClearBackground(WHITE);

        Matrix lightView;
        Matrix lightProj;
        // shadow pass
        {
            BeginMode3D(_lightCam);
            lightView = rlGetMatrixModelview();
            lightProj = rlGetMatrixProjection();
            if (_collider_renderable_visibility & RENDERABLE_VISIBLE) {
                draw_renderables();
            }
            EndMode3D();
        }
        EndTextureMode();
        Matrix lightViewProj = MatrixMultiply(lightView, lightProj);

        ClearBackground(GRAY);

        SetShaderValueMatrix(_shader, _lightVPLoc, lightViewProj);

        rlEnableShader(_shader.id);
        int slot = 10; // Can be anything 0 to 15, but 0 will probably be taken up
        rlActiveTextureSlot(10);
        rlEnableTexture(_shadowmap.depth.id);
        rlSetUniform(_shadowmapLoc, &slot, SHADER_UNIFORM_INT, 1);

        // color pass
        {
            BeginMode3D(_camera);
            //std::cout << "camera" << std::endl;
            //std::cout << _camera.target.x << ", " << _camera.target.y << ", " << _camera.target.z << std::endl;
            //std::cout << _camera.position.x << ", " << _camera.position.y << ", " << _camera.position.z << std::endl;

            if (_collider_renderable_visibility & RENDERABLE_VISIBLE) {
                draw_renderables();
            }
            if (_collider_renderable_visibility & COLLIDER_VISIBLE) {
                draw_colliders();
            }
            // draw visual aids
            if (opts.show_light_config) {
                draw_wireframe_aabb(_world_aabb, VIOLET);
                draw_wireframe_obb(_light_obb, VIOLET);
                
                draw_arrow_3d(
                    V3(_light_obb.center - _light_obb.bases[2] * _light_obb.half_dims.z),
                    V3(_light_obb.center - _light_obb.bases[2] * _light_obb.half_dims.z * 0.5f),
                    VIOLET, 0.3f, 0.2f);
            }

            if (opts.show_axis) {
                // DrawGrid(2, 100.0f);
                DrawRay({ {0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f} }, PURE_RED);
                DrawRay({ {0.0f, 0.0f, 0.0f}, {-1.0f, 0.0f, 0.0f} }, PURE_RED);
                DrawRay({ {0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f} }, PURE_GREEN);
                DrawRay({ {0.0f, 0.0f, 0.0f}, {0.0f, -1.0f, 0.0f} }, PURE_GREEN);
                DrawRay({ {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f} }, PURE_BLUE);
                DrawRay({ {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, -1.0f} }, PURE_BLUE);
            }

            if (draw_3d_aid_cb) {
                draw_3d_aid_cb(dt, _frame_id);
            }

            EndMode3D();
        }

        // HUD elements
        if (opts.show_coordinate_gizmo) {
            draw_coordinate_gizmo();
        }
        DrawText("Use [C] to toggle free roam camera", 10, 10, 20, DARKGRAY);
        DrawText("Use [F] to take a screenshot", 10, 35, 20, DARKGRAY);
        DrawText("Use [G] to toggle visibility of colliders and renderables", 10, 60, 20, DARKGRAY);
        if (opts.movable_light) {
            DrawText("Use [H][J][K][U] to rotate directional light", 10, 85, 20, DARKGRAY);
        }

        EndDrawing();

        // key press handling
        static int screenshot_id = 0;
        if (IsKeyPressed(KEY_F)) {
            TakeScreenshot((std::string("screenshot_") + std::to_string(screenshot_id++) + ".png").c_str());
        }

        if (IsKeyPressed(KEY_C)) {
            if (!_camera_free_roam) {
                HideCursor();
                Vector2 screenCenter = { GetScreenWidth() / 2.0f, GetScreenHeight() / 2.0f };
                SetMousePosition(screenCenter.x, screenCenter.y);
            }
            else {
                ShowCursor();
            }
            _camera_free_roam = !_camera_free_roam;
        }
        
        if (IsKeyPressed(KEY_G)) {
            (++_collider_renderable_visibility %= 3) += 1;
        }
        //----------------------------------------------------------------------------------
    }
}

void RigidWorldRenderer::draw_renderables() {
    for (Body& b : _bodies) {
        Vector3 axis = V3(glm::axis(b.rotation));
        // TODO: using degrees?
        float angle_deg = glm::degrees(glm::angle(b.rotation));
        // DrawModelEx(*b.model, V3(b.translation), axis, angle_deg, V3(b.half_dims * 2.0f), b.color);
        DrawModelEx(b.renderable, V3(b.translation), axis, angle_deg, { 1.0f, 1.0f, 1.0f }, b.color);
    }
}

void RigidWorldRenderer::draw_colliders() {
    rlDisableBackfaceCulling();
    for (Body& b : _bodies) {
        Vector3 axis = V3(glm::axis(b.rotation));
        // TODO: using degrees?
        float angle_deg = glm::degrees(glm::angle(b.rotation));
        for (const Model& m : b.collider) {
            DrawModelWiresEx(m, V3(b.translation), axis, angle_deg, { 1.0f, 1.0f, 1.0f }, BLACK);
        }
    }
    rlEnableBackfaceCulling();
}

void RigidWorldRenderer::build_shaders(Config config) {
    _shader = LoadShader((std::string(SHADER_DIR) + "shadowmap.vs").c_str(), (std::string(SHADER_DIR) + "shadowmap.fs").c_str());
    _shader.locs[SHADER_LOC_VECTOR_VIEW] = GetShaderLocation(_shader, "viewPos");
    Vector3 lightDir = Vector3Normalize(V3(config.light_dir));
    Color lightColor = WHITE;
    Vector4 lightColorNormalized = ColorNormalize(lightColor);
    _lightDirLoc = GetShaderLocation(_shader, "lightDir");
    int lightColLoc = GetShaderLocation(_shader, "lightColor");
    SetShaderValue(_shader, _lightDirLoc, &lightDir, SHADER_UNIFORM_VEC3);
    SetShaderValue(_shader, lightColLoc, &lightColorNormalized, SHADER_UNIFORM_VEC4);
    int ambientLoc = GetShaderLocation(_shader, "ambient");
    float ambient[4] = { 0.78f, 0.78f, 0.78f, 1.0f };
    SetShaderValue(_shader, ambientLoc, ambient, SHADER_UNIFORM_VEC4);
    _lightVPLoc = GetShaderLocation(_shader, "lightVP");
    _shadowmapLoc = GetShaderLocation(_shader, "shadowMap");
    SetShaderValue(_shader, GetShaderLocation(_shader, "shadowMapResolution"), &shadowmap_resolution, SHADER_UNIFORM_INT);
}
void RigidWorldRenderer::destroy_shaders() {
    UnloadShader(_shader);
}

void RigidWorldRenderer::build_textures() {
    Image checked_img = GenImageChecked(512, 512, 128, 128, GRAY, WHITE);
    _checked_tex = LoadTextureFromImage(checked_img);
    UnloadImage(checked_img);
}

void RigidWorldRenderer::destroy_textures() {
    UnloadTexture(_checked_tex);
}

void RigidWorldRenderer::build_models() {
    _phong_models[Shape::Cuboid] = LoadModelFromMesh(GenMeshCube(1.0f, 1.0f, 1.0f));
    _phong_models[Shape::Cylinder] = LoadModelFromMesh(GenMeshCylinder(1.0f, 1.0f, 32));
    _phong_models[Shape::Sphere] = LoadModelFromMesh(GenMeshSphere(1.0f, 32, 32));
    _phong_models[Shape::Cone] = LoadModelFromMesh(GenMeshCone(1.0f, 1.0f, 16));
    // set materials
    for (auto& ele : _phong_models) {
        ele.second.materials[0].shader = _shader;
        ele.second.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = _checked_tex;
    }
}

void RigidWorldRenderer::destroy_models() {
    for (auto& ele : _phong_models) {
        UnloadModel(ele.second);
    }
}

void RigidWorldRenderer::destroy_bodies() {
    for (int i = 0; i < _bodies.size(); ++i) {
        Body& b = _bodies[i];
        UnloadModel(b.renderable);
        for (Model& m : b.collider) {
            UnloadModel(m);
        }
    }
}

void RigidWorldRenderer::draw_wireframe_aabb(AABB aabb, Color color) {
    glm::vec3 verts[8] = {
        glm::vec3(aabb.min.x, aabb.min.y, aabb.min.z), // 0
        glm::vec3(aabb.max.x, aabb.min.y, aabb.min.z), // 1
        glm::vec3(aabb.min.x, aabb.max.y, aabb.min.z), // 2
        glm::vec3(aabb.max.x, aabb.max.y, aabb.min.z), // 3
        glm::vec3(aabb.min.x, aabb.min.y, aabb.max.z), // 4
        glm::vec3(aabb.max.x, aabb.min.y, aabb.max.z), // 5
        glm::vec3(aabb.min.x, aabb.max.y, aabb.max.z), // 6
        glm::vec3(aabb.max.x, aabb.max.y, aabb.max.z)// 7
    };

    // Bottom face
    DrawLine3D(V3(verts[0]), V3(verts[1]), color);
    DrawLine3D(V3(verts[1]), V3(verts[3]), color);
    DrawLine3D(V3(verts[3]), V3(verts[2]), color);
    DrawLine3D(V3(verts[2]), V3(verts[0]), color);

    // Top face
    DrawLine3D(V3(verts[4]), V3(verts[5]), color);
    DrawLine3D(V3(verts[5]), V3(verts[7]), color);
    DrawLine3D(V3(verts[7]), V3(verts[6]), color);
    DrawLine3D(V3(verts[6]), V3(verts[4]), color);

    // Vertical edges
    DrawLine3D(V3(verts[0]), V3(verts[4]), color);
    DrawLine3D(V3(verts[1]), V3(verts[5]), color);
    DrawLine3D(V3(verts[2]), V3(verts[6]), color);
    DrawLine3D(V3(verts[3]), V3(verts[7]), color);

    //Model* model = &_default_models.at(Shape::Cuboid);
    //rlEnableWireMode();
    //rlDisableBackfaceCulling();
    //DrawModelEx(*model, V3((aabb.min + aabb.max) * 0.5f), { 0.0f, 1.0f, 0.0f }, 0.0f, V3(aabb.max - aabb.min), color);
    //rlEnableBackfaceCulling();
    //rlDisableWireMode();
}

void RigidWorldRenderer::draw_wireframe_obb(OBB obb, Color color) {
    glm::vec3 min = -obb.half_dims;
    glm::vec3 max = obb.half_dims;

    glm::vec3 verts[8] = {
        obb.bases * glm::vec3(min.x, min.y, min.z) + obb.center, // 0
        obb.bases * glm::vec3(max.x, min.y, min.z) + obb.center, // 1
        obb.bases * glm::vec3(min.x, max.y, min.z) + obb.center, // 2
        obb.bases * glm::vec3(max.x, max.y, min.z) + obb.center, // 3
        obb.bases * glm::vec3(min.x, min.y, max.z) + obb.center, // 4
        obb.bases * glm::vec3(max.x, min.y, max.z) + obb.center, // 5
        obb.bases * glm::vec3(min.x, max.y, max.z) + obb.center, // 6
        obb.bases * glm::vec3(max.x, max.y, max.z) + obb.center  // 7
    };

    // Bottom face
    DrawLine3D(V3(verts[0]), V3(verts[1]), color);
    DrawLine3D(V3(verts[1]), V3(verts[3]), color);
    DrawLine3D(V3(verts[3]), V3(verts[2]), color);
    DrawLine3D(V3(verts[2]), V3(verts[0]), color);

    // Top face
    DrawLine3D(V3(verts[4]), V3(verts[5]), color);
    DrawLine3D(V3(verts[5]), V3(verts[7]), color);
    DrawLine3D(V3(verts[7]), V3(verts[6]), color);
    DrawLine3D(V3(verts[6]), V3(verts[4]), color);

    // Vertical edges
    DrawLine3D(V3(verts[0]), V3(verts[4]), color);
    DrawLine3D(V3(verts[1]), V3(verts[5]), color);
    DrawLine3D(V3(verts[2]), V3(verts[6]), color);
    DrawLine3D(V3(verts[3]), V3(verts[7]), color);

    //Model* model = &_default_models.at(Shape::Cuboid);
    //rlEnableWireMode();
    //rlDisableBackfaceCulling();
    //glm::quat rot(obb.bases);
    //Vector3 axis = V3(glm::axis(rot));
    //float angle_deg = glm::degrees(glm::angle(rot));
    //DrawModelEx(*model, V3(obb.center), axis, angle_deg, V3(obb.half_dims * 2.0f), color);
    //rlEnableBackfaceCulling();
    //rlDisableWireMode();
}

void RigidWorldRenderer::draw_arrow_3d(Vector3 start, Vector3 end, Color color, float girth, float head_ratio) {
    Model* cylinder_model = &_phong_models.at(Shape::Cylinder);
    Model* cone_model = &_phong_models.at(Shape::Cone);

    glm::quat rot = glm::rotation(glm::vec3(0.0f, 1.0f, 0.0f), glm::normalize(v3(end) - v3(start)));
    Vector3 axis = V3(glm::axis(rot));
    float angle_deg = glm::degrees(glm::angle(rot));
    float head_length = Vector3Length(Vector3Subtract(end, start)) * head_ratio;
    float handle_length = head_length * (1.0f - head_ratio) / head_ratio;

    DrawModelEx(*cylinder_model, start, axis, angle_deg, { girth, handle_length, girth }, color);
    DrawModelEx(*cone_model, V3(glm::mix(v3(end), v3(start), head_ratio)), axis, angle_deg, {girth * 1.5f, head_length, girth * 1.5f}, color);
}

void RigidWorldRenderer::draw_sphere(Vector3 center, float radius, Color color) {
    Model* model = &_phong_models.at(Shape::Sphere);
    DrawModelEx(*model, center, {0.0f, 0.0f, 0.0f}, 0.0f, { radius, radius, radius }, color);
}

void RigidWorldRenderer::update_light(AABB world_aabb, glm::vec3 light_dir) {
    OBB obb = DirectionalLightOBB(world_aabb, light_dir);
    _lightCam.position = Vector3Subtract(V3(obb.center), V3(glm::normalize(light_dir)) * obb.half_dims.z);
    _lightCam.target = V3(obb.center);
    _lightCam.projection = CAMERA_ORTHOGRAPHIC;
    _lightCam.up = V3(obb.bases[0]);
    _lightCam.fovy = glm::max(obb.half_dims.x, obb.half_dims.y) * 2.0f;
    _light_obb = obb;
}

glm::vec3 RigidWorldRenderer::rotate_light(glm::vec3 light_dir, float dt, float radps) {
    float theta = glm::clamp(glm::acos(light_dir.y), 0.0f, glm::pi<float>()); // [0, pi]
    float phi = glm::clamp(glm::atan(light_dir.x, light_dir.z), -glm::pi<float>(), glm::pi<float>()); // [-pi, pi]
    if (phi < 0.0f) {
        phi += glm::two_pi<float>();
    }
    if (IsKeyDown(KEY_H)) {
        phi += radps * dt;
    }
    if (IsKeyDown(KEY_K)) {
        phi -= radps * dt;
    }
    if (IsKeyDown(KEY_U)) {
        theta += radps * dt;
    }
    if (IsKeyDown(KEY_J)) {
        theta -= radps * dt;
    }
    theta = glm::clamp(theta, 0.0f, glm::pi<float>()); // [0, pi]
    return glm::vec3(glm::sin(theta) * glm::sin(phi), glm::cos(theta), glm::sin(theta) * glm::cos(phi));
}

void RigidWorldRenderer::draw_coordinate_gizmo() {
    Camera3D cam = _camera;

    cam.position = Vector3Subtract(cam.position, cam.target);
    cam.position = Vector3Scale(Vector3Normalize(cam.position), 16.0f);
    cam.target = { 0.0f, 0.0f, 0.0f };
    
    Vector2 oc = GetWorldToScreen({ 0.0f, 0.0f, 0.0f }, cam);
    Vector2 xc = GetWorldToScreen({ 1.0f, 0.0f, 0.0f }, cam) - oc;
    Vector2 yc = GetWorldToScreen({ 0.0f, 1.0f, 0.0f }, cam) - oc;
    Vector2 zc = GetWorldToScreen({ 0.0f, 0.0f, 1.0f }, cam) - oc;

    Vector2 gizmo_origin = { GetScreenWidth() - 80.0f, 80.0f };

    // Draw axes
    Vector2 xe = gizmo_origin + GetWorldToScreen({ 1.0f, 0.0f, 0.0f }, cam) - oc;
    Vector2 ye = gizmo_origin + GetWorldToScreen({ 0.0f, 1.0f, 0.0f }, cam) - oc;
    Vector2 ze = gizmo_origin + GetWorldToScreen({ 0.0f, 0.0f, 1.0f }, cam) - oc;

    DrawLineEx(gizmo_origin, xe, 5, PURE_RED);
    DrawText("X", (int)xe.x + 5, (int)xe.y, 20, PURE_RED);
    DrawLineEx(gizmo_origin, ye, 5, PURE_GREEN);
    DrawText("Y", (int)ye.x + 5, (int)ye.y, 20, PURE_GREEN);
    DrawLineEx(gizmo_origin, ze, 5, PURE_BLUE);
    DrawText("Z", (int)ze.x + 5, (int)ze.y, 20, PURE_BLUE);
}

void RigidWorldRenderer::UpdateCameraFreeRoam(Camera3D* camera, float moveSpeed, float mouseSensitivity)
{
    // Get screen center
    Vector2 screenCenter = { GetScreenWidth() / 2.0f, GetScreenHeight() / 2.0f };

    // Get mouse position
    Vector2 mousePos = GetMousePosition();

    // Calculate delta from center
    Vector2 delta = {
        mousePos.x - screenCenter.x,
        mousePos.y - screenCenter.y
    };

    // Recenter mouse cursor
    SetMousePosition(screenCenter.x, screenCenter.y);

    // Calculate current forward vector from camera
    Vector3 currentForward = Vector3Normalize(Vector3Subtract(camera->target, camera->position));

    // Calculate current yaw and pitch
    float yaw, pitch;

    // Calculate pitch (vertical angle)
    pitch = asinf(currentForward.y) * RAD2DEG;

    // Calculate yaw (horizontal angle) - handle all quadrants
    if (fabsf(currentForward.x) > 0.001f || fabsf(currentForward.z) > 0.001f) {
        yaw = atan2f(currentForward.z, currentForward.x) * RAD2DEG;
    }
    else {
        yaw = 0.0f;
    }

    // Ensure yaw is in reasonable range
    while (yaw < 0.0f) yaw += 360.0f;
    while (yaw >= 360.0f) yaw -= 360.0f;

    // Mouse look - update angles based on mouse movement
    yaw += delta.x * mouseSensitivity;
    pitch -= delta.y * mouseSensitivity;

    // Clamp pitch to avoid gimbal lock
    if (pitch > 89.0f) pitch = 89.0f;
    if (pitch < -89.0f) pitch = -89.0f;

    // Calculate new forward vector from updated angles
    Vector3 forward = {
        cosf(DEG2RAD * pitch) * cosf(DEG2RAD * yaw),
        sinf(DEG2RAD * pitch),
        cosf(DEG2RAD * pitch) * sinf(DEG2RAD * yaw)
    };
    forward = Vector3Normalize(forward);

    // Right and up vectors
    Vector3 right = Vector3Normalize(Vector3CrossProduct(forward, { 0, 1, 0 }));
    Vector3 up = Vector3CrossProduct(right, forward);

    // Keyboard movement (WASD)
    float dt = GetFrameTime();
    if (IsKeyDown(KEY_W)) camera->position = Vector3Add(camera->position, Vector3Scale(forward, moveSpeed * dt));
    if (IsKeyDown(KEY_S)) camera->position = Vector3Subtract(camera->position, Vector3Scale(forward, moveSpeed * dt));
    if (IsKeyDown(KEY_A)) camera->position = Vector3Subtract(camera->position, Vector3Scale(right, moveSpeed * dt));
    if (IsKeyDown(KEY_D)) camera->position = Vector3Add(camera->position, Vector3Scale(right, moveSpeed * dt));
    if (IsKeyDown(KEY_LEFT_CONTROL))    camera->position = Vector3Subtract(camera->position, Vector3Scale(up, moveSpeed * dt));
    if (IsKeyDown(KEY_SPACE))           camera->position = Vector3Add(camera->position, Vector3Scale(up, moveSpeed * dt));

    // Update camera target and up
    camera->target = Vector3Add(camera->position, forward);
    camera->up = up;
}