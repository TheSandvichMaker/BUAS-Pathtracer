#pragma once

struct IntegratorOption;

struct Medium {
    V3 absorb;
};

enum MaterialFlag {
    Material_Mirror   = 0x1,
    Material_Checkers = 0x2,
    Material_Emissive = 0x4,
};

struct Material {
    u32 flags;

    V3 albedo;
    V3 checker_color;
    V3 emission_color;

    f32 ior;
    f32 metallic;
    f32 roughness;

    // TODO: something cool instead of a dumb boolean
    b32 is_participating_medium;
    Medium medium;
};

struct Camera {
    V3 p;
    V3 x;
    V3 y;
    V3 z;

    f32 vfov;
    f32 aspect_ratio;

    f32 lens_radius;
    f32 focus_distance;

    f32 film_distance;
    f32 half_film_w;
    f32 half_film_h;
};

struct StratifiedDisk {
    static constexpr usize dim = 8;
    static constexpr f32 rcp_dim = 1.0f / (f32)dim;
    union {
        f32 strata[dim][dim];
        f32 all_strata[dim*dim];
    };
};

struct ScreenspacePathGuide {
    static constexpr usize grid_size = 1;

    u32 w, h;
    StratifiedDisk* data;
};

struct SceneSettings {
    b32 next_event_estimation;
    b32 importance_sample_lights;
    b32 importance_sample_diffuse;
    b32 use_mis;
    b32 russian_roulette;
    b32 caustics;
    SamplingStrategy sampling_strategy;
    b32 use_path_guide;
    f32 vignette_strength;
    f32 lens_distortion;
    f32 f_factor;
    f32 diaphragm_edges;
    f32 phi_shutter_max;
    u32 samples_per_pixel;
    u32 max_bounce_count;

    IntegratorOption* integrator;
};

struct PostProcessSettings {
    f32 exposure;
    b32 tonemapping;
    b32 srgb_transform;
    f32 midpoint;
    f32 contrast;
};

struct Scene {
    const char* name;

    Arena arena;

    u32 total_frame_index;

    EnvironmentMap* skydome;
    V3 top_sky_color;
    V3 bot_sky_color;
    V3 ambient_light;

    Camera camera;
    Camera new_camera;

    f32 t;

    SceneSettings settings;
    SceneSettings new_settings;

    PostProcessSettings post_settings;

    StretchyBuffer<Material> materials;
    StretchyBuffer<PrimitiveID> lights;
    StretchyBuffer<Primitive> planes; // NOTE: This is a separate array, because planes don't really work in a BVH
    StretchyBuffer<Primitive> primitives;
    
    BVH* bvh;
};

struct AddPrimitiveResult {
    PrimitiveID id;
    Primitive* primitive;
};

inline SceneSettings*
get_writable_settings(Scene* scene) {
    return &scene->new_settings;
}

//

MaterialID add_material(Scene* scene, Material source_material);
MaterialID add_diffuse_material(Scene* scene, V3 diffuse_color, f32 ior, f32 roughness = 0.0f, b32 checkers = false, V3 checker_color = v3(0.1f));
MaterialID add_translucent_material(Scene* scene, V3 absorb, f32 ior, f32 roughness = 0.0f);
MaterialID add_emissive_material(Scene* scene, V3 emission_color);
M4x4Inv* push_transform(Scene* scene, M4x4Inv transform);
AddPrimitiveResult add_primitive(Scene* scene, PrimitiveType type, MaterialID material_id, M4x4Inv* transform = nullptr);
PrimitiveID add_plane(Scene* scene, MaterialID material_id, V3 n, f32 d);
PrimitiveID add_sphere(Scene* scene, MaterialID material_id, f32 r, M4x4Inv* transform = nullptr);
PrimitiveID add_sphere(Scene* scene, MaterialID material_id, f32 r, M4x4Inv transform);
PrimitiveID add_box(Scene* scene, MaterialID material_id, V3 r, M4x4Inv* transform = nullptr);
PrimitiveID add_box(Scene* scene, MaterialID material_id, V3 r, M4x4Inv transform);
PrimitiveID add_mesh(Scene* scene, MaterialID material_id, Mesh* mesh, M4x4Inv* transform = nullptr);
PrimitiveID add_mesh(Scene* scene, MaterialID material_id, Mesh* mesh, M4x4Inv transform);
PrimitiveID add_test_difference(Scene* scene, MaterialID positive_material, MaterialID negative_material, M4x4Inv* transform = nullptr);
void create_scene_bvh(Scene* scene, Arena* temp_arena);
void clear_scene(Scene* scene);
