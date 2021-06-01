#pragma once

struct Scene;

struct Ray {
    V3 o;
    V3 d;
    V3 inv_d;
    b32 d_is_negative[3];
    f32 max_t;
};

inline Ray
make_ray(V3 o, V3 d, f32 far_clip = FLT_MAX) {
    Ray result;
    result.o = o;
    result.d = d;
    result.inv_d = 1.0f / d;
    result.d_is_negative[0] = d.x < 0.0f;
    result.d_is_negative[1] = d.y < 0.0f;
    result.d_is_negative[2] = d.z < 0.0f;
    result.max_t = far_clip;
    return result;
}

enum IntersectType {
    Intersect_Full,
    Intersect_Occlusion,
};

//

struct TraversalStats {
    volatile u64 mesh_intersection_count = 0;
    volatile u64 mesh_bvh_traversals     = 0;
    volatile u64 mesh_node_traversals    = 0;
    volatile u64 mesh_leaf_traversals    = 0;
};

extern TraversalStats g_stats;

//

b32 intersect_shadow_ray(Scene* scene, const Ray& ray, PrimitiveID ignored_primitive_index);
Primitive* intersect_scene(Scene* scene, const Ray& ray, f32* out_t = nullptr, V3* out_hit_p = nullptr, V3* out_n = nullptr);

