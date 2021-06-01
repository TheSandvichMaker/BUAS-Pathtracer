#include "precomp.h"
#include "common.h"
#include "primitives.h"
#include "assets.h"
#include "bvh.h"
#include "samplers.h"
#include "scene.h"
#include "intersection.h"

TraversalStats g_stats;

static force_inline b32
ray_intersect_plane(const Ray& ray,
                    const V3& plane_n,
                    f32 plane_distance,
                    f32* out_t)
{
    b32 result = false;

    // NOTE: From "Realtime Collision Detection" by Christer Ericson, page 176:
    //       "Note that this code does not explicitly handle division by zero. Assuming IEEE-
    //        754 floating-point arithmetic, it does still give the correct result in the case
    //        of the denominator being 0."
    //       So, I had the check for division by 0 in here at first, but now I've eliminated it.
    //       Reading the section in the book on avoiding division by 0 checks, the reason why this
    //       works is because in x / 0, for a non-zero x the answer will be either INF or -INF,
    //       and INF has well-defined ordering. That is, -INF < x < INF for any x that is isn't INF
    //       or NaN. Therefore, our check ((t >= EPSILON) && (t < *out_t)) will correctly discard
    //       a t of -INF or INF, and return non-intersection for the case where our camera is exactly
    //       aligned with the plane, where dot(plane_n, ray_d) would be 0.

    f32 denom = dot(plane_n, ray.d);
    if (denom < -EPSILON) {
        f32 t = (plane_distance - dot(plane_n, ray.o)) / denom;
        if ((t >= EPSILON) && (t < *out_t)) {
            result = true;
            *out_t = t;
        }
    }

    return result;
}

static force_inline b32
ray_intersect_sphere(const Ray& ray,
                     f32 sphere_r,
                     f32* out_t)
{
    b32 result = false;

    // NOTE: Source: "Realtime Collision Detection" by Christer Ericson, page 177

    V3 sphere_rel_o = ray.o;
    f32 sphere_r_sq = sphere_r*sphere_r;

    f32 b = dot(ray.d, sphere_rel_o);
    f32 c = dot(sphere_rel_o, sphere_rel_o) - sphere_r_sq;

    f32 discr = (b*b - c);
    // NOTE: Check the discriminant to determine there is at least one intersection with the sphere
    if (discr >= 0) {
        // NOTE: The ray intersects the sphere, compute the closest intersection
        f32 discr_root = square_root(discr);
        f32 tn = -b - discr_root;
        f32 tf = -b + discr_root;
        f32 t = (tn >= 0.0f ? tn : tf);
        if ((t >= EPSILON) && (*out_t > t)) {
            result = true;
            *out_t = t;
        }
    }

    return result;
}

static force_inline b32
ray_intersect_box(const Ray& ray,
                  const V3& box_r,
                  f32* out_t)
{
    b32 result = false;

	// NOTE: Box intersection based on https://www.iquilezles.org/www/articles/boxfunctions/boxfunctions.htm
    V3 box_rel_p = ray.o;

    V3 m = ray.inv_d;
    V3 n = m*box_rel_p;
    V3 k = absolute_value(m)*box_r;

    V3 t1 = -n - k;
    V3 t2 = -n + k;

    f32 tn = max(max(t1.x, t1.y), t1.z);
    f32 tf = min(min(t2.x, t2.y), t2.z);

    if (tn < tf) {
        f32 t = (tn >= 0.0f ? tn : tf);
        if ((*out_t > t) && (t >= EPSILON)) {
            *out_t = t;
            result = true;
        }
    }

    return result;
}

static force_inline b32
ray_intersect_bounding_volume(const Ray& ray,
                              const V3& box_p,
                              const V3& box_r,
                              f32 far_clip)
{
    b32 result = false;

	// NOTE: Box intersection based on https://www.iquilezles.org/www/articles/boxfunctions/boxfunctions.htm
    V3 box_rel_p = ray.o - box_p;

    V3 m = ray.inv_d;
    V3 n = m*box_rel_p;
    V3 k = absolute_value(m)*box_r;

    V3 t1 = -n - k;
    V3 t2 = -n + k;

    f32 tn = max(max(t1.x, t1.y), t1.z);
    f32 tf = min(min(t2.x, t2.y), t2.z);

    if (((tn < tf) && (tf > 0.0f)) && (tn < far_clip)) {
        result = true;
    }

    return result;
}

static force_inline b32
ray_intersect_triangle(const Ray& ray,
                       const V3& a,
                       const V3& b,
                       const V3& c,
                       f32* out_t,
                       V3* out_uvw)
{
    f32 epsilon = 0.000000001f;

    V3 edge1 = b - a;
    V3 edge2 = c - a;

    V3 pvec = cross(ray.d, edge2);

    f32 det = dot(edge1, pvec);

    if (det > -epsilon && det < epsilon) {
        return false;
    }

    f32 inv_det = 1.0f / det;

    V3 tvec = ray.o - a;

    f32 v = dot(tvec, pvec)*inv_det;
    if (v < 0.0f || v > 1.0f) {
        return false;
    }

    V3 qvec = cross(tvec, edge1);

    f32 w = dot(ray.d, qvec)*inv_det;
    if (w < 0.0f || v + w > 1.0f) {
        return false;
    }

    f32 t = dot(edge2, qvec)*inv_det;

    if ((t < epsilon) || (*out_t < t)) {
        return false;
    }

    *out_t = t;
    *out_uvw = v3(1.0f - v - w, v, w);

    return true;
}

static force_inline f32_4x
ray_intersect_triangle_4x(const V3_4x& ray_o,
                          const V3_4x& ray_d,
                          const V3_4x& a,
                          const V3_4x& b,
                          const V3_4x& c,
                          f32_4x* out_t,
                          f32_4x* out_u,
                          f32_4x* out_v,
                          f32_4x* out_w)
{
    f32_4x epsilon = f32_4x::from(0.000000001f);
    f32_4x one = f32_4x::from(1.0f);
    f32_4x zero = f32_4x::from(0.0f);

    V3_4x edge1 = b - a;
    V3_4x edge2 = c - a;

    V3_4x pvec = cross(ray_d, edge2);

    f32_4x det = dot(edge1, pvec);

    f32_4x hit_mask = ((det <= -epsilon) | (det >= epsilon));
    if (mask_is_zero(hit_mask)) {
        return hit_mask;
    }

    f32_4x inv_det = f32_4x::from(1.0f) / det;

    V3_4x tvec = ray_o - a;

    f32_4x v = dot(tvec, pvec)*inv_det;
    hit_mask = hit_mask & ((v >= zero) & (v <= one));
    if (mask_is_zero(hit_mask)) {
        return hit_mask;
    }

    V3_4x qvec = cross(tvec, edge1);

    f32_4x w = dot(ray_d, qvec)*inv_det;
    hit_mask = hit_mask & ((w >= zero) & ((v + w) <= one));
    if (mask_is_zero(hit_mask)) {
        return hit_mask;
    }

    f32_4x t = dot(edge2, qvec)*inv_det;
    hit_mask = hit_mask & ((t >= epsilon) & (t <= *out_t));
    if (mask_is_zero(hit_mask)) {
        return hit_mask;
    }

    *out_t = t;
    *out_u = f32_4x::from(1.0f) - v - w;
    *out_v = v;
    *out_w = w;

    return hit_mask;
}

static force_inline b32
intersect_mesh(Mesh*         mesh,
               const Ray&    ray,
               IntersectType type,
               f32*          out_t,
               u32*          out_hit_triangle_index,
               V3*           out_uvw,
               V3*           out_a,
               V3*           out_b,
               V3*           out_c)
{
    atomic_add(&g_stats.mesh_intersection_count, 1);

    MeshBVH* bvh = mesh->bvh;

    u32 hit_triangle_index = UINT32_MAX;
    if (bvh) {
        u32 node_stack_at = 0;
        u32 node_stack[64];

        // NOTE: Push on root node
        node_stack[node_stack_at++] = 0;

        u64 traversals = 0;
        u64 leaf_traversals = 0;
        u64 node_traversals = 0;
        while (node_stack_at > 0) {
            u32 node_index = node_stack[--node_stack_at];

            BVHNode* node = &bvh->nodes[node_index];

            ++traversals;

            f32 node_t = *out_t;
            if (ray_intersect_bounding_volume(ray, node->bv_p, node->bv_r, node_t)) {
                if (node->count) {
                    ++leaf_traversals;

                    u32 first          = node->left_first;
                    u32 triangle_count = node->count;

                    switch (bvh->storage) {
                        case BVHStorage_Scalar: {
                            u32* triangle_indices = &bvh->indices[first];
                            Triangle* triangles   = &bvh->triangles[first];

                            for (usize triangle_lookup_index = 0; triangle_lookup_index < triangle_count; ++triangle_lookup_index) {
                                Triangle* triangle = &triangles[triangle_lookup_index];

                                V3 a = triangle->a;
                                V3 b = triangle->b;
                                V3 c = triangle->c;

                                if (ray_intersect_triangle(ray, a, b, c, out_t, out_uvw)) {
                                    if (type == Intersect_Occlusion) {
                                        return true;
                                    }

                                    *out_a = a;
                                    *out_b = b;
                                    *out_c = c;

                                    hit_triangle_index = triangle_indices[triangle_lookup_index];
                                }
                            }
                        } break;

                        case BVHStorage_4x: {
                            V3_4x ray_o = V3_4x::from(ray.o);
                            V3_4x ray_d = V3_4x::from(ray.d);

                            // TODO: We need to look up indices in increments of 4, this is a little obtuse.
                            u32* triangle_indices  = &bvh->indices[4*first];
                            Triangle_4x* triangles = &bvh->triangles_4x[first];

                            for (usize triangle_4x_index = 0; triangle_4x_index < triangle_count; triangle_4x_index += 1) {
                                Triangle_4x* triangle = &triangles[triangle_4x_index];

                                V3_4x a = triangle->a;
                                V3_4x b = triangle->b;
                                V3_4x c = triangle->c;

                                f32_4x t = f32_4x::from(*out_t);
                                f32_4x u, v, w;

                                f32_4x hit_mask = ray_intersect_triangle_4x(ray_o, ray_d, a, b, c, &t, &u, &v, &w);
                                if (!mask_is_zero(hit_mask)) {
                                    if (type == Intersect_Occlusion) {
                                        return true;
                                    }

                                    usize lowest_t_index = (usize)-1;
                                    f32 lowest_t = *out_t;
                                    for (usize i = 0; i < 4; ++i) {
                                        if (extract_mask(hit_mask, i)) {
                                            f32 test_t = t[i];
                                            if (test_t < lowest_t) {
                                                lowest_t = test_t;
                                                lowest_t_index = i;
                                            }
                                        }
                                    }

                                    *out_t = lowest_t;
                                    *out_uvw = v3(u[lowest_t_index], v[lowest_t_index], w[lowest_t_index]);
                                    *out_a = extract(a, lowest_t_index);
                                    *out_b = extract(b, lowest_t_index);
                                    *out_c = extract(c, lowest_t_index);

                                    hit_triangle_index = triangle_indices[4*triangle_4x_index + lowest_t_index];
                                }
                            }
                        } break;
                    }
                } else {
                    ++node_traversals;

                    u32 left = node->left_first;

                    // NOTE: BVH traversal order check based on ISPC raytracer example
                    //       This pushes the closest node on second so that we traverse
                    //       it next.
                    if (ray.d_is_negative[node->split_axis]) {
                        // NOTE: Traverse right node first by pushing it on the stack second
                        node_stack[node_stack_at++] = left;
                        node_stack[node_stack_at++] = left + 1;
                    } else {
                        // NOTE: Traverse left node first by pushing it on the stack second
                        node_stack[node_stack_at++] = left + 1;
                        node_stack[node_stack_at++] = left;
                    }
                }
            }
        }

        atomic_add(&g_stats.mesh_bvh_traversals, traversals);
        atomic_add(&g_stats.mesh_node_traversals, node_traversals);
        atomic_add(&g_stats.mesh_leaf_traversals, leaf_traversals);
    } else {
        u32 triangle_count = mesh->triangle_count;
        Triangle* triangles = mesh->triangles;
        for (usize i = 0; i < triangle_count; ++i) {
            Triangle* triangle = &mesh->triangles[i];
            if (ray_intersect_triangle(ray, triangle->a, triangle->b, triangle->c, out_t, out_uvw)) {
                if (type == Intersect_Occlusion) {
                    return true;
                }
                hit_triangle_index = (u32)i;
            }
        }
    }

    if (hit_triangle_index != UINT32_MAX) {
        *out_hit_triangle_index = hit_triangle_index;
        return true;
    }

    return false;
}

static force_inline Ray
transform_ray(Ray ray, M4x4 m) {
    V3 ray_o = transform(m, ray.o, 1.0f);
    V3 ray_d = transform(m, ray.d, 0.0f);
    Ray result = make_ray(ray_o, ray_d, ray.max_t);
    return result;
}

static Primitive*
intersect_scene_internal(Scene*        scene,
                         const Ray&    ray,
                         IntersectType type,
                         PrimitiveID   ignored_primitive_index, // NOTE: For excluding the light source from shadow rays
                         f32*          out_t         = nullptr,
                         V3*           out_hit_p     = nullptr,
                         V3*           out_n         = nullptr)
{
	f32 t = ray.max_t;
    
    Primitive* hit_primitive = nullptr;

    for (usize plane_index = 0; plane_index < scene->planes.count; ++plane_index) {
        Primitive* primitive = &scene->planes[plane_index];
        assert_slow(primitive->type == Primitive_Plane);

        Plane* plane = &primitive->plane;

        if (ray_intersect_plane(ray, plane->n, plane->d, &t)) {
            hit_primitive = primitive;
        }
    }

    u32 hit_triangle_index = 0;
    V3 a = {}, b = {}, c = {};
    V3 uvw = {};

    Ray object_space_ray;

    BVH* bvh = scene->bvh;
    assert(bvh);

    u32 node_stack_at = 0;
    u32 node_stack[64];

    // NOTE: Push on root node
    node_stack[node_stack_at++] = 0;

    while (node_stack_at > 0) {
        BVHNode* node = &bvh->nodes[node_stack[--node_stack_at]];

        f32 node_t = t;
        if (ray_intersect_bounding_volume(ray, node->bv_p, node->bv_r, node_t)) {
            if (node->count) {
                u32 first = node->left_first;

                u32  primitive_count   = node->count;
                u32* primitive_indices = &bvh->indices[first];

                for (usize primitive_lookup_index = 0;
                     primitive_lookup_index < primitive_count;
                     ++primitive_lookup_index)
                {
                    u32 primitive_index = primitive_indices[primitive_lookup_index];
                    if (primitive_index == ignored_primitive_index) {
                        continue;
                    }

                    Primitive* primitive = &scene->primitives[primitive_index];

                    Ray intersect_ray = transform_ray(ray, primitive->transform->inverse);

                    b32 hit_any = false;
                    switch (primitive->type) {
                        case Primitive_Sphere: {
                            Sphere* sphere = &primitive->sphere;
                            hit_any = ray_intersect_sphere(intersect_ray, sphere->r, &t);
                        } break;

                        case Primitive_Box: {
                            Box* box = &primitive->box;
                            hit_any = ray_intersect_box(intersect_ray, box->r, &t);
                        } break;

                        case Primitive_Mesh: {
                            Mesh* mesh = &primitive->mesh;
                            hit_any = intersect_mesh(mesh, intersect_ray, type, &t, &hit_triangle_index, &uvw, &a, &b, &c);
                        } break;
                    }

                    if (hit_any) {
                        if (type == Intersect_Occlusion) {
                            return primitive;
                        }

                        hit_primitive = primitive;
                        object_space_ray = intersect_ray;
                    }
                }
            } else {
                u32 left = node->left_first;

                // NOTE: BVH traversal order check based on ISPC raytracer example
                //       This pushes the node closest to the camera on second so that
                //       we traverse it next, if you don't see how this works I recommend
                //       working it out in your head, since it'd be clearer than me
                //       explaining it.
                if (ray.d_is_negative[node->split_axis]) {
                    // NOTE: Traverse right node first by pushing it on the stack second
                    node_stack[node_stack_at++] = left;
                    node_stack[node_stack_at++] = left + 1;
                } else {
                    // NOTE: Traverse left node first by pushing it on the stack second
                    node_stack[node_stack_at++] = left + 1;
                    node_stack[node_stack_at++] = left;
                }
            }
        }
    }

    //
    // :NormalCalculation
    //

    if (hit_primitive) {
        V3 object_space_hit_p = object_space_ray.o + t*object_space_ray.d;
        V3 hit_p = ray.o + t*ray.d;
        if (out_hit_p) {
            *out_hit_p = hit_p;
        }

        if (out_n) {
            V3 n = {};
            switch (hit_primitive->type) {
                case Primitive_Plane: {
                    auto plane = &hit_primitive->plane;
                    n = plane->n;
                } break;

                case Primitive_Sphere: {
                    auto sphere = &hit_primitive->sphere;
                    n = object_space_hit_p;
                } break;

                case Primitive_Box: {
                    auto box = &hit_primitive->box;
                    V3 rel_p = object_space_hit_p / box->r;

#if 1
                    u32 largest_element_index = 0;
                    f32 largest_element = absolute_value(rel_p.x);
                    if (absolute_value(rel_p.y) > largest_element) {
                        largest_element_index = 1;
                        largest_element = absolute_value(rel_p.y);
                    }
                    if (absolute_value(rel_p.z) > largest_element) {
                        largest_element_index = 2;
                        largest_element = absolute_value(rel_p.z);
                    }
                    n = v3(0, 0, 0);
                    n.e[largest_element_index] = sign_of(rel_p.e[largest_element_index]);
#else
                    f32 bias = 1.0f + EPSILON;
                    n = v3((f32)((s32)(rel_p.x*bias)),
                           (f32)((s32)(rel_p.y*bias)),
                           (f32)((s32)(rel_p.z*bias)));
#endif
                } break;

                case Primitive_Mesh: {
                    auto mesh = &hit_primitive->mesh;

                    if (mesh->has_normals) {
						Triangle* normal_triangle = &get_normals(mesh)[hit_triangle_index];
						n = (uvw.x*normal_triangle->a +
						     uvw.y*normal_triangle->b +
							 uvw.z*normal_triangle->c);
                    } else {
						V3 e1 = normalize(b - a);
						V3 e2 = normalize(c - a);
						n = cross(e1, e2);
                    }
                } break;
            }

            n = noz(transform_normal(hit_primitive->transform->inverse, n));

            *out_n = n;
        }
    }

    if (out_t) {
        *out_t = t;
    }

    return hit_primitive;
}

b32
intersect_shadow_ray(Scene* scene, const Ray& ray, PrimitiveID ignored_primitive_index) {
    b32 result = !!intersect_scene_internal(scene, ray, Intersect_Occlusion, ignored_primitive_index);
    return result;
}

Primitive*
intersect_scene(Scene* scene, const Ray& ray, f32* out_t, V3* out_hit_p, V3* out_n) {
    Primitive* result = intersect_scene_internal(scene, ray, Intersect_Full, PrimitiveID::from(0), out_t, out_hit_p, out_n);
    return result;
}

