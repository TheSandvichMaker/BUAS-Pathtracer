#include "precomp.h"
#include "common.h"
#include "samplers.h"
#include "primitives.h"
#include "assets.h"
#include "bvh.h"
#include "scene.h"
#include "intersection.h"
#include "integrators.h"

static V3
random_in_unit_sphere(RandomSeries* entropy) {
    V3 result;
    do {
        result = random_bilaterals(entropy).xyz;
    } while (length_sq(result) >= 1.0f);

    return result;
}

static V2
random_in_unit_disk(RandomSeries* entropy) {
    V2 result;
    do {
        result = random_bilaterals(entropy).xy;
    } while (length_sq(result) >= 1.0f);

    return result;
}

static V2
sample_in_unit_disk(V2 u) {
    V2 result = v2(0.0f, 0.0f);
    V2 u_offset = 2.0f*u - v2(1.0f, 1.0f);
    if ((u_offset.x != 0.0f) || (u_offset.y != 0.0f)) {
        f32 theta, r;
        if (absolute_value(u_offset.x) > absolute_value(u_offset.y)) {
            r = u_offset.x;
            theta = 0.25f*PI_32*(u_offset.y / u_offset.x);
        } else {
            r = u_offset.y;
            theta = 0.5f*PI_32 - 0.25f*PI_32*(u_offset.x / u_offset.y);
        }
        result = r*v2(cosf(theta), sinf(theta));
    }
    return result;
}

static V3
sample_on_unit_sphere(V2 u) {
    f32 z = 1.0f - 2.0f*u.x;
    f32 r = square_root(max(0.0f, 1.0f - z*z));
    f32 phi = 2.0f*PI_32*u.y;
    V3 result = v3(r*cosf(phi), r*sinf(phi), z);
    return result;
}

static force_inline void
get_tangents(const V3& n, V3* b1, V3* b2) {
    // SOURCE: https://graphics.pixar.com/library/OrthonormalB/paper.pdf
    f32 sign = copy_sign(1.0f, n.z);
    f32 a = -1.0f / (sign + n.z);
    f32 b = n.x*n.y*a;
    *b1 = v3(1.0f + sign*n.x*n.x*a, sign*b, -sign*n.x);
    *b2 = v3(b, sign + n.y*n.y*a, -n.y);
}

static force_inline V3
oriented_around_normal(V3 v, V3 N) {
	V3 T, B;
	get_tangents(N, &T, &B);

    V3 result = (v.x*B + v.y*N + v.z*T);
    return result;
}

static V3
random_in_cone(V3 N, f32 angle, V2 random_sample) {
    f32 cos_angle = cosf(angle);

	f32 azimuth = TAU_32*random_sample.x;
	f32 y       = cos_angle + (1.0f - cos_angle)*random_sample.y;

    V3 hemi;
    hemi.x = cosf(azimuth)*square_root(1.0f - y*y);
    hemi.y = y;
    hemi.z = sinf(azimuth)*square_root(1.0f - y*y);

    V3 result = oriented_around_normal(hemi, N);
    return result;
}

static force_inline V3
map_to_hemisphere(V3 N, V2 random_sample) {
	f32 azimuth = TAU_32*random_sample.x;
	f32 y       = random_sample.y;

    V3 hemi;
    hemi.x = cosf(azimuth)*square_root(1.0f - y*y);
    hemi.y = y;
    hemi.z = sinf(azimuth)*square_root(1.0f - y*y);

    V3 result = oriented_around_normal(hemi, N);
    return result;
}

static force_inline V3
map_to_cosine_weighted_hemisphere(V3 N, V2 random_sample) {
	f32 azimuth = TAU_32*random_sample.x;
	f32 y       = random_sample.y;

    V3 hemi;
	hemi.x = cosf(azimuth)*square_root(1.0f - y);
	hemi.y = square_root(y);
	hemi.z = sinf(azimuth)*square_root(1.0f - y);

	V3 result = oriented_around_normal(hemi, N);
    return result;
}

static b32 g_solid_angle_test = true;

static force_inline f32
projected_solid_angle(Primitive* primitive, V3 v, f32 distance_sq) {
    f32 result = 0.0f;
    switch (primitive->type) {
        case Primitive_Sphere: {
            Sphere* sphere = &primitive->sphere;
            result = PI_32*sphere->r*sphere->r / distance_sq;
        } break;
    }
    return result;
}

static force_inline PrimitiveID
pick_random_light(Scene* scene, f32 random_sample, V3 I, Arena* task_arena, f32* out_rcp_pdf) {
    PrimitiveID result = PrimitiveID::from(0);

    // TODO: What makes more sense when importance sampling, using perceptual luma or just max3?
    //       I can imagine using perceptual luma might create a perceptually less noisy result.

    u32 light_count = scene->lights.count;
    if (light_count > 0) {
        if (scene->settings.importance_sample_lights) {
            ScopedMemory scoped_memory(task_arena);

            f32 sum = 0.0f;
            f32* pdfs = push_array(task_arena, light_count, f32, no_clear());
            f32* cdf  = push_array(task_arena, light_count, f32);

            for (usize i = 0; i < light_count; ++i) {
                PrimitiveID light_id = scene->lights[i];
                Primitive* light = &scene->primitives[light_id];
                Material* material = &scene->materials[light->material_id];

                V3 light_v = translation(light->transform->forward) - I;
                f32 light_distance_sq = length_sq(light_v);

                f32 l = max3(material->emission_color);
                f32 pdf = l*projected_solid_angle(light, light_v, light_distance_sq);

                sum    += pdf;
                pdfs[i] = pdf;

                f32 prev = (i > 0 ? cdf[i - 1] : 0.0f);
                f32 curr = pdf;

                cdf[i] = prev + curr;
            }

            f32 e = sum*random_sample;

            u32 light_index = 0;
            while (cdf[light_index] < e) ++light_index;

            *out_rcp_pdf = pdfs[light_index] / sum;
             result      = scene->lights[light_index];
        } else {
            *out_rcp_pdf = 1.0f / (f32)light_count;
            // FUN FACT: If you put the code to calculate index in the lookup
            //           in the lights array directly, even though I am explicitly
            //           casting to u32, it will _still_ see the fact that the
            //           stretchy buffer [] overload takes a usize, and intellisense
            //           will still complain about a 32 bit calculation being
            //           upcast to 64! Hope you enjoyed that fun fact.
            u32 light_index = (u32)(random_sample*(f32)light_count - EPSILON);
            result = scene->lights[light_index];
        }
    }

    return result;
}

struct LightSample {
    V3 L, Nl;
    f32 dist, dist_sq, A;
};

static force_inline LightSample
random_point_on_light(Primitive* light, V2 random_sample, V3 I) {
    LightSample result = {};

    V3 light_p = translation(light->transform->forward);
    V3 towards_light = normalize(light_p - I);

    V3 sample_point = {};
    switch (light->type) {
        case Primitive_Sphere: {
            Sphere* sphere = &light->sphere;
            V3 Nl      = map_to_hemisphere(-towards_light, random_sample);
            V3 p       = Nl*sphere->r;
            V3 p_world = light->transform->forward*p;

            V3 L = p_world - I;
            result.dist_sq = length_sq(L);
            result.dist    = square_root(result.dist_sq);
            L /= result.dist;

            result.A  = 2.0f*PI_32*sphere->r*sphere->r;
            result.L  = L;
            result.Nl = Nl;
        } break;

        INVALID_DEFAULT_CASE;
    }

    return result;
}

static force_inline V3
sample_environment_map(Scene* scene, EnvironmentMap* map, V2 random_sample, V3 I, V3 N, Arena* task_arena) {
    /* NOT YET IMPLEMENTED */
}

static force_inline f32
fresnel_dielectric(f32 cos_theta_i, f32 eta_i, f32 eta_t, f32 eta_i_over_eta_t, f32* out_cos_theta_t) {
    // SOURCE: http://www.pbr-book.org/3ed-2018/Reflection_Models/Specular_Reflection_and_Transmission.html#fragment-ComputemonocosThetaTusingSnellslaw-0

    // NOTE: Compute cos_theta_t using snell's law
    f32 sin_theta_i = square_root(max(0.0f, 1.0f - cos_theta_i*cos_theta_i));
    f32 sin_theta_t = eta_i_over_eta_t*sin_theta_i;
    f32 cos_theta_t = square_root(max(0.0f, 1.0f - sin_theta_t*sin_theta_t));

    *out_cos_theta_t = cos_theta_t;

    // NOTE: Handle total internal reflection
    if (sin_theta_t >= 1) {
        return 1;
    }

    f32 r_parallel      = (((eta_t*cos_theta_i) - (eta_i*cos_theta_t)) /
                           ((eta_t*cos_theta_i) + (eta_i*cos_theta_t)));
    f32 r_perpendicular = (((eta_i*cos_theta_i) - (eta_t*cos_theta_t)) /
                           ((eta_i*cos_theta_i) + (eta_t*cos_theta_t)));

    f32 result = 0.5f * (r_parallel * r_parallel + r_perpendicular * r_perpendicular);
    return result;
}

static force_inline V3
refract(const V3& D, const V3& N, f32 cos_theta_i, f32 cos_theta_t, f32 eta_i_over_eta_t) {
    V3 result = eta_i_over_eta_t*D + N*(eta_i_over_eta_t*cos_theta_i - cos_theta_t);
    return result;
}

static force_inline Ray
make_reflected_ray(const Ray& in_ray, V3 hit_p, V3 N) {
    V3 reflected_d = reflect(in_ray.d, N);
    return make_ray(hit_p + reflected_d*EPSILON, reflected_d);
}

static force_inline V3
sample_sky(Scene* scene, const Ray& ray) {
    V3 result = {};

    Image_V3* skydome = scene->skydome;
    if (skydome) {
        f32 rcp_pi  = 1.0f / PI_32;
        f32 rcp_2pi = 0.5f / PI_32;
        f32 phi   = atan2f(ray.d.z, ray.d.x);
        f32 theta = asinf(ray.d.y);
        f32 u = 0.5f + rcp_2pi*phi;
        f32 v = 0.5f + rcp_pi*theta;

        s32 skydome_x = (s32)(u*(f32)skydome->w) % skydome->w;
        s32 skydome_y = (s32)(v*(f32)skydome->h) % skydome->h;

        result = skydome->pixels[skydome_y*skydome->w + skydome_x];
    } else {
        f32 sky_t = absolute_value(ray.d.y);
        result = lerp(scene->bot_sky_color, scene->top_sky_color, sky_t);
    }

    return result;
}

static force_inline V3
evaluate_material(Material* material, const V3& hit_p) {
    V3 result = material->albedo;
    if (material->flags & Material_Checkers) {
        s32 checker = (((s32)floorf(0.25f*hit_p.x))^
                       ((s32)floorf(0.25f*hit_p.z))) & 1;
        if (checker) {
            result = material->checker_color;
        }
    }
    return result;
}

static V3
raytrace_recursively(Scene* scene, Sampler* sampler, const Ray& ray, Arena* task_arena, s32 recursion, Material* previous_material = nullptr) {
    if (recursion > 0) {
        f32 t;
        V3 I, N;
        Primitive* primitive = intersect_scene(scene, ray, &t, &I, &N);

        if (primitive) {
            Material* material = &scene->materials[primitive->material_id];

            if (material->flags & Material_Emissive) {
                return material->emission_color;
            }

            f32 cos_theta_i = -dot(ray.d, N);
            b32 ray_hit_inside = (cos_theta_i < 0.0f);

            f32 eta_i = 1.0f;
            f32 eta_t = material->ior;

            V3 throughput = v3(1.0f);

            if (ray_hit_inside) {
                N = -N;
                cos_theta_i = -cos_theta_i;
                Swap(eta_i, eta_t);
                if (previous_material) {
                    material = previous_material;
                }
            }

            if (ray_hit_inside && material->is_participating_medium) {
                throughput.x *= expf(-material->medium.absorb.x*t);
                throughput.y *= expf(-material->medium.absorb.y*t);
                throughput.z *= expf(-material->medium.absorb.z*t);
            }

            const usize light_sample_count = 1;

            V3 illumination = {};
            for (usize light_index = 0; light_index < scene->lights.count; ++light_index) {
                PrimitiveID light_id       = scene->lights[light_index];
                Primitive*  light          = &scene->primitives[light_id];
                Material*   light_material = &scene->materials[light->material_id];
                for (usize i = 0; i < light_sample_count; ++i) {
                    LightSample ls = random_point_on_light(light, get_next_sample_2d(sampler, Sample_DirectLighting, 0), I);

                    V3 L  = ls.L;
                    V3 Nl = ls.Nl;

                    f32 N_dot_L      =  dot(N, L);
                    f32 neg_Nl_dot_L = -dot(Nl, L);

                    if (N_dot_L > 0.0f && neg_Nl_dot_L > 0.0f) {
                        if (!intersect_shadow_ray(scene, make_ray(I + L*EPSILON, L, ls.dist - 2*EPSILON), light_id)) {
                            illumination += neg_Nl_dot_L*ls.A*N_dot_L*light_material->emission_color / ls.dist_sq;
                        }
                    }
                }
            }
            illumination *= 1.0f / (f32)light_sample_count;
            illumination += scene->ambient_light;

            V3 brdf = (1.0f / PI_32)*evaluate_material(material, I);
            V3 metallic_color = lerp(v3(1.0f), material->albedo, material->metallic);

            f32 eta_i_over_eta_t = eta_i / eta_t;

            f32 cos_theta_t;
            f32 reflectance = fresnel_dielectric(cos_theta_i, eta_i, eta_t, eta_i_over_eta_t, &cos_theta_t);

            reflectance = lerp(reflectance, 1.0f, material->metallic);

            if (material->is_participating_medium) {
                V3 refracted_d = refract(ray.d, N, cos_theta_i, cos_theta_t, eta_i_over_eta_t);

                V3 reflected_d = reflect(ray.d, N);
                if (material->roughness > 0.0f) {
                    reflected_d = normalize((1.0f + EPSILON)*reflected_d + material->roughness*random_in_unit_sphere(sampler->entropy));
                }

                V3 refracted_light = raytrace_recursively(scene, sampler, make_ray(I + refracted_d*EPSILON, refracted_d), task_arena, recursion - 1, material);
                V3 reflected_light = raytrace_recursively(scene, sampler, make_ray(I + reflected_d*EPSILON, reflected_d), task_arena, recursion - 1);

                return lerp(throughput*refracted_light, reflected_light, reflectance);
            } else if (reflectance > 0.05f) {
                V3 reflected_d = reflect(ray.d, N);
                if (material->roughness > 0.0f) {
                    reflected_d = normalize((1.0f + EPSILON)*reflected_d + material->roughness*random_in_unit_sphere(sampler->entropy));
                }

                V3 diffuse_light   = throughput*brdf*illumination;
                V3 reflected_light = metallic_color*raytrace_recursively(scene, sampler, make_ray(I + reflected_d*EPSILON, reflected_d), task_arena, recursion - 1);

                return lerp(diffuse_light, reflected_light, reflectance);
            } else {
                return throughput*brdf*illumination;
            }
        }

        return sample_sky(scene, ray);
    }

    return v3(0.0f);
}

static
INTEGRATOR(whitted_integrator) {
    Scene* scene           = state->scene;
    Arena* task_arena      = state->task_arena;
    RandomSeries* entropy  = state->entropy;
    V3 in_ray_o            = state->in_ray_o;
    V3 in_ray_d            = state->in_ray_d;

    Ray ray = make_ray(in_ray_o, in_ray_d);
    return raytrace_recursively(scene, &state->sampler, ray, task_arena, scene->settings.max_bounce_count);
}

static V3
pathtrace_recursively(Scene* scene, RandomSeries* entropy, const Ray& ray, Arena* task_arena, s32 recursion) {
    if (recursion > 0) {
        f32 t;
        V3 I, N;
        Primitive* primitive = intersect_scene(scene, ray, &t, &I, &N);

        if (primitive) {
            Material* material = &scene->materials[primitive->material_id];

            if (material->flags & Material_Emissive) {
                return material->emission_color;
            }

            V4 r = random_unilaterals(entropy);

            f32 eta_i = 1.0f;
            f32 eta_t = material->ior;
            f32 eta_i_over_eta_t = eta_i / eta_t;

            f32 cos_theta_i = -dot(ray.d, N);
            f32 cos_theta_t;
            f32 reflectance = fresnel_dielectric(cos_theta_i, eta_i, eta_t, eta_i_over_eta_t, &cos_theta_t);
            f32 reflect_test = r.x;
            if (reflect_test < reflectance) {
                return pathtrace_recursively(scene, entropy, make_reflected_ray(ray, I, N), task_arena, recursion - 1);
            }

            V3 brdf = evaluate_material(material, I) * (1.0f / PI_32);

            const usize light_sample_count = 1;

            V3 R = map_to_hemisphere(N, r.yz);
            Ray diffuse_ray = make_ray(I + N*EPSILON, R);

            V3 light = pathtrace_recursively(scene, entropy, diffuse_ray, task_arena, recursion - 1);
            light *= max(0.0f, dot(N, diffuse_ray.d));

            return 2.0f*PI_32*light*brdf;
        }
    }

    return sample_sky(scene, ray);
}

static
INTEGRATOR(ground_truth_recursive_integrator) {
    Scene* scene          = state->scene;
    Arena* task_arena     = state->task_arena;
    RandomSeries* entropy = state->entropy;
    V3 in_ray_o           = state->in_ray_o;
    V3 in_ray_d           = state->in_ray_d;

    Ray ray = make_ray(in_ray_o, in_ray_d);
    return pathtrace_recursively(scene, entropy, ray, task_arena, scene->settings.max_bounce_count);
}

static
INTEGRATOR(ground_truth_iterative_integrator) {
    Scene* scene           = state->scene;
    Arena* task_arena      = state->task_arena;
    RandomSeries* entropy  = state->entropy;
    V3 in_ray_o            = state->in_ray_o;
    V3 in_ray_d            = state->in_ray_d;

    Ray ray = make_ray(in_ray_o, in_ray_d);

    V3 throughput  = v3(1.0f);
    V3 total_color = v3(0.0f);

    for (usize bounce_index = 0; bounce_index < scene->settings.max_bounce_count; ++bounce_index) {
        f32 t;
        V3 I, N;
        Primitive* primitive = intersect_scene(scene, ray, &t, &I, &N);

        if (primitive) {
            Material* material = &scene->materials[primitive->material_id];

            V3 light = {};
            if (material->flags & Material_Emissive) {
                total_color += throughput*material->emission_color;
                break;
            }

            V4 r = random_unilaterals(entropy);

            f32 eta_i = 1.0f;
            f32 eta_t = material->ior;
            f32 eta_i_over_eta_t = eta_i / eta_t;

            f32 cos_theta_i = -dot(ray.d, N);
            f32 cos_theta_t;
            f32 reflectance = fresnel_dielectric(cos_theta_i, eta_i, eta_t, eta_i_over_eta_t, &cos_theta_t);
            f32 reflect_test = r.x;
            if (reflect_test < reflectance) {
                ray = make_reflected_ray(ray, I, N);
            } else {
                V3 brdf = evaluate_material(material, I) * (1.0f / PI_32);
                throughput *= brdf;

                V3 R = map_to_hemisphere(N, r.yz);
                ray = make_ray(I + N*EPSILON, R);
                throughput *= dot(ray.d, N);

                throughput *= 2.0f*PI_32;
            }
        } else {
            total_color += throughput*sample_sky(scene, ray);
            break;
        }
    }

    return total_color;
}

static
INTEGRATOR(normals_integrator) {
    Scene* scene = state->scene;
    V3 in_ray_o  = state->in_ray_o;
    V3 in_ray_d  = state->in_ray_d;

    Ray ray = make_ray(in_ray_o, in_ray_d);

    f32 t;
    V3 I, N;
    Primitive* primitive = intersect_scene(scene, ray, &t, &I, &N);

    if (primitive) {
        return 0.5f*(v3(1.0f) + N);
    } else {
        return sample_sky(scene, ray);
    }
}

static
INTEGRATOR(distances_integrator) {
    Scene* scene = state->scene;
    V3 in_ray_o  = state->in_ray_o;
    V3 in_ray_d  = state->in_ray_d;

    Ray ray = make_ray(in_ray_o, in_ray_d);

    f32 t;
    V3 I, N;
    Primitive* primitive = intersect_scene(scene, ray, &t, &I, &N);

    if (primitive) {
        return v3(1.0f - saturate(t / 15.0f));
    } else {
        return sample_sky(scene, ray);
    }
}

static
INTEGRATOR(advanced_integrator) {
    Scene* scene          = state->scene;
    Arena* task_arena     = state->task_arena;
    RandomSeries* entropy = state->entropy;
    Sampler* sampler      = &state->sampler;
    V3 in_ray_o           = state->in_ray_o;
    V3 in_ray_d           = state->in_ray_d;
    u32 x                 = state->x;
    u32 y                 = state->y;

    Ray ray = make_ray(in_ray_o, in_ray_d);

    V3 total_color = v3(0.0f);
    V3 throughput  = v3(1.0f);

    Material air = {};
    air.ior = 1.0f;
    air.is_participating_medium = true;

    ssize material_stack_at = 0;
    Material* material_stack[64] = { &air };

    // TODO: Use this to log an warning of sorts.
    b32 material_stack_overrun = false;

    b32 is_specular_bounce = true;

    V3 prev_N = {};

    u32 max_bounce_count = scene->settings.max_bounce_count;
    for (usize bounce_index = 0; bounce_index < max_bounce_count; ++bounce_index) {
        f32 t;
        V3 I, N;
        Primitive* primitive = intersect_scene(scene, ray, &t, &I, &N);

        if (primitive) {
            f32 cos_theta_i = -dot(ray.d, N);
            b32 ray_hit_inside_surface = (cos_theta_i < 0.0f);

			Material* surface_material = &scene->materials[primitive->material_id];

            // NOTE: Following the convention from the mathematics describing refraction
            //       material_i denotes the outer material, and material_t the inner
            //       (surface) material.
            Material* material_i;
            Material* material_t;

            if (ray_hit_inside_surface) {
                material_i = surface_material;
                material_t = material_stack[Max(0, material_stack_at - 1)];

                cos_theta_i = -cos_theta_i;
                N = -N;
            } else {
                material_i = material_stack[material_stack_at];
                material_t = surface_material;
            }

            if (material_i->is_participating_medium) {
                Medium* medium = &material_i->medium;

                // NOTE: Beer's law
                V3 absorption = v3(expf(-medium->absorb.x*t),
                                   expf(-medium->absorb.y*t),
                                   expf(-medium->absorb.z*t));

                throughput *= absorption;
            }

            if (material_t->flags & Material_Emissive) {
                // NOTE: This slightly bizarre conditional deals with the fact that when
                //       we're using next event estimation, we must reject lights from
                //       our indirect lighting. However, if we still want caustics, we
                //       do want to accept indirect light from specular paths!
                b32 allow_direct_lighting = (!scene->settings.next_event_estimation ||
                                             ((scene->settings.caustics || (bounce_index < 2)) && is_specular_bounce));
                if (allow_direct_lighting) {
                    total_color += throughput*material_t->emission_color;
                } else if (bounce_index > 0 && scene->settings.use_mis) {
                    Primitive* light          = primitive;
                    Material*  light_material = &scene->materials[light->material_id];
                    f32 light_distance_sq = t*t;
                    f32 light_pdf = light_distance_sq / cos_theta_i;
                    f32 brdf_pdf  = (scene->settings.importance_sample_diffuse ? dot(prev_N, ray.d) / PI_32
                                                                               : 1.0f / (2.0f*PI_32));
                    f32 mis_pdf = light_pdf + brdf_pdf;
                    total_color += (1.0f / mis_pdf)*throughput*material_t->emission_color;
                }
                break;
            } else {
                f32 eta_i = material_i->ior;
                f32 eta_t = material_t->ior;
                f32 eta_i_over_eta_t = eta_i / eta_t;

                f32 cos_theta_t;
                f32 reflectance  = fresnel_dielectric(cos_theta_i, eta_i, eta_t, eta_i_over_eta_t, &cos_theta_t);
                f32 reflect_test = get_next_sample_1d(sampler, Sample_Reflectance, bounce_index);

                reflectance = lerp(reflectance, 1.0f, material_t->metallic);

                is_specular_bounce = true;

                if (reflect_test < reflectance) {
                    //
                    // NOTE: Reflect
                    //

                    V3 reflected_d = reflect(ray.d, N);

                    if (material_t->roughness > 0.0f) {
                        reflected_d = normalize((1.0f + EPSILON)*reflected_d + material_t->roughness*random_in_unit_sphere(entropy));
                    }

                    ray = make_ray(I + EPSILON*reflected_d, reflected_d);
                    throughput *= lerp(v3(1.0f), material_t->albedo, material_t->metallic);
                } else {
                    if (material_t->is_participating_medium) {
                        //
                        // NOTE: Refract
                        //

                        if (ray_hit_inside_surface) {
                            if (material_stack_at > 0) {
                                --material_stack_at;
                            }
                        } else {
                            if (material_stack_at < (ArrayCount(material_stack) - 1)) {
                                ++material_stack_at;
                                material_stack[material_stack_at] = material_t;
                            } else {
                                material_stack_overrun = true;
                            }
                        }

                        V3 refracted_d = refract(ray.d, N, cos_theta_i, cos_theta_t, eta_i_over_eta_t);
                        ray = make_ray(I + refracted_d*EPSILON, refracted_d);
                    } else {
                        is_specular_bounce = false;

                        //
                        // NOTE: Evaluate material
                        //

						V3 albedo = evaluate_material(material_t, I);

                        //
                        // NOTE: Evaluate BRDF
                        //

                        // NOTE: Lambertian BRDF
                        V3 brdf = (1.0f / PI_32)*albedo;

                        //
                        // NOTE: Next Event Estimation
                        //

                        if (scene->settings.next_event_estimation && (scene->lights.count > 0)) {
                            // REFERENCE: https://jacco.ompf2.com/2019/12/11/probability-theory-for-physically-based-rendering/
                            // REFERENCE: Jacco's Utrecht Lecture - "Variance Reduction"
                            f32 light_pick_sample = get_next_sample_1d(sampler, Sample_LightSelection, bounce_index);
                            f32 light_pick_rcp_pdf;
                            PrimitiveID light_id = pick_random_light(scene, light_pick_sample, I, task_arena, &light_pick_rcp_pdf);
                            Primitive*  light    = &scene->primitives[light_id];
                            Material*   light_material = &scene->materials[light->material_id];

                            V2 sample = get_next_sample_2d(sampler, Sample_DirectLighting, bounce_index);
                            LightSample ls = random_point_on_light(light, sample, I);
                            V3 L  = ls.L;
                            V3 Nl = ls.Nl;

                            f32 N_dot_L      =  dot(N, L);
                            f32 neg_Nl_dot_L = -dot(Nl, L);

                            if (N_dot_L > 0.0f && neg_Nl_dot_L > 0.0f) {
                                if (!intersect_shadow_ray(scene, make_ray(I + L*EPSILON, L, ls.dist - 2*EPSILON), light_id)) {
                                    f32 solid_angle = (neg_Nl_dot_L * ls.A) / ls.dist_sq;
                                    f32 pdf;
                                    if (scene->settings.use_mis) {
                                        f32 light_pdf   = 1.0f / solid_angle;
                                        f32 brdf_pdf    = (scene->settings.importance_sample_diffuse ? N_dot_L / PI_32
                                                                                                     : 1.0f / (2.0f*PI_32));
                                        pdf = light_pdf + brdf_pdf;
                                    } else {
                                        pdf = 1.0f / solid_angle;
                                    }
                                    pdf *= light_pick_rcp_pdf;
                                    total_color += throughput*(dot(N, ls.L) / pdf)*brdf*light_material->emission_color;
                                }
                            }
                        }

                        //
                        // NOTE: Indirect Lighting
                        //

                        V2 sample = get_next_sample_2d(sampler, Sample_IndirectLighting, bounce_index);

                        V3 R;
                        if (scene->settings.importance_sample_diffuse) {
                            R = map_to_cosine_weighted_hemisphere(N, sample);
                            throughput *= PI_32;
                        } else {
                            R = map_to_hemisphere(N, sample);
                            throughput *= 2.0f*PI_32*dot(N, R);
                        }
                        throughput *= brdf;

                        ray = make_ray(I + N * EPSILON, R);
                    }
                }
            }

            //
            // NOTE: Russian roulette
            // REFERENCE: https://github.com/RichieSams/lantern/blob/master/source/core/integrator/integrator.cpp#L234
            // REFERENCE: https://cseweb.ucsd.edu/classes/sp17/cse168-a/CSE168_13_PathTracing2.pdf
            // REFERENCE: Jacco's Advanced Lecture - "Various"
            //

            if (scene->settings.russian_roulette) {
                // NOTE: Caustics already need all the help they can get, so I exclude them from the roulette.
                if (!is_specular_bounce) {
                    f32 p = clamp(max3(throughput), 0.1f, 0.9f);
                    f32 e = get_next_sample_1d(sampler, Sample_Roulette, bounce_index);
                    if (e > p) {
                        break;
                    }
                    throughput *= 1.0f / p;
                }
            }
        } else {
            total_color += throughput*sample_sky(scene, ray);
            break;
        }

        prev_N = N;
    }

    return total_color;
}

IntegratorOption g_integrators[] = {
    { "Advanced Pathtracer",    advanced_integrator },
    { "Whitted",                whitted_integrator },
    { "Ground Truth Recursive", ground_truth_recursive_integrator },
    { "Ground Truth Iterative", ground_truth_iterative_integrator },
    { "Normals",                normals_integrator },
    { "Distances",              distances_integrator },
};

usize g_integrator_count = ArrayCount(g_integrators);

IntegratorOption*
find_integrator(const char* name) {
    IntegratorOption* result = &g_integrators[0]; // NOTE: We return the default integrator if not found
    for (usize i = 0; i < g_integrator_count; ++i) {
        IntegratorOption* option = &g_integrators[i];
        if (0 == strcmp(name, option->name)) {
            result = option;
            break;
        }
    }
    return result;
}
