//
// Raytracer.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

// NOTE: Credit to Oliver Cadel for DragonLowPoly.obj!!

#include "precomp.h"

#include "common.h"
#include "primitives.h"
#include "assets.h"
#include "bvh.h"
#include "reconstruction_filters.h"
#include "samplers.h"
#include "scene.h"
#include "intersection.h"
#include "integrators.h"
#include "raytracer.h"

#include "about_strings.h"

//
// NOTE: Camera
//

static void
aim_camera(Camera* camera, V3 camera_d) {
    camera->z = noz(camera_d);
    camera->x = noz(cross(v3(0, 1, 0), camera->z));
    camera->y = noz(cross(camera->z, camera->x));

    f32 film_w = camera->aspect_ratio;
    f32 film_h = 1.0f;
    camera->half_film_w = 0.5f*film_w;
    camera->half_film_h = 0.5f*film_h;

    f32 film_distance = film_h / tanf(camera->vfov);

    camera->film_distance = film_distance;
}

static void
aim_camera_at(Camera* camera, V3 at) {
    V3 camera_vector = at - camera->p;
    V3 camera_d = normalize(camera_vector);
    aim_camera(camera, -camera_d);
    camera->focus_distance = length(camera_vector);
}

static void
recompute_camera(Camera* camera) {
    f32 film_w = camera->aspect_ratio;
    f32 film_h = 1.0f;

    camera->half_film_w = 0.5f*film_w;
    camera->half_film_h = 0.5f*film_h;

    camera->film_distance = film_h / tanf(camera->vfov);
}

static force_inline V3
get_ray_d(Camera* camera, f32 u, f32 v) {
    V3 film_center = -camera->film_distance*camera->z;
	V3 film_p = film_center + u*camera->half_film_w*camera->x + v*camera->half_film_h*camera->y;
	V3 result = noz(film_p);
    return result;
}

inline f32
sigmoidal_contrast(f32 x, f32 contrast, f32 midpoint) {
    // SOURCE: I am the source: https://www.shadertoy.com/view/3ssSz2

    f32 curve;
    if (x < midpoint) {
        f32 scale = (1.0f / midpoint)*x;
        curve = midpoint*(scale*scale);
    } else {
        f32 y = (1.0f / (1.0f - midpoint));
        f32 scale =  y - y*x;
        curve = 1.0f - (1.0f - midpoint)*(scale*scale);
    }

    return lerp(x, curve, contrast);
}

inline V2
transform_bokeh_sample(V2 o, f32 f, f32 n, f32 phi_shutter_max) {
    // SOURCE: https://www.shadertoy.com/view/MtlGRn
    V2 ab = (o * 2.0f) - v2(1.0f);    
    V2 phir = ((ab.x * ab.x) > (ab.y * ab.y)) ? v2((absolute_value(ab.x) > 1e-8f) ? ((PI_32 * 0.25f) * (ab.y / ab.x)) : 0.0f, ab.x) : v2((absolute_value(ab.y) > 1e-8f) ? ((PI_32 * 0.5f) - ((PI_32 * 0.25f) * (ab.x / ab.y))) : 0.0f, ab.y); 
    phir.x += f * phi_shutter_max;
   	phir.y *= (f > 0.0f) ? powf((cosf(PI_32 / n) / cosf(phir.x - ((2.0f * (PI_32 / n)) * floor(((n * phir.x) + PI_32) / (2.0f * PI_32))))), f) : 1.0f;
    return v2(cosf(phir.x), sinf(phir.x)) * phir.y;
}

inline V2
brown_conrady_distortion(V2 uv, f32 amount, f32 width_over_height) {
    uv.y /= width_over_height;
    V2 barrel_distortion_1 =    0.1f*v2(amount);
    V2 barrel_distortion_2 = -0.025f*v2(amount);

    f32 r2 = dot(uv, uv);
    uv *= v2(1.0f) + r2*barrel_distortion_1 + r2*r2*barrel_distortion_2;

    uv.y *= width_over_height;
    return uv;
}

inline void
apply_lens_distortion(f32 amount, u32 w, u32 h, f32* out_u, f32* out_v) {
    V2 min_uv_scale = brown_conrady_distortion(v2(0, 0), amount, (f32)w / (f32)h);
    V2 max_uv_scale = brown_conrady_distortion(v2(1, 1), amount, (f32)w / (f32)h);

    V2 uv = v2(*out_u, *out_v);
    uv = brown_conrady_distortion(uv, amount, (f32)w / (f32)h);

    if (amount > 0.0f) {
        uv = (uv - min_uv_scale) / (min_uv_scale + max_uv_scale);
    }

    *out_u = uv.x;
    *out_v = uv.y;
}

static f32
remap_tpdf(f32 x) {
    f32 orig = 2.0f*x - 1.0f;
    x = orig*fast_approx_inverse_square_root(absolute_value(orig));
    x = max(-1.0f, x);
    x = x - sign_of(x);
    return x;
}

static char*
aprintf(Arena* arena, const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);

    int len = vsnprintf(0, 0, fmt, args);
    char* result = push_array(arena, len + 1, char);
    vsnprintf(result, (usize)len + 1, fmt, args);
    
    va_end(args);

    return result;
}

static Mesh
load_mesh(Scene* scene, const char* file_name, Arena* temp_arena) {
    Mesh mesh = {};

    char* model_file = platform_read_entire_file(temp_arena, "data/dragon_mcguire.obj");
    if (parse_obj(&scene->arena, temp_arena, model_file, &mesh, MeshWinding_CounterClockwise)) {
        mesh.bvh = create_bvh_for_mesh(mesh.triangle_count, mesh.triangles, &scene->arena, temp_arena, BVH_MidpointSplit, BVHStorage_Scalar);
    }
    
    return mesh;
}

//
// NOTE: Filter Caching
//

inline void
load_reconstruction_kernel(FilterKernelOption* kernel) {
    const usize lookup_size = 256;

    assert(kernel);

    FilterCache* cache = &g_filter_cache;
    zero_struct(cache);

    cache->filter = *kernel;

    if (kernel->f) {
        zero_array(ArrayCount(cache->cache), cache->cache);

        cache->kernel_size = kernel->radius;
        cache->cache_size = lookup_size;
        for (usize i = 0; i < lookup_size; ++i) {
            f32 x = ((f32)kernel->radius*(f32)i) / (f32)(lookup_size - 1);
            cache->cache[i] = kernel->f(x);
        }
    }
}

static void
splat_filter(Arena* arena, FilterCache* cache, AccumulationBuffer* buffer, ssize x, ssize y, f32 jitter_x, f32 jitter_y, V3 sample) {
    ssize w = buffer->w;
    ssize h = buffer->h;
    V4* pixels = buffer->pixels;

    ssize kernel_size        = cache->kernel_size;
    ssize kernel_lookup_size = cache->cache_size;
    f32*  kernel_lookup      = cache->cache;
    f32   kernel_scale       = (f32)(kernel_lookup_size - 1) / (f32)kernel_size;

    ScopedMemory scoped_memory(arena);

    ssize kernel_span = 2*kernel_size + 1;
    f32* lookup_x = push_array(arena, 2*kernel_span, f32, no_clear());
    f32* lookup_y = lookup_x + kernel_span;

    for (ssize i = 0; i < kernel_span; ++i) {
        s32 j = (s32)absolute_value(0.5f + kernel_scale*((f32)(i - kernel_size) - jitter_x));
        lookup_x[i] = kernel_lookup[j];
    }

    for (ssize i = 0; i < kernel_span; ++i) {
        s32 j = (s32)absolute_value(0.5f + kernel_scale*((f32)(i - kernel_size) - jitter_y));
        lookup_y[i] = kernel_lookup[j];
    }

    ssize x_margin = 0;
    ssize y_margin = 0;

    ssize splat_min_x          = x - kernel_size;
    ssize splat_one_past_max_x = x + kernel_size + 1;
    ssize splat_min_y          = y - kernel_size;
    ssize splat_one_past_max_y = y + kernel_size + 1;

    if (splat_min_x < 0) {
        x_margin = -splat_min_x;
        splat_min_x = 0;
    }

    if (splat_min_y < 0) {
        y_margin = -splat_min_y;
        splat_min_y = 0;
    }

    if (splat_one_past_max_x > w) {
        splat_one_past_max_x = w;
    }

    if (splat_one_past_max_y > h) {
        splat_one_past_max_y = h;
    }

    V4* dst_row = &pixels[splat_min_y*w + splat_min_x];

    f32* filter_y_at = &lookup_y[y_margin];
    for (ssize splat_y = splat_min_y; splat_y < splat_one_past_max_y; ++splat_y) {
        f32 filter_y = *filter_y_at++;

        V4* dst = dst_row;
        f32* filter_x_at = &lookup_x[x_margin];
        for (ssize splat_x = splat_min_x; splat_x < splat_one_past_max_x; ++splat_x) {
            f32 filter_x = *filter_x_at++;

            f32 filter = filter_x*filter_y;

            dst->xyz += filter*sample;
            dst->w   += filter;
            ++dst;
        }
        dst_row += w;
    }
}

static void
splat_filter_optimized(Arena* arena, FilterCache* cache, V4* pixels, ssize w, ssize h, ssize x, ssize y, f32 jitter_x, f32 jitter_y, V3 sample) {
    ssize kernel_size        = cache->kernel_size;
    ssize kernel_lookup_size = cache->cache_size;
    f32*  kernel_lookup      = cache->cache;
    f32   kernel_scale       = (f32)(kernel_lookup_size - 1) / (f32)kernel_size;

    ScopedMemory scoped_memory(arena);

    if ((x - kernel_size < 0)  ||
        (y - kernel_size < 0)  ||
        (x + kernel_size >= w) ||
        (y + kernel_size >= h))
    {
        return;
    }

    ssize kernel_span = 2*kernel_size + 1;
    f32* lookup = push_array(arena, Align16(kernel_span*kernel_span), f32, no_clear(16));

    for (ssize j = 0; j < kernel_span; ++j) {
        ssize iy = (ssize)absolute_value(0.5f + kernel_scale*((f32)(j - kernel_size) - jitter_y));
        f32 filter_y = kernel_lookup[iy];
        for (ssize i = 0; i < kernel_span; ++i) {
            ssize ix = (ssize)absolute_value(0.5f + kernel_scale*((f32)(i - kernel_size) - jitter_x));
            f32 filter_x = kernel_lookup[ix];
            lookup[j*kernel_span + i] = filter_x*filter_y;
        }
    }

#if 0
    for (ssize j = 0; j < kernel_span; ++j) {
        for (ssize i = 0; i < kernel_span; ++i) {
            fprintf(stderr, "%+f,", lookup[j*kernel_span + i]);
        }
        fprintf(stderr, "\n");
    }
    fprintf(stderr, "\n");
#endif

    ssize splat_min_x = Max(0,     x - kernel_size);
    ssize splat_max_x = Min(w - 1, x + kernel_size);
    ssize splat_min_y = Max(0,     y - kernel_size);
    ssize splat_max_y = Min(h - 1, y + kernel_size);

    ssize splat_w = 1 + splat_max_x - splat_min_x;
    ssize splat_h = 1 + splat_max_y - splat_min_y;

    ssize domain_size = splat_w*splat_h;
    V4* domain = push_array(arena, Align16(domain_size), V4, no_clear(16));

    V4* starting_pixels = &pixels[splat_min_y*w + splat_min_x];
    {
        V4* dst = domain;
        V4* src = starting_pixels;
        for (ssize i = 0; i < splat_h; ++i) {
            copy_array(splat_w, src, dst);
            src += w;
            dst += splat_w;
        }
    }

    V4_4x sample_4x = V4_4x::transposed(v4(sample, 1.0f));

    f32_4x* lookup_4x = (f32_4x*)lookup;
    V4_4x*  domain_4x =  (V4_4x*)domain;
    for (ssize i = 0; i < (domain_size / 4); ++i) {
        f32_4x filter = lookup_4x[i];
        V4_4x orig = domain_4x[i];
        V4_4x modi = filter*sample_4x;
        domain_4x[i] = orig + modi;
    }

    {
        V4* src = domain;
        V4* dst = starting_pixels;
        for (ssize splat_y = 0; splat_y < splat_h; ++splat_y) {
            copy_array(splat_w, src, dst);
            src += splat_w;
            dst += w;
        }
    }
}

static void
debug_visualize_filter_cache(FilterCache* cache, AccumulationBuffer* buffer, Arena* temp_arena) {
    ssize w = buffer->w;
    ssize h = buffer->h;
    V4* pixels = buffer->pixels;

    if (!cache->cache_size) {
        return;
    }

    for (ssize y = 0; y < 256; ++y)
    for (ssize x = 0; x < 256; ++x) {
        pixels[y*w + x] = {};
    }

    for (ssize y = 0; y < 256; y += 32)
    for (ssize x = 0; x < 256; x += 32) {
        splat_filter(temp_arena, cache, buffer, x, y, 0.0f, 0.0f, v3(1.0f));
    }
}

static void
render_tile(WorkQueue* queue,
            RandomSeries entropy,
            s32 min_x, s32 min_y, s32 one_past_max_x, s32 one_past_max_y,
            Arena* task_arena)
{
    Scene* scene = queue->parameters.scene;
    AccumulationBuffer* accumulation_buffer = queue->parameters.backbuffer;

    u32 w = accumulation_buffer->w;
    u32 h = accumulation_buffer->h;
    V4* pixels = accumulation_buffer->pixels;

    u32 samples_per_pixel = scene->settings.samples_per_pixel;

    Camera* camera = &scene->camera;
    V3 camera_p = camera->p;
    V3 camera_z = camera->z;
    V3 camera_x = camera->x;
    V3 camera_y = camera->y;

    f32 focus_distance = camera->focus_distance;
	f32 lens_radius    = camera->lens_radius;

    f32 half_film_w = camera->half_film_w;
    f32 half_film_h = camera->half_film_h;

    // NOTE: Since we're moving the "film" forward by the focus distance, we also want to scale by it.
    half_film_w *= focus_distance;
    half_film_h *= focus_distance;

    f32 film_distance = focus_distance*camera->film_distance;
    V3  film_center   = (camera_p - film_distance*camera_z);

    f32 pixel_w = 1.0f / w;
    f32 pixel_h = 1.0f / h;

    IntegratorState state = {};
    state.scene = scene;
    state.task_arena = task_arena;
    state.entropy = &entropy;

    V4* row = pixels + (usize)min_y*w + min_x;
    for (ssize y = min_y; y < one_past_max_y; ++y) {
        f32 v_ = 1.0f - 2.0f*(f32)y*pixel_h;

        V4* column = row;
        for (ssize x = min_x; x < one_past_max_x; ++x) {
            f32 u_ = 1.0f - 2.0f*(f32)x*pixel_w;

            f32 u = u_, v = v_;
            apply_lens_distortion(scene->settings.lens_distortion, w, h, &u, &v);

            state.x = (u32)x;
            state.y = (u32)y;

            for (usize sample_index = 0; sample_index < samples_per_pixel; ++sample_index) {
                if (queue->discard_render) {
                    return;
                }

                u32 frame_index = accumulation_buffer->frame_count;
                u32 canonical_sample_index = frame_index + (u32)sample_index;

                {
                    Sampler sampler = {};
                    sampler.entropy = &entropy;
                    sampler.sample_index = canonical_sample_index;
                    sampler.strategy = scene->settings.sampling_strategy;
                    sampler.x = (u32)x;
                    sampler.y = (u32)y;

                    state.sampler = sampler;
                }

                Sampler* sampler = &state.sampler;

                // NOTE: Ray setup
                V2 aa_jitter = get_next_sample_2d(sampler, Sample_AA, 0) - v2(0.5f);
                f32 jitter_x = aa_jitter.x;
                f32 jitter_y = aa_jitter.y;

                V2 dof_jitter = get_next_sample_2d(sampler, Sample_DOF, 0);
                dof_jitter = transform_bokeh_sample(dof_jitter, scene->settings.f_factor, scene->settings.diaphragm_edges, PI_32*scene->settings.phi_shutter_max);
                f32 dof_jitter_x = half_film_w*pixel_w*lens_radius*dof_jitter.x;
                f32 dof_jitter_y = half_film_h*pixel_h*lens_radius*dof_jitter.y;

                V3 film_p = film_center;
                film_p += (u + pixel_w*jitter_x)*half_film_w*camera_x;
                film_p += (v + pixel_h*jitter_y)*half_film_h*camera_y;

                V3 jitter_camera_p = (camera_p + dof_jitter_x*camera_x + dof_jitter_y*camera_y);

                V3 ray_o = jitter_camera_p;
                V3 ray_d = normalize(film_p - jitter_camera_p);

                state.in_ray_o = ray_o;
                state.in_ray_d = ray_d;

                // NOTE: Trace
                state.out_first_bounce_was_diffuse = false;
                V3 result = scene->settings.integrator->f(&state);

                // NOTE: Natural vignetting
                f32 vignette = dot(ray_d, camera_z);
                vignette = vignette*vignette*vignette*vignette;

                vignette = lerp(1.0f, vignette, scene->settings.vignette_strength);
                result *= vignette;

                FilterCache* cache = &g_filter_cache;
                // NOTE: Reconstruction
                if (cache->cache_size) {
#if 1
                    splat_filter(task_arena, cache, accumulation_buffer, x, y, jitter_x, jitter_y, result);
#else
                    // NOTE: This one crashes, don't enable it. The SIMD does not like me, need to continue work to see why.
                    splat_filter_optimized(task_arena, cache, pixels, w, h, x, y, jitter_x, jitter_y, result);
#endif
                } else {
                    column->xyz += result;
                    column->w   += 1.0f;
                }
            }

            ++column;
        }
        row += w;
    }
}

//
// NOTE: Accumulation Buffer
//

static AccumulationBuffer*
allocate_accumulation_buffer(Arena* arena, u32 w, u32 h) {
    AccumulationBuffer* result = push_struct(arena, AccumulationBuffer);
    result->w = w;
    result->h = h;
    result->pixels = push_array(arena, w*h, V4);

    return result;
}

static void
reset(AccumulationBuffer* buf) {
    zero_array(buf->w*buf->h, buf->pixels);
    buf->frame_count = 0;
}

static void
copy(AccumulationBuffer* src, AccumulationBuffer* dst) {
    assert((src->w == dst->w) && (src->h == dst->h));
    dst->frame_count = src->frame_count;
    memcpy(dst->pixels, src->pixels, sizeof(*src->pixels)*src->w*src->h);
}

//
// NOTE: Path Guider
//

static ScreenspacePathGuide*
allocate_path_guide(Arena* arena, u32 w, u32 h) {
    ScreenspacePathGuide* result = push_struct(arena, ScreenspacePathGuide);
    result->w = (w + result->grid_size - 1) / result->grid_size;
    result->h = (h + result->grid_size - 1) / result->grid_size;
    result->data = push_array(arena, (usize)w*h, StratifiedDisk, no_clear());
    return result;
}

static void
reset(ScreenspacePathGuide* guide) {
    for (usize i = 0; i < (usize)guide->w*guide->h; ++i) {
        StratifiedDisk* disk = &guide->data[i];
        for (usize j = 0; j < ArrayCount(disk->all_strata); ++j) {
            disk->all_strata[j] = EPSILON;
        }
    }
}

//
// NOTE: WorkQueue
//

static b32
try_render_next_tile(WorkQueue* queue, Arena* task_arena) {
    b32 result = false;

    s32 tile_index = atomic_add(&queue->tile_index, -1);
    if (tile_index >= 0) {
        result = true;

        if (queue->discard_render) {
            // NOTE: If we want to discard the current render, we just start running up the clock on these tiles,
            //       so that complete_all_work() can be used to wait on the threads to finish.
            atomic_add(&queue->tiles_retired, 1);
        } else {
            Scene* scene = queue->parameters.scene;
            assert(scene);

            AccumulationBuffer* accumulation_buffer = queue->parameters.backbuffer;
            assert(accumulation_buffer);

            // NOTE: We clear the task arena each time, since each task the thread does is unrelated
            //       to the last, there is no retention here.
            task_arena->clear();

            u32 w = accumulation_buffer->w;
            u32 h = accumulation_buffer->h;

            u32 tile_w = queue->tile_w;
            u32 tile_h = queue->tile_h;
            u32 tile_count_x = queue->tile_count_x;
            u32 tile_count_y = queue->tile_count_y;

            u32 min_x = tile_w*((u32)tile_index % tile_count_x);
            u32 min_y = tile_h*((u32)tile_index / tile_count_x);

            u32 one_past_max_x = Min(w, min_x + tile_w);
            u32 one_past_max_y = Min(h, min_y + tile_h);

            u32 rng_seed = hash_coordinate(scene->total_frame_index,
                                           accumulation_buffer->frame_count,
                                           (u32)tile_index);
            RandomSeries entropy = random_seed(rng_seed);

            render_tile(queue,
                        entropy,
                        min_x, min_y, one_past_max_x, one_past_max_y,
                        task_arena);

            atomic_add(&queue->tiles_retired, 1);
        }
    }

    return result;
}

struct ThreadParameters {
    WorkQueue* queue;
    Arena arena;
};

static void
thread_proc(void* userdata, PlatformSemaphore semaphore) {
	ThreadParameters* parameters = (ThreadParameters*)userdata;
    WorkQueue* queue = parameters->queue;
    Arena task_arena = parameters->arena;

    // NOTE: Signal to resume the calling thread now that we're done with the thread parameters
    platform_release_semaphore(semaphore);

    atomic_add(&queue->threads_in_flight, 1);
    for (;;) {
        if (!try_render_next_tile(queue, &task_arena)) {
            atomic_add(&queue->threads_in_flight, -1);
            platform_wait_on_semaphore(queue->semaphore);
            atomic_add(&queue->threads_in_flight, 1);
        }
    }
}

static void
init_work_queue(WorkQueue* queue,
                u32 thread_count,
                Arena* parent_arena,
                RenderParameters parameters,
                u32 tile_w, u32 tile_h)
{
    u32 task_align = 64;
    assert(parent_arena->get_size_remaining(task_align) >= ((usize)thread_count + 1)*THREAD_TASK_ARENA_SIZE);

    zero_struct(queue);

    queue->parameters = parameters;
    AccumulationBuffer* frontbuffer = parameters.frontbuffer;

    queue->tile_w = tile_w;
    queue->tile_h = tile_h;

    queue->tile_count_x = (frontbuffer->w + tile_w - 1) / tile_w;
    queue->tile_count_y = (frontbuffer->h + tile_h - 1) / tile_h;
    queue->total_tile_count = queue->tile_count_x*queue->tile_count_y;

    queue->semaphore = platform_create_semaphore(0, thread_count);
    queue->thread_count = thread_count;

    parent_arena->init_child_arena(&queue->main_thread_task_arena, THREAD_TASK_ARENA_SIZE, no_clear(task_align));

    for (usize thread_index = 0; thread_index < thread_count; ++thread_index) {
        Arena task_arena;
        parent_arena->init_child_arena(&task_arena, THREAD_TASK_ARENA_SIZE, no_clear(task_align));

        ThreadParameters thread_parameters = {};
        thread_parameters.queue = queue;
        thread_parameters.arena = task_arena;

        platform_create_thread(thread_proc, &thread_parameters);
    }

    queue->discard_render = true;
}

static void
complete_all_work(WorkQueue* queue) {
    // NOTE: Then, we go do some work on the main thread since it's sitting around
    //       lazily otherwise anyway, and waste a little bit of electrictiy on a
    //       spinlock if queue->tile_index has gone below 0 but other threads haven't
    //       yet finished all their work.
    while (queue->threads_in_flight) {
        s64 tiles_retired = queue->tiles_retired;
        if (tiles_retired < (s64)queue->total_tile_count) {
            try_render_next_tile(queue, &queue->main_thread_task_arena);
        } else {
            break;
        }
    }
}

static void
discard_current_render(WorkQueue* queue) {
    queue->discard_render = true;
    complete_all_work(queue);
}

static b32
render_all_tiles(WorkQueue* queue, f64* out_time_elapsed = nullptr, TraversalStats* out_stats = nullptr) {
    b32 result = false;

    RenderParameters* parameters = &queue->parameters;

    Scene* scene = parameters->scene;

    if (!structs_are_equal(&scene->settings, &scene->new_settings)) {
        discard_current_render(queue);
    }

    s64 tiles_retired = queue->tiles_retired;
    if (queue->discard_render || (tiles_retired >= (s64)queue->total_tile_count)) {
        result = !queue->discard_render;

        copy(parameters->backbuffer, parameters->frontbuffer);
        Swap(parameters->backbuffer, parameters->frontbuffer);

        if (queue->discard_render ||
            !structs_are_equal(&scene->camera, &scene->new_camera))
        {
            reset(parameters->backbuffer);
            reset(parameters->path_guide);

            recompute_camera(&scene->new_camera);

            scene->camera   = scene->new_camera;
            scene->settings = scene->new_settings;
        } else {
            parameters->backbuffer->frame_count += scene->settings.samples_per_pixel;
            scene->total_frame_index            += 1;
        }

        queue->discard_render = false;

        if (out_stats) {
            *out_stats = g_stats;
        }
        zero_struct(&g_stats);

        if (out_time_elapsed) {
            *out_time_elapsed = platform_get_seconds_elapsed(queue->render_start_time,
                                                             platform_get_timestamp());
        }

        queue->render_start_time = platform_get_timestamp();

        // NOTE: At this point, all threads should be waiting on the semaphore,
        //       so we're safe to set these things.
        queue->tile_index    = queue->total_tile_count;
        queue->tiles_retired = 0;

        // NOTE: Then, we make sure those writes up there are done before we release the semaphore
        WRITE_BARRIER;

        u32 previous_count;
        platform_release_semaphore(queue->semaphore, queue->thread_count, &previous_count);

        // NOTE: And we make sure (to some degree) that all threads were in fact
        //       waiting on the semaphore
        assert(previous_count == 0);
    }

    return result;
}

static AccumulationBuffer*
get_read_buffer(WorkQueue* queue) {
    return queue->parameters.frontbuffer;
}

//
// NOTE: Frame Timing
//

static void
record_frame_time(FrameHistory* history, f64 time) {
    history->clocks[history->write_cursor++] = time;
    if (history->write_cursor >= ArrayCount(history->clocks)) {
        history->write_cursor = 0;
    }
}

static FrameTimingInfo
get_frame_timing_info(FrameHistory* history) {
    f64 min_time     = DBL_MAX;
    f64 average_time = 0.0f;
    f64 max_time     = 0.0f;
    for (usize frame_index = 0; frame_index < ArrayCount(history->clocks); ++frame_index) {
        f64 timing = history->clocks[frame_index];
        min_time      = min(min_time, timing);
        average_time += timing;
        max_time      = max(max_time, timing);
    }
    FrameTimingInfo result;
    result.min     = min_time;
    result.average = average_time / (f64)ArrayCount(history->clocks);
    result.max     = max_time;
    return result;
}

//
// NOTE: Scene setup
//

static 
SCENE_DESCRIPTION(week_1_scene) {
    camera->vfov           = DEG_TO_RAD*60.0f;
    camera->aspect_ratio   = (f32)w / (f32)h;
    camera->lens_radius    = 0.0f;
    camera->focus_distance = 1.0f;

    camera->p = v3(0, 4, -10);
    aim_camera(camera, v3(0, 0, -1));

    settings->lens_distortion = 0.0f;
    settings->integrator = find_integrator("Whitted");
    load_reconstruction_kernel(find_filter("Box"));
    post_settings->tonemapping = false;
    scene->ambient_light = v3(PI_32);

    MaterialID ground_material = add_diffuse_material(scene, v3(1), 1.0f, 0.0f, true, v3(0.0f));
    add_plane(scene, ground_material, v3(0, 1, 0), 0.0f);
}

static 
SCENE_DESCRIPTION(week_2_scene) {
    camera->vfov           = DEG_TO_RAD*60.0f;
    camera->aspect_ratio   = (f32)w / (f32)h;
    camera->lens_radius    = 0.0f;
    camera->focus_distance = 1.0f;

    camera->p = v3(0, 4, -10);
    aim_camera(camera, v3(0, 0, -1));

    settings->lens_distortion = 0.0f;
    settings->integrator = find_integrator("Whitted");
    load_reconstruction_kernel(find_filter("Box"));
    post_settings->tonemapping = false;
    scene->ambient_light = v3(PI_32);

    MaterialID ground_material = add_diffuse_material(scene, v3(1), 1.0f, 0.0f, true, v3(0.0f));
    MaterialID sphere_material = add_diffuse_material(scene, v3(1.0f, 0.0f, 0.0f), 1.0f);
    add_plane(scene, ground_material, v3(0, 1, 0), 0.0f);
    add_sphere(scene, sphere_material, 4.0f, transform_translate(v3(0, 4, 0)));
}

static 
SCENE_DESCRIPTION(week_3_scene) {
    camera->vfov           = DEG_TO_RAD*60.0f;
    camera->aspect_ratio   = (f32)w / (f32)h;
    camera->lens_radius    = 0.0f;
    camera->focus_distance = 1.0f;

    camera->p = v3(0, 4, -10);
    aim_camera(camera, v3(0, 0, -1));

    settings->lens_distortion = 0.0f;
    settings->integrator = find_integrator("Whitted");
    load_reconstruction_kernel(find_filter("Box"));
    post_settings->tonemapping = false;

    MaterialID ground_material = add_diffuse_material(scene, v3(1), 1.0f, 0.0f, true, v3(0.0f));
    MaterialID sphere_material = add_diffuse_material(scene, v3(1.0f, 0.0f, 0.0f), 1.0f);
    MaterialID light_material  = add_emissive_material(scene, v3(12500));
    add_plane(scene, ground_material, v3(0, 1, 0), 0.0f);
    add_sphere(scene, sphere_material, 4.0f, transform_translate(v3(0, 4, 0)));
    add_sphere(scene, light_material, 0.1f, transform_translate(v3(8, 16, -8)));
}

static 
SCENE_DESCRIPTION(week_4_scene) {
    camera->vfov           = DEG_TO_RAD*60.0f;
    camera->aspect_ratio   = (f32)w / (f32)h;
    camera->lens_radius    = 0.0f;
    camera->focus_distance = 1.0f;

    camera->p = v3(0, 4, -10);
    aim_camera(camera, v3(0, 0, -1));

    settings->lens_distortion = 0.0f;
    settings->integrator = find_integrator("Whitted");
    load_reconstruction_kernel(find_filter("Box"));
    post_settings->tonemapping = false;

    MaterialID ground_material = add_diffuse_material(scene, v3(1), 1.0f, 0.0f, true, v3(0.0f));
    MaterialID sphere_material = add_material(scene, {
        .albedo = v3(0.5f),
        .ior      = 1.5f,
        .metallic = 0.5f,
        .roughness = 0.05f,
    });
    MaterialID light_material  = add_emissive_material(scene, v3(12500));
    add_plane(scene, ground_material, v3(0, 1, 0), 0.0f);
    add_sphere(scene, sphere_material, 4.0f, transform_translate(v3(0, 4, 0)));
    add_sphere(scene, light_material, 0.1f, transform_translate(v3(8, 16, -8)));
}

static 
SCENE_DESCRIPTION(week_5_scene) {
    camera->vfov           = DEG_TO_RAD*50.0f;
    camera->aspect_ratio   = (f32)w / (f32)h;
    camera->lens_radius    = 0.0f;
    camera->focus_distance = 1.0f;

    camera->p = v3(-5, 8, -15);
    aim_camera(camera, v3(0, 0, -1));

    settings->lens_distortion = 0.0f;
    settings->integrator = find_integrator("Advanced Pathtracer");
    settings->max_bounce_count = 12;
    settings->caustics = false;
    load_reconstruction_kernel(find_filter("Gaussian 3"));
    post_settings->tonemapping = true;
    scene->bot_sky_color = scene->top_sky_color = v3(0.1f, 0.7f, 2.0f);
    scene->ambient_light = scene->bot_sky_color;

    MaterialID ground_material = add_diffuse_material(scene, v3(1.0f, 0.0f, 0.0f), 1.0f, 0.0f, true, v3(1.0f, 1.0f, 0.0f));
    MaterialID glass_material  = add_translucent_material(scene, v3(0), 1.8f);
    MaterialID metal_material = add_material(scene, {
        .albedo = v3(0.95f),
        .ior      = 1.5f,
        .metallic = 0.8f,
    });
    MaterialID air_material    = add_translucent_material(scene, v3(0.0f), 1.0f);
    MaterialID light_material  = add_emissive_material(scene, v3(325000000));
    add_box(scene, ground_material, v3(16, 1, 20), transform_translate(v3(0, -1.0f, 16)));
    add_sphere(scene, glass_material, 4.0f, transform_translate(v3(-5, 8, 0)));
    add_sphere(scene, air_material, 3.8f, transform_translate(v3(-5, 8, 0)));
    add_sphere(scene, metal_material, 4.0f, transform_translate(v3(0, 5, 8)));
    add_sphere(scene, light_material, 10.0f, transform_translate(10000.0f*v3(-1, 10, -8)));
}

static 
SCENE_DESCRIPTION(week_6_scene) {
    camera->vfov = DEG_TO_RAD*45.0f;
    camera->aspect_ratio = (f32)w / (f32)h;
    camera->lens_radius = 10.0f;

    camera->p = v3(0, 7.5f, -25);
    aim_camera(camera, v3(0, 0, -1));

    camera->focus_distance = 19.77f;

    settings->integrator = find_integrator("Whitted");
    settings->lens_distortion = 0.0f;

    MaterialID ground_material        = add_diffuse_material(scene, v3(0.55f, 0.55f, 0.55f), 1.0f);
    MaterialID cornell_white_material = add_diffuse_material(scene, v3(0.75f, 0.75f, 0.75f), 1.1f, 0.25f);
    MaterialID cornell_red_material   = add_diffuse_material(scene, v3(0.95f, 0.1f, 0.1f), 1.0f);
    MaterialID cornell_green_material = add_diffuse_material(scene, v3(0.1f, 0.95f, 0.1f), 1.0f);
    MaterialID cornell_blue_material  = add_diffuse_material(scene, v3(0.1f, 0.1f, 0.95f), 1.0f);
    MaterialID glass_material          = add_translucent_material(scene, v3(0.15f), 1.5f);
    MaterialID red_material           = add_translucent_material(scene, v3(0.0f, 0.1f, 0.1f), 1.6f);
    MaterialID air_material           = add_translucent_material(scene, v3(0.0f), 1.0f);

    MaterialID metal_material = add_material(scene, {
        .albedo   = v3(0.85f, 0.85f, 0.85f),
        .ior      = 0.2f,
        .metallic = 1.0f,
    });

    MaterialID mixed_metal_material = add_material(scene, {
        .albedo   = v3(0.05f, 0.05f, 0.95f),
        .ior      = 1.5f,
        .metallic = 0.15f,
    });

    MaterialID white_light_material = add_emissive_material(scene, 6.0f*v3(10.0f, 10.0f, 10.0f));
    MaterialID   red_light_material = add_emissive_material(scene, 10.0f*v3(10.0f, 2.0f, 0.0f));
    MaterialID  blue_light_material = add_emissive_material(scene, 3.0f*v3(2.0f, 6.0f, 10.0f));
    MaterialID green_light_material = add_emissive_material(scene, 3.0f*v3(1.0f, 10.0f, 2.0f));

    add_box(scene, metal_material, v3(2.0f, 6.0f, 2.0f), transform_translate(v3(-3, 3, 1))*transform_rotate_y_axis(-0.125f*PI_32));
    add_sphere(scene, glass_material, 2.0f, transform_translate(v3(-3, 2.3f, -5)));
    add_sphere(scene, mixed_metal_material, 2.0f, transform_translate(v3(3, 2.0f, -4)));

    add_plane(scene, ground_material, v3(0, 1, 0), 0.0f);
    add_plane(scene, ground_material, v3(0,-1, 0), -15.0f);
    add_plane(scene, ground_material, v3(0, 0,-1), -8.0f);
    add_plane(scene, cornell_blue_material, v3(0, 0, 1), -8.0f);
    add_plane(scene, cornell_red_material, v3(1, 0, 0), -7.5f);
    add_plane(scene, cornell_green_material, v3(-1, 0, 0), -7.5f);

    add_sphere(scene,  white_light_material, 1.5f, transform_translate(v3(0, 13.4f, -2)));
}

static 
SCENE_DESCRIPTION(week_7_scene) {
    camera->vfov = DEG_TO_RAD*39.0f;
    camera->aspect_ratio = (f32)w / (f32)h;
    camera->lens_radius = 0.0f;

    camera->p = v3(0, 7.0f, -15);
    aim_camera_at(camera, v3(0, 0, 0));
    camera->focus_distance = 10.8f;

    settings->integrator = find_integrator("Whitted");
    settings->lens_distortion = 0.0f;
    settings->vignette_strength = 0.0f;
    settings->caustics = false;
    scene->bot_sky_color = scene->top_sky_color = v3(0.2f, 0.7f, 0.95f);

    load_reconstruction_kernel(find_filter("Gaussian 3"));

    MaterialID ground_material = add_diffuse_material(scene, v3(0.55f, 0.55f, 0.55f), 1.0f);
    MaterialID sphere_material = add_material(scene, {
        .albedo   = v3(0.85f, 0.85f, 0.85f),
        .ior      = 1.5f,
        .metallic = 1.0f,
    });

    add_plane(scene, ground_material, v3(0, 1, 0), 0.0f);
    add_sphere(scene, sphere_material, 1.0f, transform_translate(v3(0, 1.0f, 0)));

    MaterialID white_light_material = add_emissive_material(scene, 3.0f*v3(10.0f, 10.0f, 10.0f));
    add_sphere(scene, white_light_material, 30.0f, transform_translate(v3(-50, 100.0f, -50)));

    RandomSeries entropy = random_seed(2);

    for (int x = -100; x <= 100; ++x)
    for (int y = -100; y <= 100; ++y) {
        if ((x < -2 || x > 2) ||
            (y < -2 || y > 2))
        {
            V4 rnd  = random_unilaterals(&entropy);
            V4 rnd2 = random_unilaterals(&entropy);
            V4 rnd3 = random_unilaterals(&entropy);

            MaterialID box_material;
            box_material = add_diffuse_material(scene,
                                                v3(0.25f + 0.75f*rnd3.x, 0.25f + 0.75f*rnd3.y, 0.25f + 0.75f*rnd3.z),
                                                1.5f, 0.75f);

            M4x4Inv m = transform_translate(v3(2.0f*(f32)(-.5f + rnd.x + x), 1.0f, 2.0f*(f32)(-0.5f + rnd.y + y)));
            m *= transform_rotate_y_axis(PI_32*rnd.z);
            m *= transform_rotate_x_axis(-0.25f + 0.5f*PI_32*rnd.w);
            add_box(scene, box_material, v3(0.25f + rnd2.x, 0.5f + rnd2.y, 0.25f + rnd2.z), m);
        }
    }
}

static 
SCENE_DESCRIPTION(week_7_nicer_scene) {
    camera->vfov = DEG_TO_RAD*39.0f;
    camera->aspect_ratio = (f32)w / (f32)h;
    camera->lens_radius = 6.0f;

    camera->p = v3(0, 8.0f, -15);
    aim_camera_at(camera, v3(0, 0, 0));
    camera->focus_distance = 10.8f;

    settings->integrator = find_integrator("Advanced Pathtracer");
    settings->lens_distortion = -0.5f;
    settings->vignette_strength = 1.0f;
    settings->caustics = false;
    scene->bot_sky_color = scene->top_sky_color = v3(0.2f, 0.7f, 0.95f);

    post_settings->contrast = 0.1f;

    load_reconstruction_kernel(find_filter("Gaussian 3"));

    MaterialID ground_material = add_diffuse_material(scene, v3(0.55f, 0.55f, 0.55f), 1.0f);
    MaterialID sphere_material = add_material(scene, {
        .albedo   = v3(0.85f, 0.85f, 0.85f),
        .ior      = 1.5f,
        .metallic = 1.0f,
    });

    // scene->skydome = load_environment_map(&scene->arena, temp_arena, "data/boiler_room_2k.hdr");

    add_plane(scene, ground_material, v3(0, 1, 0), 0.0f);
    add_sphere(scene, sphere_material, 1.0f, transform_translate(v3(0, 1.0f, 0)));

    MaterialID white_light_material = add_emissive_material(scene, 25.0f*v3(10.0f, 7.0f, 4.0f));
    add_sphere(scene, white_light_material, 1000.0f, transform_translate(100.0f*v3(-50, 100.0f, -50)));

    RandomSeries entropy = random_seed(1);

    for (int x = -100; x <= 100; ++x)
    for (int y = -100; y <= 100; ++y) {
        if ((x < -2 || x > 2) ||
            (y < -2 || y > 2))
        {
            V4 rnd  = random_unilaterals(&entropy);
            V4 rnd2 = random_unilaterals(&entropy);
            V4 rnd3 = random_unilaterals(&entropy);

            MaterialID box_material;
            if (rnd3.w > 0.67f && rnd3.w < 0.90f) {
                box_material = add_translucent_material(scene,
                                                        v3(1.0f) - v3(0.25f + 0.75f*rnd3.x, 0.25f + 0.75f*rnd3.y, 0.25f + 0.75f*rnd3.z),
                                                        1.5f);
            } else if (rnd3.w > 0.90f) {
                box_material = add_material(scene, {
                    .albedo   = v3(0.25f + 0.75f*rnd3.x, 0.25f + 0.75f*rnd3.y, 0.25f + 0.75f*rnd3.z),
                    .ior      = 1.5f,
                    .metallic = 1.0f,
                });
            } else {
                box_material = add_diffuse_material(scene,
                                                    v3(0.25f + 0.75f*rnd3.x, 0.25f + 0.75f*rnd3.y, 0.25f + 0.75f*rnd3.z),
                                                    1.5f, 0.25f);
            }

            M4x4Inv m = transform_translate(v3(2.0f*(f32)(-.5f + rnd.x + x), 1.0f, 2.0f*(f32)(-0.5f + rnd.y + y)));
            m *= transform_rotate_y_axis(PI_32*rnd.z);
            m *= transform_rotate_x_axis(-0.25f + 0.5f*PI_32*rnd.w);
            add_box(scene, box_material, v3(0.25f + rnd2.x, 0.5f + rnd2.y, 0.25f + rnd2.z), m);
        }
    }
}

static 
SCENE_DESCRIPTION(cornell_box_scene) {
    camera->vfov = DEG_TO_RAD*45.0f;
    camera->aspect_ratio = (f32)w / (f32)h;
    camera->lens_radius = 10.0f;

    camera->p = v3(0, 7.5f, -25);
    aim_camera(camera, v3(0, 0, -1));

    camera->focus_distance = 19.77f;

    settings->integrator = find_integrator("Advanced Pathtracer");
    settings->lens_distortion = 1.0f;

    MaterialID ground_material        = add_diffuse_material(scene, v3(0.55f, 0.55f, 0.55f), 1.0f);
    MaterialID cornell_white_material = add_diffuse_material(scene, v3(0.75f, 0.75f, 0.75f), 1.1f, 0.25f);
    MaterialID cornell_red_material   = add_diffuse_material(scene, v3(0.95f, 0.1f, 0.1f), 1.0f);
    MaterialID cornell_green_material = add_diffuse_material(scene, v3(0.1f, 0.95f, 0.1f), 1.0f);
    MaterialID cornell_blue_material  = add_diffuse_material(scene, v3(0.1f, 0.1f, 0.95f), 1.0f);
    MaterialID glass_material          = add_translucent_material(scene, v3(0.15f), 1.5f);
    MaterialID red_material           = add_translucent_material(scene, v3(0.0f, 0.1f, 0.1f), 1.6f);
    MaterialID air_material           = add_translucent_material(scene, v3(0.0f), 1.0f);

    MaterialID metal_material = add_material(scene, {
        .albedo   = v3(0.85f, 0.75f, 0.45f),
        .ior      = 0.2f,
        .metallic = 1.0f,
    });

    MaterialID mixed_metal_material = add_material(scene, {
        .albedo   = v3(0.05f, 0.05f, 0.95f),
        .ior      = 1.5f,
        .metallic = 0.15f,
    });

    MaterialID white_light_material = add_emissive_material(scene, 6.0f*v3(10.0f, 10.0f, 10.0f));
    MaterialID   red_light_material = add_emissive_material(scene, 10.0f*v3(10.0f, 2.0f, 0.0f));
    MaterialID  blue_light_material = add_emissive_material(scene, 3.0f*v3(2.0f, 6.0f, 10.0f));
    MaterialID green_light_material = add_emissive_material(scene, 3.0f*v3(1.0f, 10.0f, 2.0f));

    add_box(scene, metal_material, v3(2.5f, 8.0f, 2.5f), transform_translate(v3(-3, 4, 1))*transform_rotate_y_axis(-0.125f*PI_32));

    add_box(scene, metal_material, v3(0.5f, 2.0f, 0.5f), transform_translate(v3(-5, 2, -5)));
    add_sphere(scene, glass_material, 2.0f, transform_translate(v3(-5, 6.0f, -5)));

    Mesh dragon = load_mesh(scene, "data/dragon_mcguire.obj", temp_arena);
    if (dragon.triangle_count) {
        add_mesh(scene, mixed_metal_material, &dragon, (transform_translate(v3(5, 2.0f, -3))*
                                                        transform_scale(v3(10.0f))*
                                                        transform_rotate_y_axis(0.25f*PI_32)));
    }

    add_plane(scene, ground_material, v3(0, 1, 0), 0.0f);
    add_plane(scene, ground_material, v3(0,-1, 0), -15.0f);
    add_plane(scene, ground_material, v3(0, 0,-1), -8.0f);
    add_plane(scene, cornell_red_material, v3(1, 0, 0), -10.5f);
    add_plane(scene, cornell_green_material, v3(-1, 0, 0), -10.5f);

    add_sphere(scene,  white_light_material, 1.5f, transform_translate(v3(0, 13.4f, -2)));
}

static 
SCENE_DESCRIPTION(dragon_scene) {
    camera->vfov = DEG_TO_RAD*40.0f;
    camera->aspect_ratio = (f32)w / (f32)h;
    camera->lens_radius = 6.0f;

    camera->p = v3(-25, 6, 0);
    aim_camera_at(camera, v3(1, 5, 0));

    MaterialID ground_material       = add_diffuse_material(scene, v3(0.55f, 0.55f, 0.55f), 1.0f, 0.0f, true);
    MaterialID wall_material         = add_diffuse_material(scene, v3(0.55f, 0.85f, 0.55f), 1.0f, 0.0f, true, v3(0.65f, 0.15f, 0.65f));
    MaterialID test_dragon_material  = add_diffuse_material(scene, v3(0.25f, 0.35f, 0.55f), 1.3f);
    MaterialID blue_glass_material   = add_translucent_material(scene, v3(0.98f, 0.35f, 0.15f), 1.5f);
    MaterialID red_glass_material    = add_translucent_material(scene, v3(0.15f, 0.35f, 0.95f), 1.5f);
    MaterialID cool_material         = add_translucent_material(scene, v3(0.98f, 0.35f, 0.15f), 1.5f);
    MaterialID marble_material       = add_translucent_material(scene, v3(0.0f, 0.0f, 0.0f), 1.5f);
    MaterialID air_material          = add_translucent_material(scene, v3(0.0f, 0.0f, 0.0f), 1.0f);
    MaterialID red_material          = add_translucent_material(scene, v3(0.0f, 0.1f, 0.2f), 1.5f);

    MaterialID rough_material = add_material(scene, {
        .albedo = v3(0.15f, 0.5f, 0.8f),
        .ior       = 1.3f,
        .roughness = 0.75f,
    });

    MaterialID metal_material = add_material(scene, {
        .albedo = v3(0.85f, 0.85f, 0.85f),
        .metallic = 1.0f,
    });

    MaterialID white_light_material = add_emissive_material(scene, 8.0f*v3(10.0f, 10.0f, 9.0f));
    MaterialID   red_light_material = add_emissive_material(scene, 10.0f*v3(10.0f, 2.0f, 0.0f));
    MaterialID  blue_light_material = add_emissive_material(scene, 3.0f*v3(2.0f, 6.0f, 10.0f));
    MaterialID green_light_material = add_emissive_material(scene, 3.0f*v3(1.0f, 10.0f, 2.0f));

    scene->skydome = load_environment_map(&scene->arena, temp_arena, "data/ballroom_2k.hdr");

    Mesh dragon = load_mesh(scene, "data/dragon_mcguire.obj", temp_arena);
    if (dragon.triangle_count) {
        add_mesh(scene, blue_glass_material, &dragon, transform_translate(v3(0, 6.0f, 0))*transform_scale(v3(14.0f)));
        add_mesh(scene, red_glass_material, &dragon, transform_translate(v3(-5, 3.7f, 0))*transform_scale(v3(6.0f)));
        add_mesh(scene, rough_material, &dragon, transform_translate(v3(-5, 3.7f, -7))*transform_scale(v3(6.0f)));
        add_mesh(scene, metal_material, &dragon, transform_translate(v3(-5, 3.7f,  7))*transform_scale(v3(6.0f)));
    }

    add_box(scene, ground_material, v3(10, 1, 10), transform_translate(v3(0, 1.0f, 0)));
    add_box(scene, ground_material, v3(40, 1, 40), transform_translate(v3(8.0f, -1.0f, 0)));

#if 0
    add_sphere(scene, marble_material, 7.2f, transform_translate(v3(0, 1, 0)));
    add_sphere(scene, air_material, 7.0f, transform_translate(v3(0, 1, 0)));
    add_box(scene, wall_material, v3(3.0f, 10, 20), transform_translate(v3(-15, 5, 0)));
#endif

    // NOTE: Lights
    add_sphere(scene, blue_light_material, 2, transform_translate(v3(-5.0f, 25.0f, 5)));
    add_sphere(scene, red_light_material, 2, transform_translate(v3(5.0f, 35.0f, 8)));
    add_sphere(scene, white_light_material, 2, transform_translate(v3(0.0f, 15.0f, 12)));
}

static 
SCENE_DESCRIPTION(platforms_scene) {
    camera->vfov = DEG_TO_RAD*40.0f;
    camera->aspect_ratio = (f32)w / (f32)h;
    camera->lens_radius = 10.0f;
    camera->focus_distance = 15.0f;

    camera->p = v3(0, 3, -18);
    aim_camera_at(camera, v3(0, 0, 0));

    settings->lens_distortion = 2.0f;
    settings->caustics        = false;

    scene->skydome = load_environment_map(&scene->arena, temp_arena, "data/boiler_room_2k.hdr");

    MaterialID ground_material = add_diffuse_material(scene, v3(0.8f, 0.1f, 0.1f), 1.0f, 0.0f, true, v3(0.8f, 0.8f, 0.1f));
    MaterialID marble_material = add_translucent_material(scene, v3(0.5f, 0.25f, 0.0f), 1.5f);
    MaterialID inner_material  = add_diffuse_material(scene, v3(0.85f, 0.85f, 0.35f), 1.5f);
    MaterialID air_material    = add_translucent_material(scene, v3(0.0f, 0.0f, 0.0f), 1.0f);

    MaterialID pedestal_material  = add_diffuse_material(scene, v3(0.5f, 0.5f, 0.5f), 1.0f);
    MaterialID checker_material   = add_material(scene, {
        .flags         = Material_Checkers,
        .albedo        = v3(0.5f,  0.5f,  0.5f),
        .checker_color = v3(0.25f, 0.25f, 0.25f),
        .ior           = 1.1f,
    });

#if 1
#if 1
    MaterialID roughness_000_material = add_material(scene, {
        .albedo    = v3(0.95f, 0.95f, 0.95f),
        .ior       = 1.5f,
        .metallic  = 1.0f,
        .roughness = 0.0f,
    });
    MaterialID roughness_025_material = add_material(scene, {
        .albedo    = v3(0.95f, 0.95f, 0.95f),
        .ior       = 1.5f,
        .metallic  = 1.0f,
        .roughness = 0.10f,
    });
    MaterialID roughness_050_material = add_material(scene, {
        .albedo    = v3(0.95f, 0.95f, 0.95f),
        .ior       = 1.5f,
        .metallic  = 1.0f,
        .roughness = 0.20f,
    });
    MaterialID roughness_100_material = add_material(scene, {
        .albedo    = v3(0.95f, 0.95f, 0.95f),
        .ior       = 1.5f,
        .metallic  = 1.0f,
        .roughness = 0.4f,
    });

    add_sphere(scene, marble_material, 2.5f, transform_translate(v3(-9.0f, 0.0f, 0.0f)));
    add_sphere(scene, marble_material, 2.5f, transform_translate(v3(-3.0f, 0.0f, 0.0f)));
    add_sphere(scene, marble_material, 2.5f, transform_translate(v3( 3.0f, 0.0f, 0.0f)));
    add_sphere(scene, marble_material, 2.5f, transform_translate(v3( 9.0f, 0.0f, 0.0f)));

    add_box(scene, checker_material, v3(50.0f, 1.0f, 50.0f), transform_translate(v3(0.0f, -10.0f,  0.0f)));
    add_box(scene, pedestal_material, v3(10.0f, 1.0f, 10.0f), transform_translate(v3(-35.0f, -6.5f,  0.0f)));
    add_box(scene, pedestal_material, v3(10.0f, 1.0f, 10.0f), transform_translate(v3( 35.0f,  3.5f,  0.0f)));
    add_box(scene, pedestal_material, v3(10.0f, 1.0f, 10.0f), transform_translate(v3( 0.0f,   9.5f, -35.0f)));
    add_box(scene, pedestal_material, v3(10.0f, 1.0f, 10.0f), transform_translate(v3( 0.0f,   0.5f,  35.0f)));

    // add_box(scene, pedestal_material, v3(2.0f, 10.0f, 2.0f), transform_translate(v3( 15.0f, 5.0f + -3.5f,  5.0f)));
    // add_box(scene, pedestal_material, v3(2.0f, 10.0f, 2.0f), transform_translate(v3(-15.0f, 5.0f + -3.5f, -5.0f)));
#else
    f32 sphere_radius = 2.5f;
    f32 sphere_diameter = 2.0f*sphere_radius;
    f32 sphere_spacing  = 1.0f;
    f32 left   = -4.0f*(sphere_diameter + sphere_spacing);
    f32 bottom = -4.0f*(sphere_diameter + sphere_spacing);
    f32 horizontal_step = 8.0f*(sphere_diameter + sphere_spacing);
    f32 vertical_step   = 8.0f*(sphere_diameter + sphere_spacing);
    for (int x = -4; x <= 4; ++x)
    for (int y = -4; y <= 4; ++y) {
        f32 u = (4.0f + (f32)x) / 8.0f;
        f32 v = (4.0f + (f32)y) / 8.0f;

        f32 sphere_x = left   + u*horizontal_step;
        f32 sphere_y = bottom + v*vertical_step;

        Material m = {};
        m.ior = 1.5f;
        m.albedo = v3(0.25f, 0.55f, 0.15f);
        m.metallic = u;
        m.roughness = v;

        add_sphere(scene, add_material(scene, m), sphere_radius, transform_translate(v3(sphere_x, 0.0f, sphere_y)));
    }

    add_box(scene, pedestal_material, v3(horizontal_step, 1.0f, vertical_step), transform_translate(v3(0.0f, -3.5f, 0.0f)));
#endif
#endif

    // add_sphere(scene, marble_material, 5.0f, transform_translate(v3(0.0f)));
    // add_plane(scene, ground_material, v3(0, 1, 0), -6.0f);

#if 0
    RandomSeries entropy = random_seed(1);
    for (usize i = 0; i < 64; ++i) {
        V3 p = 2.8f*random_in_unit_sphere(&entropy);
        f32 size = 0.1f + 0.4f*random_unilateral(&entropy);
        add_sphere(scene, air_material, size, transform_translate(p));
    }
#endif

    MaterialID pink_light_material  = add_emissive_material(scene, 50.0f*v3(10.0f, 1.0f, 10.0f));
    MaterialID red_light_material   = add_emissive_material(scene, 50.0f*v3(10.0f, 1.0f, 1.0f));
    MaterialID green_light_material = add_emissive_material(scene, 50.0f*v3(1.0f, 10.0f, 1.0f));
    MaterialID blue_light_material  = add_emissive_material(scene, 50.0f*v3(1.0f, 1.0f, 10.0f));

    add_sphere(scene,  blue_light_material,  2, transform_translate(v3(-35.0f, -6.5f + 10.0f,  0.0f)));
    add_sphere(scene,  red_light_material,   2, transform_translate(v3( 35.0f,  3.5f + 10.0f,  0.0f)));
    add_sphere(scene,  pink_light_material,  2, transform_translate(v3( 0.0f,   9.5f + 10.0f, -35.0f)));
    add_sphere(scene,  green_light_material, 2, transform_translate(v3( 0.0f,   0.5f + 10.0f,  35.0f)));

    add_sphere(scene,  green_light_material, 0.25f, transform_translate(v3( 0.0f, 20.0f,  0.0f)));
}

static 
SCENE_DESCRIPTION(nested_dielectrics_scene) {
    camera->vfov = DEG_TO_RAD*40.0f;
    camera->aspect_ratio = (f32)w / (f32)h;
    camera->lens_radius = 6.0f;

    camera->p = v3(-25, 6, 0);
    aim_camera_at(camera, v3(1, 5, 0));

    MaterialID clear_marble_material = add_translucent_material(scene, v3(0.0f, 0.0f, 0.0f), 1.5f);
    MaterialID blue_marble_material  = add_translucent_material(scene, v3(0.6f, 0.3f, 0.0f), 1.5f);
    MaterialID air_material    = add_translucent_material(scene, v3(0.0f, 0.0f, 0.0f), 1.0f);
    MaterialID ground_material = add_diffuse_material(scene, v3(0.55f, 0.55f, 0.55f), 1.0f, 0.0f, true);

    MaterialID white_light_material = add_emissive_material(scene, 8.0f*v3(10.0f, 10.0f, 9.0f));

    scene->skydome = load_environment_map(&scene->arena, temp_arena, "data/epping_forest_02_2k.hdr");

    add_box(scene, ground_material, v3(10, 1, 10), transform_translate(v3(0, 1.0f, 0)));
    add_box(scene, ground_material, v3(40, 1, 40), transform_translate(v3(8.0f, -1.0f, 0)));

    float floor_height = 2.0f;

    RandomSeries entropy = random_seed(SDL_GetTicks());
    uint32_t marble_count = random_range(&entropy, 20, 40);

    for (size_t marble_index = 0; marble_index < marble_count; marble_index++)
    {
        V3 absorption = v3(0.25f) + 0.75f*random_unilaterals(&entropy).xyz;
		MaterialID marble_material = add_translucent_material(scene, absorption, 1.5f);

        V2 marble_xy = 8.0f*random_bilaterals(&entropy).xy;
		float marble_radius = 0.6f + random_unilaterals(&entropy).x;

		V3 marble_p = v3(marble_xy.x, floor_height + marble_radius, marble_xy.y);
		add_sphere(scene, marble_material, marble_radius, transform_translate(marble_p));

		uint32_t bubble_count = random_range(&entropy, 5, 12);

		float min_bubble_radius = 0.05f;
		float max_bubble_radius = 0.15f;

		for (size_t i = 0; i < bubble_count; i++)
		{
			V4 r1 = random_bilaterals(&entropy);

			float bubble_radius = min_bubble_radius + remap(r1.w, -1.0f, 1.0f)*max_bubble_radius;
			float bubble_max_offset_from_center = marble_radius - bubble_radius - 0.05f;
			float bubble_offset = bubble_max_offset_from_center*random_unilaterals(&entropy).x;

			V3 bubble_p = marble_p + bubble_offset*r1.xyz;

			add_sphere(scene, ground_material, bubble_radius, transform_translate(bubble_p));
		}
    }

    // NOTE: Lights
    add_sphere(scene, white_light_material, 2, transform_translate(v3(0.0f, 15.0f, 12)));
}

static SceneDescription g_scenes[] = {
    { "Dragon",             dragon_scene             },
    { "Cornell Box",        cornell_box_scene        },
    { "Floating Platforms", platforms_scene          },
    { "Nested Dielectrics", nested_dielectrics_scene },
    { "Week 1",             week_1_scene             },
    { "Week 2",             week_2_scene             },
    { "Week 3",             week_3_scene             },
    { "Week 4",             week_4_scene             },
    { "Week 5",             week_5_scene             },
    { "Week 6",             week_6_scene             },
    { "Week 7",             week_7_scene             },
    { "Week 7, Nicer",      week_7_nicer_scene       },
};

static void
init_scene(Scene* scene) {
    (void)scene->materials.add();  // NOTE: Null material
    (void)scene->primitives.add(); // NOTE: Null primitive
    load_reconstruction_kernel(find_filter("Mitchell Netravali"));

    // NOTE: Default settings
    SceneSettings* settings = &scene->new_settings;

    settings->next_event_estimation     = true;
    settings->importance_sample_lights  = true;
    settings->importance_sample_diffuse = true;
    settings->use_mis                   = true;
    settings->russian_roulette          = true;
    settings->sampling_strategy         = SamplingStrategy_Stratified;
    settings->use_path_guide            = false;
    settings->caustics                  = true;
    settings->lens_distortion           = 1.0f;
    settings->f_factor                  = 0.0f;
    settings->diaphragm_edges           = 6.0f;
    settings->phi_shutter_max           = 0.5f;
    settings->vignette_strength         = 0.25f;
    settings->samples_per_pixel         = 1;
    settings->max_bounce_count          = 12;
    settings->integrator                = &g_integrators[0];

    scene->post_settings.tonemapping    = true;
    scene->post_settings.srgb_transform = true;
    scene->post_settings.midpoint       = 0.5f;
}

static void
load_scene(Scene* scene, SceneDescription* desc, u32 w, u32 h, Arena* temp_arena) {
    clear_scene(scene);
    init_scene(scene);
    desc->f(scene, w, h, temp_arena, &scene->new_camera, &scene->new_settings, &scene->post_settings);
    scene->name = desc->name;

    ScopedMemory scoped_memory(temp_arena);

    PlatformHighResTime bvh_clock_start = platform_get_timestamp();
    create_scene_bvh(scene, temp_arena);
    PlatformHighResTime bvh_clock_end   = platform_get_timestamp();

    f64 bvh_time = platform_get_seconds_elapsed(bvh_clock_start, bvh_clock_end);
    printf("BVH Construction took: %fs\n", bvh_time);
}

//
// NOTE: Main
//

static int
mu_text_width_callback(mu_Font mu_font, const char* str, int len) {
    if (len == -1) len = (int)strlen(str);
    return 8*len;
}

static int
mu_text_height_callback(mu_Font font) {
    return 12;
}

struct Font {
    SDL_Surface* surface;
    SDL_Texture* texture;
};

static char mu_button_map[256];
static char mu_key_map[256];

static void
render_text(SDL_Renderer* renderer, Font* font, unsigned char* text, u32 x, u32 y, Color_BGRA color) {
    u32 font_w = font->surface->w;
    u32 font_h = font->surface->h;

    u32 char_w = 8;
    u32 char_h = 12;

    u32 chars_per_row = font_w / char_w;

    u32 at_x = x;
    u32 at_y = y;

    for (unsigned char* at = text; *at; ++at) {
        unsigned char c = *at;

        SDL_Rect src_rect;
        src_rect.x = char_w*(c % chars_per_row);
        src_rect.y = char_h*(c / chars_per_row);
        src_rect.w = char_w;
        src_rect.h = char_h;

        SDL_Rect dst_rect;
        dst_rect.x = at_x;
        dst_rect.y = at_y;
        dst_rect.w = char_w;
        dst_rect.h = char_h;

        SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
        SDL_RenderCopy(renderer, font->texture, &src_rect, &dst_rect);

        at_x += char_w;
    }
}

static int
mu_u32_slider(mu_Context* ctx, u32* value, u32 low, u32 high) {
    f32 temp = (f32)*value;
    int result = mu_slider_ex(ctx, &temp, (f32)low, (f32)high, 0, "%.0f", MU_OPT_ALIGNCENTER);
    if (result & MU_RES_CHANGE) {
        *value = (u32)temp;
    }
    return result;
}

static void
mu_popup_window_with_button(mu_Context* mu_ctx, const char* button_title, const char* window_title, mu_Rect rect, const char* window_contents) {
    {
        int width = 10 + mu_text_width_callback(0, button_title, -1);
        mu_layout_row(mu_ctx, 1, &width, 0);
    }
    if (mu_button(mu_ctx, button_title)) {
        mu_Container* help_window = mu_get_container(mu_ctx, window_title);
        help_window->open = true;
    }

    if (mu_begin_window_ex(mu_ctx, window_title, rect, MU_OPT_CLOSED)) {
        int width = -1;
        mu_layout_row(mu_ctx, 1, &width, 0);
        mu_text(mu_ctx, window_contents);

        mu_end_window(mu_ctx);
    }
}

int
SDL_main(int argument_count, char** arguments) {
    //
    // NOTE: Platform Setup
    //

    platform_init();

    int w          = 1024;
    int h          = 576;
    b32 fullscreen = false;
    f32 max_dt     = 1.0f / 10.0f;

    int max_threads = INT_MAX;
#if _DEBUG
    max_threads = 1;
    w = 520;
    h = 480;
#endif

    int cpu_count = SDL_GetCPUCount();

    // NOTE: Give the OS a little more threads to play with. This is just a theory,
    //       but I imagine that if you create exactly the number of threads to match
    //       your logical cores, it will be less likely to always schedule all of them
    //       on all your cores optimally.
    // TODO: Profile to see how this affects performance (as of writing I have literally
    //       no way to profile or benchmark anything, it's all messed up).
    cpu_count += cpu_count / 4;

    if (cpu_count > max_threads) {
        cpu_count = max_threads;
    }

    SDL_Init(SDL_INIT_VIDEO|SDL_INIT_EVENTS);
    SDL_Window* window = SDL_CreateWindow("Hello Raytracer", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, w, h, SDL_WINDOW_SHOWN);

    SDL_bool capture_input = SDL_FALSE;
    SDL_SetRelativeMouseMode(capture_input);

    SDL_Renderer* renderer   = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED|SDL_RENDERER_PRESENTVSYNC);
    SDL_Texture*  backbuffer = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_BGRA32, SDL_TEXTUREACCESS_STREAMING, w, h);

    //
    // NOTE: Program setup
    //

    Arena permanent_arena = {};
    Arena transient_arena = {};
    Arena frame_arena     = {};

    mu_Context* mu_ctx = push_struct(&permanent_arena, mu_Context);
    mu_init(mu_ctx);

    mu_ctx->text_width  = mu_text_width_callback;
    mu_ctx->text_height = mu_text_height_callback;

    mu_button_map[SDL_BUTTON_LEFT   & 0xff] = MU_MOUSE_LEFT;
    mu_button_map[SDL_BUTTON_RIGHT  & 0xff] = MU_MOUSE_RIGHT;
    mu_button_map[SDL_BUTTON_MIDDLE & 0xff] = MU_MOUSE_MIDDLE;

    mu_key_map   [SDLK_LSHIFT       & 0xff] = MU_KEY_SHIFT;
    mu_key_map   [SDLK_RSHIFT       & 0xff] = MU_KEY_SHIFT;
    mu_key_map   [SDLK_RSHIFT       & 0xff] = MU_KEY_SHIFT;
    mu_key_map   [SDLK_LCTRL        & 0xff] = MU_KEY_CTRL;
    mu_key_map   [SDLK_RCTRL        & 0xff] = MU_KEY_CTRL;
    mu_key_map   [SDLK_LALT         & 0xff] = MU_KEY_ALT;
    mu_key_map   [SDLK_RALT         & 0xff] = MU_KEY_ALT;
    mu_key_map   [SDLK_RETURN       & 0xff] = MU_KEY_RETURN;
    mu_key_map   [SDLK_BACKSPACE    & 0xff] = MU_KEY_BACKSPACE;

    Font font = {};
    font.surface = SDL_LoadBMP("data/font8x12.bmp");
    font.texture = SDL_CreateTextureFromSurface(renderer, font.surface);

    load_blue_noise_textures();

    //
    // NOTE: Scene setup
    //

    Scene scene = {};
    load_scene(&scene, &g_scenes[3], w, h, &transient_arena);

    Camera* camera = &scene.new_camera;
    SceneSettings* settings = &scene.new_settings;

    RenderParameters parameters = {};
    parameters.scene       = &scene;
    parameters.backbuffer  = allocate_accumulation_buffer(&transient_arena, w, h);
    parameters.frontbuffer = allocate_accumulation_buffer(&transient_arena, w, h);
    parameters.path_guide  = allocate_path_guide(&transient_arena, w, h);

    // NOTE: alignas 64 to align the queue to a cache line to avoid false sharing, is the theory
    //       but I neither understand false sharing very well, nor do I know how to actually discover
    //       if it is happening, so I should research this properly.
    alignas(64) WorkQueue queue;
    init_work_queue(&queue,
                    cpu_count,
                    &permanent_arena,
                    parameters,
                    64, 64);

    WRITE_BARRIER;

    //
    // NOTE: Main loop
    //

    f32 dt = 16.66666f / 1000.0f;

    struct Button {
        b32 pressed;
        b32 down;
    };

    struct Input {
        Button forward;
        Button left;
        Button right;
        Button back;
        Button up;
        Button down;
        Button sprint;

        Button lmb;
        Button mmb;
        Button rmb;
    } input = {};

    b32 flying          = true;
    b32 supported       = false;
    V3  support_surface = {};
    f32 camera_dy       = 0.0f;

    FrameHistory  frame_time_history = {};
    FrameHistory render_time_history = {};

    PlatformHighResTime clock_start = platform_get_timestamp();

    b32 running = true;
    while (running) {
        transient_arena.check();
        frame_arena.clear();

        //
        // NOTE: Handle events
        //

        ForStructAsArrayOf(Button, button, &input) {
            button->pressed = false;
        }

        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            switch (e.type) {
                case SDL_QUIT: {
                    running = false;
                } break;

                case SDL_MOUSEWHEEL: {
                    mu_input_scroll(mu_ctx, 0, e.wheel.y * -30);
                } break;

                case SDL_TEXTINPUT: {
                    mu_input_text(mu_ctx, e.text.text);
                } break;

                case SDL_MOUSEMOTION: {
                    if (capture_input) {
                        s32 dxi = e.motion.xrel;
                        s32 dyi = e.motion.yrel;
                        f32 dx = (f32)dxi / (f32)w;
                        f32 dy = (f32)dyi / (f32)h;
                        V3 aim_adjust = dx*camera->x + dy*camera->y;
                        V3 aim_d = camera->z + aim_adjust;
                        aim_d.y = clamp(aim_d.y, -0.9f, 0.9f);
                        aim_camera(camera, aim_d);
                    } else {
                        mu_input_mousemove(mu_ctx, e.motion.x, e.motion.y);
                    }
                } break;

                case SDL_MOUSEBUTTONDOWN:
                case SDL_MOUSEBUTTONUP: {
                    int b = mu_button_map[e.button.button & 0xff];
                    if (b && e.type == SDL_MOUSEBUTTONDOWN) {
                        mu_input_mousedown(mu_ctx, e.button.x, e.button.y, b);
                    }
                    if (b && e.type == SDL_MOUSEBUTTONUP  ) {
                        mu_input_mouseup(mu_ctx, e.button.x, e.button.y, b);
                    }

                    b32 pressed = (e.type == SDL_MOUSEBUTTONDOWN);
                    switch (e.button.button) {
                        case SDL_BUTTON_LEFT:   { input.lmb = { pressed, pressed }; } break;
                        case SDL_BUTTON_MIDDLE: { input.mmb = { pressed, pressed }; } break;
                        case SDL_BUTTON_RIGHT:  { input.rmb = { pressed, pressed }; } break;
                    }
                } break;

                case SDL_KEYUP:
                case SDL_KEYDOWN: {
                    b32 pressed = (e.key.state == SDL_PRESSED);

                    int c = mu_key_map[e.key.keysym.sym & 0xff];
                    if (c &&  pressed) { mu_input_keydown(mu_ctx, c); }
                    if (c && !pressed) { mu_input_keyup(mu_ctx, c);   }

                    switch (e.key.keysym.sym) {
                        case SDLK_ESCAPE: {
                            if (e.key.state == SDL_RELEASED) {
                                running = false;
                            }
                        } break;

                        case SDLK_w:      { input.forward = { pressed, pressed }; } break;
                        case SDLK_a:      { input.left    = { pressed, pressed }; } break;
                        case SDLK_s:      { input.back    = { pressed, pressed }; } break;
                        case SDLK_d:      { input.right   = { pressed, pressed }; } break;
                        case SDLK_SPACE:  { input.up      = { pressed, pressed }; } break;
                        case SDLK_LCTRL:  { input.down    = { pressed, pressed }; } break;
                        case SDLK_LSHIFT: { input.sprint  = { pressed, pressed }; } break;

                        case SDLK_RETURN: {
                            if (pressed && (e.key.keysym.mod & (KMOD_LALT|KMOD_RALT))) {
                                fullscreen = !fullscreen;
                                SDL_SetWindowFullscreen(window, fullscreen ? SDL_WINDOW_FULLSCREEN_DESKTOP : 0);
                            }
                        } break;

                        case SDLK_v: {
                            if (capture_input && pressed) {
                                flying = !flying;
                            }
                        } break;

                        case SDLK_TAB: {
                            if (pressed) {
                                mu_Container* wnd = mu_get_container(mu_ctx, "Control Panel");
                                if (wnd) {
                                    wnd->open = !wnd->open;
                                }
                            }
                        } break;
                    }
                } break;
            }
        }

        if (input.down.down && input.lmb.down) {
            int mouse_x, mouse_y;
            SDL_GetMouseState(&mouse_x, &mouse_y);

            f32 u = 1.0f - 2.0f*((f32)mouse_x / (f32)w);
            f32 v = 1.0f - 2.0f*((f32)mouse_y / (f32)h);
            apply_lens_distortion(scene.settings.lens_distortion, w, h, &u, &v);

            Ray ray = make_ray(camera->p, get_ray_d(camera, u, v));

            f32 t = ray.max_t;
            V3 hit_p;
            V3 N;
            if (intersect_scene(&scene, ray, &t, &hit_p, &N)) {
                camera->focus_distance = t;
            } 
        }

        if (input.rmb.pressed) {
            capture_input = (capture_input == SDL_TRUE ? SDL_FALSE : SDL_TRUE);
            SDL_SetRelativeMouseMode(capture_input);
        }

        //
        // NOTE: World sim
        //

        V3 camera_move = {};

        if (capture_input) {
            if (input.forward.down) { camera_move -= 10.0f*dt*camera->z; }
            if (input.left.down)    { camera_move += 10.0f*dt*camera->x; }
            if (input.right.down)   { camera_move -= 10.0f*dt*camera->x; }
            if (input.back.down)    { camera_move += 10.0f*dt*camera->z; }

            if (flying) {
                if (input.up.down)   { camera_move += 8.0f*dt*camera->y; }
                if (input.down.down) { camera_move -= 8.0f*dt*camera->y; }
            }
        }

        if (input.sprint.down) {
            camera_move *= 2;
        }

        if (flying) {
            camera->p += camera_move;
            camera_dy = 0.0f;
        } else {
            if (supported && input.up.pressed) {
                supported = false;
                camera_dy = 8.0f;
            }

            camera_move -= support_surface*dot(support_surface, camera_move);
            camera->p += camera_move;

            V3 ref_camera_p = (camera->p - v3(0, 2, 0));

            if (!supported) {
                camera_dy -= dt*9.8f;
            }

            V3 delta = camera_move + dt*v3(0, camera_dy, 0);
            V3 delta_direction = noz(delta);

            f32 far_clip = length(delta);
            Ray ray = make_ray(ref_camera_p, delta_direction, far_clip);

            f32 t = far_clip;
            V3 hit_p;
            V3 N;
            if (intersect_scene(&scene, ray, &t, &hit_p, &N)) {
                camera_dy = 0.0f;
                delta = t*delta_direction;
                supported = true;
                support_surface = N;
            } 

            camera->p += delta;
        }

        scene.t += dt;

        static b32 taking_picture = false;
        static u32 picture_spp = 256;
        static char picture_file_name[256] = "cool_pic.bmp";

        //
        // NOTE: UI
        //

        TemporaryMemory ui_temp = begin_temporary_memory(&frame_arena);

        static b32 debug_show_noise = false;
        static b32 debug_plot_noise = false;
        static b32 debug_histogram_noise = false;
        static u32 debug_histogram_sample_count = 64;
        static u32 debug_seed = 0;
        static u32 debug_histogram_width = 256;
        static b32 debug_stratified_noise = true;
        b32 reset_noise_histogram = false;

        static TraversalStats stats;

        PostProcessSettings* post_settings = &scene.post_settings;

        mu_begin(mu_ctx);

        if (mu_begin_window(mu_ctx, "Control Panel", mu_rect(10, 10, 320, h - 20))) {
            mu_popup_window_with_button(mu_ctx, "Show Help/Controls", "Help", mu_rect(340, 10, w - 350, h - 20), g_about_help_controls);

            if (mu_header(mu_ctx, "Scenes")) {
                mu_label(mu_ctx, aprintf(&frame_arena, "Current: %s", scene.name ? scene.name : "None"));
                for (usize i = 0; i < ArrayCount(g_scenes); ++i) {
                    SceneDescription* scene_desc = &g_scenes[i];
                    if (mu_button(mu_ctx, scene_desc->name)) {
                        discard_current_render(&queue);
                        load_scene(&scene, scene_desc, w, h, &transient_arena);
                    }
                }
            }

            if (mu_header(mu_ctx, "Sampling Strategy")) {
                mu_label(mu_ctx, aprintf(&frame_arena, "Current: %s", to_string(settings->sampling_strategy)));
                for (usize i = 0; i < SamplingStrategy_COUNT; ++i) {
                    SamplingStrategy strategy = (SamplingStrategy)i;
                    if (mu_button(mu_ctx, to_string(strategy))) {
                        settings->sampling_strategy = strategy;
                        reset_noise_histogram = true;
                    }
                }
                mu_popup_window_with_button(mu_ctx, "About", "About - Sampling Strategies", mu_rect(340, 10, w - 350, h - 20), g_about_sampling_strategies);
            }

            if (mu_header(mu_ctx, "Integrators")) {
                mu_label(mu_ctx, aprintf(&frame_arena, "Current: %s", settings->integrator->name));
                for (usize i = 0; i < g_integrator_count; ++i) {
                    IntegratorOption* option = &g_integrators[i];
                    if (mu_button(mu_ctx, option->name)) {
                        settings->integrator = option;
                    }
                }
                mu_popup_window_with_button(mu_ctx, "About", "About - Integrators", mu_rect(340, 10, w - 350, h - 20), g_about_integrators);
            }

            if (mu_header(mu_ctx, "Reconstruction Filters")) {
                mu_label(mu_ctx, aprintf(&frame_arena, "Current: %s", g_filter_cache.filter.name));
                for (usize i = 0; i < g_filter_count; ++i) {
                    FilterKernelOption* option = &g_filters[i];
                    if (mu_button(mu_ctx, option->name)) {
                        discard_current_render(&queue);
                        load_reconstruction_kernel(option);
                    }
                }
                mu_popup_window_with_button(mu_ctx, "About", "About - Reconstruction Filters", mu_rect(340, 10, w - 350, h - 20), g_about_reconstruction_filters);
            }

            if (mu_header(mu_ctx, "Settings")) {
                mu_label(mu_ctx, aprintf(&frame_arena, "Samples Per Pixel: %u", settings->samples_per_pixel));

                mu_label(mu_ctx, aprintf(&frame_arena, "Max Bounce Count: %u", settings->max_bounce_count));
                mu_u32_slider(mu_ctx, &settings->max_bounce_count, 1, 32);

                mu_checkbox(mu_ctx, "Next Event Estimation", &settings->next_event_estimation);
                mu_checkbox(mu_ctx, "Importance Sample Lights", &settings->importance_sample_lights);
                mu_checkbox(mu_ctx, "Importance Sample Diffuse", &settings->importance_sample_diffuse);
                mu_checkbox(mu_ctx, "Use Multiple Importance Sampling", &settings->use_mis);
                mu_checkbox(mu_ctx, "Russian Roulette", &settings->russian_roulette);
                // mu_checkbox(mu_ctx, "Use Path Guide", &settings->use_path_guide);
                mu_checkbox(mu_ctx, "Caustics", &settings->caustics);

                mu_label(mu_ctx, "Vignette Strength");
                mu_slider(mu_ctx, &settings->vignette_strength, 0.0f, 1.0f);

                mu_label(mu_ctx, "Lens Distortion");
                mu_slider(mu_ctx, &settings->lens_distortion, -3.0f, 6.0f);

                f32 vfov_degrees = RAD_TO_DEG*camera->vfov;

                mu_label(mu_ctx, "Vertical Field of View");
                if (mu_slider(mu_ctx, &vfov_degrees, 15.0f, 120.0f)) {
                    camera->vfov = DEG_TO_RAD*vfov_degrees;
                }

                mu_label(mu_ctx, "Lens Radius");
                mu_slider(mu_ctx, &camera->lens_radius, 0.0f, 25.0f);

                mu_label(mu_ctx, "F-Factor");
                mu_slider(mu_ctx, &settings->f_factor, 0.0f, 1.0f);

                mu_label(mu_ctx, "Diahpragm Edge Count");
                mu_slider(mu_ctx, &settings->diaphragm_edges, 4.0f, 8.0f);

                mu_label(mu_ctx, "Phi Shutter Max");
                mu_slider(mu_ctx, &settings->phi_shutter_max, 0.0f, 2.0f);

                mu_label(mu_ctx, "Focus Distance");
                mu_slider(mu_ctx, &camera->focus_distance, 0.1f, 100.0f);
            }

            if (mu_header(mu_ctx, "Post Processing")) {
                mu_label(mu_ctx, "Exposure");
                mu_slider(mu_ctx, &post_settings->exposure, -4.0f, 4.0f);

                mu_checkbox(mu_ctx, "Tonemapping", &post_settings->tonemapping);
                mu_checkbox(mu_ctx, "SRGB Transform", &post_settings->srgb_transform);

                mu_label(mu_ctx, "Contrast Midpoint");
                mu_slider(mu_ctx, &post_settings->midpoint, 0.0f, 1.0f);

                mu_label(mu_ctx, "Contrast");
                mu_slider(mu_ctx, &post_settings->contrast, -1.0f, 1.0f);

                if (mu_button(mu_ctx, "Reset")) {
                    post_settings->exposure = 0.0f;
                    post_settings->midpoint = 0.5f;
                    post_settings->contrast = 0.0f;
                }
            }

            if (mu_header(mu_ctx, "Picture Taking")) {
                mu_label(mu_ctx, "Samples Per Pixel");
                mu_u32_slider(mu_ctx, &picture_spp, 1, 2*8192);

                mu_textbox(mu_ctx, picture_file_name, ArrayCount(picture_file_name));

                if (mu_button(mu_ctx, "Take picture")) {
                    taking_picture = true;
                    discard_current_render(&queue);
                    settings->samples_per_pixel = picture_spp;
                }

                if (taking_picture) {
                    u32 total_tile_count   = queue.total_tile_count;
                    s32 retired_tile_count = queue.tiles_retired;
                    mu_label(mu_ctx, aprintf(&frame_arena, "Tiles retired: %d/%u", retired_tile_count, total_tile_count));
                }
            }

            if (mu_header(mu_ctx, "Random Stats")) {
                mu_text(mu_ctx, aprintf(&frame_arena, "Mesh intersection count:  %llu", stats.mesh_intersection_count));
                mu_text(mu_ctx, aprintf(&frame_arena, "Mesh BVH traversals:      %llu", stats.mesh_bvh_traversals));
                mu_text(mu_ctx, aprintf(&frame_arena, "Mesh BVH Node traversals: %llu", stats.mesh_node_traversals));
                mu_text(mu_ctx, aprintf(&frame_arena, "Mesh BVH Leaf traversals: %llu", stats.mesh_leaf_traversals));
                mu_text(mu_ctx, aprintf(&frame_arena, "Mesh BVH Leaf traversals per traversal: %f",
                                        (f64)stats.mesh_leaf_traversals / (f64)stats.mesh_bvh_traversals));
            }

            if (mu_header(mu_ctx, "Sampling Debug Visualizations")) {
                mu_checkbox(mu_ctx, "Show Noise", &debug_show_noise);
                mu_checkbox(mu_ctx, "Plot Noise", &debug_plot_noise);
                mu_checkbox(mu_ctx, "Show Noise Histogram", &debug_histogram_noise);
                mu_text(mu_ctx, "Sample Count");
                mu_u32_slider(mu_ctx, &debug_histogram_sample_count, 1, 8192);
                mu_text(mu_ctx, "Bucket Count");
                mu_u32_slider(mu_ctx, &debug_histogram_width, 8, 1024);
                mu_text(mu_ctx, "Seed");
                mu_u32_slider(mu_ctx, &debug_seed, 0, 256);
            }

            if (mu_button(mu_ctx, "Cancel Render")) {
                discard_current_render(&queue);
                settings->samples_per_pixel = 1;
                taking_picture = false;
            }

            mu_end_window(mu_ctx);
        }

        mu_end(mu_ctx);

        end_temporary_memory(ui_temp);

        //
        // NOTE: Render
        //

        f64 render_time;
        b32 done_with_frame = render_all_tiles(&queue, &render_time, &stats);
        AccumulationBuffer* hdr_buffer = (taking_picture ? queue.parameters.backbuffer
                                                         : queue.parameters.frontbuffer);

        //
        // NOTE: Output
        //

        void* pixels;
        int pitch;
        SDL_LockTexture(backbuffer, 0, &pixels, &pitch);

        // debug_visualize_filter_cache(&scene.filter_cache, hdr_buffer, &frame_arena);

        if (taking_picture || done_with_frame) {
            RandomSeries last_entropy = random_seed(scene.total_frame_index - 1);
            RandomSeries      entropy = random_seed(scene.total_frame_index);

            Image_R8G8B8* dither_noise = &g_blue_noise3[scene.total_frame_index % ArrayCount(g_blue_noise3)];

            u32* dest = (u32*)pixels;
            usize i = 0;
            for (usize y = 0; y < (usize)h; ++y)
            for (usize x = 0; x < (usize)w; ++x, ++i) {
                V4 sample = hdr_buffer->pixels[i];
                V3 hdr_color = v3(0.0f);

                if ((sample.x != sample.x) ||
                    (sample.y != sample.y) ||
                    (sample.z != sample.z) ||
                    (sample.w != sample.w))
                {
                    // NOTE: Uh oh, Nan!
                    hdr_color = v3(0, 255, 255);
                } else if (sample.w > 0.001f) {
                    hdr_color = sample.xyz / sample.w;
                    hdr_color = max(hdr_color, v3(0.0f));

                    if (post_settings->exposure != 0.0f) {
                        hdr_color *= powf(2, post_settings->exposure);
                    }

					// NOTE: Tonemap
                    if (post_settings->tonemapping) {
                        hdr_color.x = 1.0f - expf(-hdr_color.x);
                        hdr_color.y = 1.0f - expf(-hdr_color.y);
                        hdr_color.z = 1.0f - expf(-hdr_color.z);
                    }

					// NOTE: SRGB Transform
                    if (post_settings->srgb_transform) {
                        hdr_color.x = powf(hdr_color.x, 1.0f / 2.23333f);
                        hdr_color.y = powf(hdr_color.y, 1.0f / 2.23333f);
                        hdr_color.z = powf(hdr_color.z, 1.0f / 2.23333f);
                    }

                    if (post_settings->contrast != 0.0f) {
                        hdr_color.x = sigmoidal_contrast(hdr_color.x, post_settings->contrast, post_settings->midpoint);
                        hdr_color.y = sigmoidal_contrast(hdr_color.y, post_settings->contrast, post_settings->midpoint);
                        hdr_color.z = sigmoidal_contrast(hdr_color.z, post_settings->contrast, post_settings->midpoint);
                    }

					hdr_color *= 255.0f;

                    usize rel_x = x & (dither_noise->w - 1);
                    usize rel_y = y & (dither_noise->h - 1);
                    Color_R8G8B8 dither_s = dither_noise->pixels[rel_y*dither_noise->w + rel_x];
                    V3 dither = {
                        0.5f + remap_tpdf((1.0f/255.0f)*(f32)dither_s.r),
                        0.5f + remap_tpdf((1.0f/255.0f)*(f32)dither_s.g),
                        0.5f + remap_tpdf((1.0f/255.0f)*(f32)dither_s.b),
                    };

					hdr_color += dither;
                } else if (sample.w < -0.01f) {
                    hdr_color = v3(-255.0f*sample.w, 0.0f, -255.0f*sample.w);
                }

                u8 r = (u8)clamp(hdr_color.x, 0.0f, 255.0f);
                u8 g = (u8)clamp(hdr_color.y, 0.0f, 255.0f);
                u8 b = (u8)clamp(hdr_color.z, 0.0f, 255.0f);

                *dest++ = (255 << 24)|(r << 16)|(g << 8)|b;
            }
        }

        if (done_with_frame && taking_picture) {
            write_bitmap((u32*)pixels, w, h, picture_file_name, &transient_arena);
            printf("Took %ux%u %uspp image in %f seconds.\n",
                   w, h, scene.settings.samples_per_pixel,
                   render_time);

            taking_picture = false;
            settings->samples_per_pixel = 1;
            // FIXME: BIG OMEGA HACK TO AVOID DISCARDING THE RENDER AFTER THE PICTURE IS DONE!!!
            scene.settings.samples_per_pixel = 1;
        }

        SDL_UnlockTexture(backbuffer);

        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
        SDL_RenderSetClipRect(renderer, 0);

        SDL_RenderCopy(renderer, backbuffer, 0, 0);

        RandomSeries debug_entropy = random_seed(debug_seed);
        Sampler sampler = {};
        sampler.entropy = &debug_entropy;
        sampler.strategy = settings->sampling_strategy;

        if (debug_show_noise) {
            if (debug_plot_noise) {
                {
                    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
                    SDL_Rect rect = { w - 256, 0, 256, 256 };
                    SDL_RenderFillRect(renderer, &rect);
                }

                SDL_SetRenderDrawColor(renderer, 128, 128, 128, 255);

                for (usize i = 0; i < debug_histogram_sample_count; ++i) {
                    sampler.sample_index = (u32)i;
                    V2 sample = get_next_sample_2d(&sampler, Sample_IndirectLighting, 0);
                    SDL_Rect sample_rect;
                    sample_rect.x = w - 256 + (int)(sample.x*256.0f) - 1;
                    sample_rect.y = (int)(sample.y*256.0f) - 1;
                    sample_rect.w = 3;
                    sample_rect.h = 3;
                    SDL_RenderFillRect(renderer, &sample_rect);
                }
            } else {
                static int sample_index = 0;

                SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
                SDL_Rect rect = { w - 256, 0, 256, 256 };
                SDL_RenderFillRect(renderer, &rect);

                for (int y = 0; y < 256; ++y)
                for (int x = 0; x < 256; ++x) {
                    SDL_Rect sample_rect = {
                        w - 256 + x, y,
                        1, 1,
                    };

                    sampler.sample_index = sample_index;
                    sampler.x = x;
                    sampler.y = y;
                    V2 sample = get_next_sample_2d(&sampler, Sample_IndirectLighting, 0);

                    SDL_SetRenderDrawColor(renderer, (u8)(sample.x*255.0f), (u8)(sample.y*255.0f), 0, 255);
                    SDL_RenderFillRect(renderer, &sample_rect);
                }
            }
        }

        if (debug_histogram_noise) {
            int* buckets = push_array(&frame_arena, debug_histogram_width, int);

            for (usize i = 0; i < debug_histogram_sample_count; ++i) {
                sampler.sample_index = (u32)i;
                sampler.x = 0;
                sampler.y = 0;
                V2 sample = get_next_sample_2d(&sampler, Sample_IndirectLighting, 0);
                f32 k = (f32)debug_histogram_width - EPSILON;
                u32 bucket_x = (u32)(k*sample.x);
                u32 bucket_y = (u32)(k*sample.y);
                buckets[bucket_x] += 1;
                buckets[bucket_y] += 1;
            }

            const int graph_x = w - 256;
            const int graph_y = 256;
            const int graph_w = Max(256, debug_histogram_width);
            const int graph_h = 256;

            {
                SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
                SDL_Rect rect = { graph_x, graph_y, graph_w, graph_h };
                SDL_RenderFillRect(renderer, &rect);
            }

            const int bucket_w = 1;
            const int bucket_h = graph_h;

            int peak = {};
            for (u32 i = 0; i < debug_histogram_width; ++i) {
                peak = Max(peak, buckets[i]);
            }

            for (u32 i = 0; i < debug_histogram_width; ++i) {
                int x = bucket_h*buckets[i] / peak;

                SDL_Rect rect;
                rect.x = graph_x + i*bucket_w;
                rect.y = 2*graph_y - x;
                rect.w = bucket_w;
                rect.h = x;

                SDL_SetRenderDrawColor(renderer, 128, 128, 128, 255);
                SDL_RenderFillRect(renderer, &rect);
            }
        }

        mu_Command* cmd = 0;
        while (mu_next_command(mu_ctx, &cmd)) {
            switch (cmd->type) {
                case MU_COMMAND_TEXT: {
                    Color_BGRA color = color_bgra(cmd->text.color.r,
                                                  cmd->text.color.g,
                                                  cmd->text.color.b,
                                                  cmd->text.color.a);
                    render_text(renderer,
                                &font,
                                (unsigned char*)cmd->text.str,
                                cmd->text.pos.x,
                                cmd->text.pos.y,
                                color);
                } break;

                case MU_COMMAND_ICON: {
                    unsigned char c = 0;
                    switch (cmd->icon.id) {
                        case MU_ICON_CLOSE    : { c = 229; } break;
                        case MU_ICON_CHECK    : { c = 229; } break;
                        case MU_ICON_COLLAPSED: { c =  31; } break;
                        case MU_ICON_EXPANDED : { c =  30; } break;
                        case MU_ICON_MAX      : { c =   1; } break; // TODO: I don't know where this is used, so make it a smiley
                    }
                    if (c) {
                        Color_BGRA color = color_bgra(cmd->icon.color.r,
                                                      cmd->icon.color.g,
                                                      cmd->icon.color.b,
                                                      cmd->icon.color.a);

                        s32 slop_x = Max(0, cmd->icon.rect.w -  8) / 2;
                        s32 slop_y = Max(0, cmd->icon.rect.h - 12) / 2;

                        unsigned char buf[2] = { c, 0 };
                        render_text(renderer,
                                    &font,
                                    buf,
                                    slop_x + cmd->icon.rect.x,
                                    slop_y + cmd->icon.rect.y,
                                    color);
                    }
                } break;

                case MU_COMMAND_RECT: {
                    SDL_SetRenderDrawColor(renderer,
                                           cmd->rect.color.r,
                                           cmd->rect.color.g,
                                           cmd->rect.color.b,
                                           cmd->rect.color.a);

                    SDL_Rect rect;
                    rect.x = cmd->rect.rect.x;
                    rect.y = cmd->rect.rect.y;
                    rect.w = cmd->rect.rect.w;
                    rect.h = cmd->rect.rect.h;
                    SDL_RenderFillRect(renderer, &rect);
                } break;

                case MU_COMMAND_CLIP: {
                    SDL_Rect rect;
                    rect.x = cmd->clip.rect.x;
                    rect.y = cmd->clip.rect.y;
                    rect.w = cmd->clip.rect.w;
                    rect.h = cmd->clip.rect.h;
                    SDL_RenderSetClipRect(renderer, &rect);
                } break;
            }
        }

        SDL_RenderPresent(renderer);

        //
        // NOTE: Metrics
        //

        PlatformHighResTime clock_end = platform_get_timestamp();
        f64 frame_time = platform_get_seconds_elapsed(clock_start, clock_end);
        clock_start = clock_end;

        record_frame_time(&frame_time_history, frame_time);

        if (done_with_frame) {
            record_frame_time(&render_time_history, render_time);
        }

        FrameTimingInfo frame_timing  = get_frame_timing_info(&frame_time_history);
        FrameTimingInfo render_timing = get_frame_timing_info(&render_time_history);

        char* title_string = aprintf(&frame_arena, "\rspp: %u fps: %f, time: min: %fms, average: %fms, max: %fms",
                                                   hdr_buffer->frame_count,
                                                   1.0 / render_timing.average,
                                                   1000.0*render_timing.min,
                                                   1000.0*render_timing.average,
                                                   1000.0*render_timing.max);
        SDL_SetWindowTitle(window, title_string);

        dt = min((f32)frame_time, max_dt);
    }

    discard_current_render(&queue);

    permanent_arena.check();

    return 0;
}
