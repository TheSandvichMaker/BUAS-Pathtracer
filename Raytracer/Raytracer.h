#pragma once

//
// NOTE: Mathsy raytracing related stuff
//

union Color_BGRA {
    struct {
        u8 b, g, r, a;
    };
    u32 bgra;
};

inline Color_BGRA
color_bgra(u8 r, u8 g, u8 b, u8 a = 255) {
    Color_BGRA result;
    result.r = r;
    result.g = g;
    result.b = b;
    result.a = a;
    return result;
}

//

#define SCENE_DESCRIPTION(name) void name(Scene* scene, u32 w, u32 h, Arena* temp_arena, Camera* camera, SceneSettings* settings, PostProcessSettings* post_settings)
typedef SCENE_DESCRIPTION(SceneDescriptionFunction);

struct SceneDescription {
    const char* name;
    SceneDescriptionFunction* f;
};

struct FilterCache {
    FilterKernelOption filter;

    u32 kernel_size;
    u32 cache_size;
    f32 cache[512];
};

FilterCache g_filter_cache;

struct AccumulationBuffer {
    u32 w, h;
    u32 frame_count;
    V4* pixels;
};

struct RenderParameters {
    Scene* scene;
    AccumulationBuffer* backbuffer;
    AccumulationBuffer* frontbuffer;
    ScreenspacePathGuide* path_guide;
};

struct FrameHistory {
    u32 write_cursor;
    f64 clocks[15];
};

struct FrameTimingInfo {
    f64 min;
    f64 average;
    f64 max;
};

const usize THREAD_TASK_ARENA_SIZE = Kilobytes(32);

struct WorkQueue {
    volatile b32 discard_render;

    RenderParameters parameters;

    PlatformHighResTime render_start_time;

    u32 tile_w;
    u32 tile_h;
    u32 total_tile_count;
    u32 tile_count_x;
    u32 tile_count_y;

    Arena main_thread_task_arena;

    u32 thread_count;
    volatile u32 threads_in_flight;
    volatile s32 tile_index;
    volatile s32 tiles_retired;

    PlatformSemaphore semaphore;
};
