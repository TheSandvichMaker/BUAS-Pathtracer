#pragma once

struct IntegratorState {
    Scene* scene;
    Arena* task_arena;
    RandomSeries* entropy;
    Sampler sampler;
    V3 in_ray_o;
    V3 in_ray_d;
    u32 x;
    u32 y;
    b32 out_first_bounce_was_diffuse;
    u32 out_first_bounce_index;
};

#define INTEGRATOR(name) V3 name(IntegratorState* state)
typedef INTEGRATOR(Integrator);

struct IntegratorOption {
    const char* name;
    Integrator* f;
};

extern IntegratorOption g_integrators[];
extern usize g_integrator_count;

//

IntegratorOption* find_integrator(const char* name);
