#pragma once

#define FILTER_KERNEL(name) f32 name(f32 x)
typedef FILTER_KERNEL(FilterFunction);

struct FilterKernelOption {
    const char* name;
    FilterFunction* f;
    u32 radius;
};

extern FilterKernelOption g_filters[];
extern u32 g_filter_count;

//

FilterKernelOption* find_filter(const char* name);
