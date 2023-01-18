#include "precomp.h"
#include "common.h"

#include "reconstruction_filters.h"

// REFERENCE: http://www.pbr-book.org/3ed-2018/Sampling_and_Reconstruction/Image_Reconstruction.html

inline f32
sinc(f32 x) {
    return sinf(PI_32*x) / (PI_32*x);
}

#define LANCZOS3_FILTER_RADIUS 3
FILTER_KERNEL(lanczos3_filter) {
    x = absolute_value(x);
    if (x < 0.0001f) {
        return 1.0f;
    } else if (x <= 3.0f) {
        return sinc(x)*sinc(x / 3.0f);
    }
    return 0.0f;
}

#define LANCZOS4_FILTER_RADIUS 4
FILTER_KERNEL(lanczos4_filter) {
    x = absolute_value(x);
    if (x < 0.0001f) {
        return 1.0f;
    } else if (x <= 4.0f) {
        return sinc(x)*sinc(x / 4.0f);
    }
    return 0.0f;
}

#define LANCZOS6_FILTER_RADIUS 6
FILTER_KERNEL(lanczos6_filter) {
    x = absolute_value(x);
    if (x < 0.0001f) {
        return 1.0f;
    } else if (x <= 6.0f) {
        return sinc(x)*sinc(x / 6.0f);
    }
    return 0.0f;
}

#define LANCZOS12_FILTER_RADIUS 12
FILTER_KERNEL(lanczos12_filter) {
    x = absolute_value(x);
    if (x < 0.0001f) {
        return 1.0f;
    } else if (x <= 12.0f) {
        return sinc(x)*sinc(x / 12.0f);
    }
    return 0.0f;
}

#define GAUSSIAN3_FILTER_RADIUS 3
FILTER_KERNEL(gaussian3_filter) {
    f32 alpha  = 3.0f;
    f32 radius = 3.0f;
    f32 radius_exp = exp(-alpha*radius*radius);

    f32 result = max(0.0f, expf(-alpha*x*x) - radius_exp);
    return result;
}

#define GAUSSIAN12_FILTER_RADIUS 12
FILTER_KERNEL(gaussian12_filter) {
    f32 alpha  = 0.03f;
    f32 radius = 12.0f;
    f32 radius_exp = exp(-alpha*radius*radius);

    f32 result = max(0.0f, expf(-alpha*x*x) - radius_exp);
    return result;
}

#define MITCHELL_NETRAVALI_FILTER_RADIUS 2
FILTER_KERNEL(mitchell_netravali_filter) {
    static const f32 B = 1.0f / 3.0f;
    static const f32 C = 1.0f / 3.0f;

    x = absolute_value(x);

    f32 result;
    if (x > 1.0f) {
        result = (((-B - 6*C)*x*x*x + (6*B + 30*C)*x*x +
                   (-12*B - 48*C)*x + (8*B + 24*C))*(1.0f / 6.0f));
    } else {
        result = (((12 - 9*B - 6*C)*x*x*x +
                   (-18 + 12*B + 6*C)*x*x +
                   (6 - 2*B))*(1.0f / 6.0f));
    }

    return result;
}

FilterKernelOption g_filters[] = {
    { "Box",                nullptr,                   0                                },
    { "Gaussian 3",         gaussian3_filter,          GAUSSIAN3_FILTER_RADIUS          },
    { "Gaussian 12",        gaussian12_filter,         GAUSSIAN12_FILTER_RADIUS         },
    { "Mitchell Netravali", mitchell_netravali_filter, MITCHELL_NETRAVALI_FILTER_RADIUS },
    { "Lanczos 3",          lanczos3_filter,           LANCZOS3_FILTER_RADIUS           },
    { "Lanczos 4",          lanczos4_filter,           LANCZOS4_FILTER_RADIUS           },
    { "Lanczos 6",          lanczos6_filter,           LANCZOS6_FILTER_RADIUS           },
    { "Lanczos 12",         lanczos12_filter,          LANCZOS12_FILTER_RADIUS          },
};

u32 g_filter_count = ArrayCount(g_filters);

FilterKernelOption*
find_filter(const char* name) {
    FilterKernelOption* result = &g_filters[0]; // NOTE: If not found, we return the box filter.
    for (usize i = 0; i < g_filter_count; ++i) {
        FilterKernelOption* option = &g_filters[i];
        if (0 == strcmp(name, option->name)) {
            result = option;
            break;
        }
    }
    return result;
}
