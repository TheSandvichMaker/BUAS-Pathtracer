#pragma once
#define MY_MATH_H // NOTE: Since we're using pragma once instead of old school include guards,
                  //       let's define something like this separately so you can still check
                  //       whether the library is included with #ifdef MY_MATH_H

#include "math_types.h"

#include <stdint.h>
#include <float.h>
#include <xmmintrin.h>
#include <math.h>

namespace math {

const float PI_32  = 3.14159265359f;
const float TAU_32 = 6.28318530717f;
const float DEG_TO_RAD = (TAU_32 / 360.0f);
const float RAD_TO_DEG = (360.0f / TAU_32);

//
// NOTE: Scalar operations
//

// NOTE:   I need to investigate what correctness checks I am skipping by using SSE directly,
//         but these will inline nicely, unlike calls to sqrtf which will go do whatever.
//         And I know from experience that you can't neglect the relative inefficiency of
//         calling math.h stuff, since I've seen Casey optimize code that spent 90% of its time
//         in roundf() calls for what should be trivial operations.
//         Informally, I can already see it make a difference from ~3.3ms/frame at 1280x720 to
//         ~2.9-3.0ms/frame when comparing sqrtf versus these.
// UPDATE: I've noticed through the unit test and my actual raytracing project that there is
//         notable inaccuracy, so let's revert from using these until I do some more research
//         on how sqrtf() is implemented.
// UPDATE: Obvious in hindsight, I was using rsqrt, which is of course an approximate square
//         root. When comparing sqrtf against _mm_sqrt_ss, there is no obvious speed difference
//         at first glance in my very informal test. Requires more profiling, but not very
//         immediately relevant.
// UPDATE: Upon final review, I discovered the reason why sqrtf() didn't inline
//         to a sqrtss instruction like I wanted was because /fp:precise was
//         set. Switching to /fp:fast produces the codegen I wanted.

inline float
square_root(float x) {
    return sqrtf(x);
}

// BONUS NOTE: _mm_set1_ps is faster than _mm_load_ss, although MSVC and GCC seem bad at optimizing
//             this stuff, clang is the only one who gets it right: https://godbolt.org/z/nGWonE

inline float
fast_approx_inverse_square_root(float x) {
    float result = _mm_cvtss_f32(_mm_rsqrt_ss(_mm_set1_ps(x)));
    return result;
}

inline int32_t
div_floor(int32_t a, int32_t b) {
    int32_t res = a / b;
    int32_t rem = a % b;
    int32_t corr = (rem != 0 && ((rem < 0) != (b < 0)));
    return (res - corr);
}

inline float
square(float x) {
    return x*x;
}

inline float
lerp(float a, float b, float t) {
    return a*(1.0f - t) + b*t;
}

#undef min
#undef max

inline float
min(float a, float b) {
    return a < b ? a : b;
}

inline float
max(float a, float b) {
    return a > b ? a : b;
}

inline double
min(double a, double b) {
    return a < b ? a : b;
}

inline double
max(double a, double b) {
    return a > b ? a : b;
}

inline int
min(int a, int b) {
    return a < b ? a : b;
}

inline int
max(int a, int b) {
    return a > b ? a : b;
}

inline float
clamp(float n, float a, float b) {
    return max(a, min(b, n));
}

inline int
round_to_int(float x) {
    int result = _mm_cvtss_si32(_mm_set1_ps(x));
    return result;
}

inline float
saturate(float n) {
    return clamp(n, 0.0f, 1.0f);
}

inline float
remap(float n, float min, float max) {
    float result = 0.0f;
    float range = (max - min);
    if (range != 0.0f) {
        result = (n - min) / range;
    }
    return result;
}

inline float 
remap_saturate(float n, float min, float max) {
    return saturate(remap(n, min, max));
}

inline float 
safe_ratio_n(float numerator, float divisor, float n) {
    float result = n;
    if (divisor != 0.0f) {
        result = numerator / divisor;
    }
    return result;
}

inline float 
safe_ratio_0(float numerator, float divisor) {
    return safe_ratio_n(numerator, divisor, 0);
}

inline float 
safe_ratio_1(float numerator, float divisor) {
    return safe_ratio_n(numerator, divisor, 1);
}

inline float 
smoothstep(float x) {
    return x*x*(3.0f - 2.0f*x);
}

inline float 
smootherstep(float x) {
    return x*x*x*(x*(x*6.0f - 15.0f) + 10.0f);
}

inline float
sign_of(float x) {
    return x < 0.0f ? -1.0f : 1.0f;
}

inline float
absolute_value(float x) {
#if 0
    int i = (int&)x;
    i &= ~(1 << 31); // sign bit
    return (float&)i;
#else
    // NOTE: fabsf is one of those that gets special treatment in the CRT,
    //       so the codegen is good.
    return fabsf(x);
#endif
}

inline float
copy_sign(float value_of, float sign_of) {
    // NOTE: But copysignf turns into a call, nasty. So have this instead.
    int sign_mask = 1 << 31;
    int sign_as_bits = (int&)sign_of;
    int value_as_bits = (int&)value_of;
    value_as_bits = (sign_as_bits & sign_mask)|(value_as_bits & ~sign_mask);
    float result = (float&)value_as_bits;
    return result;
}


//
// Note: V2
//

// NOTE: Operator overloads

inline V2 
operator - (const V2& a) {
    V2 result;
    result.x = -a.x;
    result.y = -a.y;
    return result;
}

#define IMPLEMENT_VECTOR_OPERATORS(op) \
    inline V2 \
    operator op (const V2& a, const V2& b) { \
        V2 result; \
        result.x = a.x op b.x; \
        result.y = a.y op b.y; \
        return result; \
    } \
    inline V2& \
    operator op##= (V2& a, const V2& b) { \
        a = a op b; \
        return a; \
    }

#define IMPLEMENT_SCALAR_OPERATORS(op) \
    inline V2 \
    operator op (const V2& a, float b) { \
        V2 result; \
        result.x = a.x op b; \
        result.y = a.y op b; \
        return result; \
    } \
    inline V2 \
    operator op (float a, const V2& b) { \
        V2 result; \
        result.x = a op b.x; \
        result.y = a op b.y; \
        return result; \
    } \
    inline V2& \
    operator op##= (V2& a, float b) { \
        a = a op b; \
        return a; \
    }

// IMPLEMENT_SCALAR_OPERATORS(+)
// IMPLEMENT_SCALAR_OPERATORS(-)
IMPLEMENT_VECTOR_OPERATORS(+)
IMPLEMENT_VECTOR_OPERATORS(-)
IMPLEMENT_SCALAR_OPERATORS(*)
IMPLEMENT_SCALAR_OPERATORS(/)
IMPLEMENT_VECTOR_OPERATORS(*)
IMPLEMENT_VECTOR_OPERATORS(/)

#undef IMPLEMENT_VECTOR_OPERATORS
#undef IMPLEMENT_SCALAR_OPERATORS


// NOTE: "Constructor" functions

inline V2 
v2(float x) {
    V2 result;
    result.x = x;
    result.y = x;
    return result;
}

inline V2 
v2(float x, float y) {
    V2 result;
    result.x = x;
    result.y = y;
    return result;
}

// NOTE: Other functions

inline V2 
lerp(const V2& a, const V2& b, float t) {
    return a*(1.0f - t) + b*t;
}

inline float 
dot(const V2& a, const V2& b) {
    return a.x*b.x + a.y*b.y;
}

inline V2 
reflect(const V2& v, const V2& n) {
    return v - 2.0f*dot(v, n)*n;
}

inline float 
length_sq(const V2& a) {
    return dot(a, a);
}

inline float 
length(const V2& a) {
    return square_root(dot(a, a));
}

inline V2 
normalize(const V2& a) {
    float rcp_len = 1.0f / length(a);
    return a*rcp_len;
}

inline V2 
noz(const V2& a) {
    V2 result = {};
    float lsq = length_sq(a);
    if ((lsq > 0.0001f) && (lsq < INFINITY)) {
        result = a / square_root(lsq);
    }
    return result;
}

inline V2
absolute_value(const V2& a) {
    V2 result;
    result.x = absolute_value(a.x);
    result.y = absolute_value(a.y);
    return result;
}

inline
V2 arm2(float angle) {
    V2 result;
    result.x = cos(angle);
    result.y = sin(angle);
    return result;
}

inline
V2 rotate(V2 v, V2 cos_sin) {
    V2 result = v2(v.x*cos_sin.x - v.y*cos_sin.y, v.x*cos_sin.y + v.y*cos_sin.x);
    return result;
}

//
// Note: V3
//

// NOTE: Operator overloads

inline V3 
operator - (const V3& a) {
    V3 result;
    result.x = -a.x;
    result.y = -a.y;
    result.z = -a.z;
    return result;
}

#define IMPLEMENT_VECTOR_OPERATORS(op) \
    inline V3 \
    operator op (const V3& a, const V3& b) { \
        V3 result; \
        result.x = a.x op b.x; \
        result.y = a.y op b.y; \
        result.z = a.z op b.z; \
        return result; \
    } \
    inline V3& \
    operator op##= (V3& a, const V3& b) { \
        a = a op b; \
        return a; \
    }

#define IMPLEMENT_SCALAR_OPERATORS(op) \
    inline V3  \
    operator op (const V3& a, float b) { \
        V3 result; \
        result.x = a.x op b; \
        result.y = a.y op b; \
        result.z = a.z op b; \
        return result; \
    } \
    inline V3  \
    operator op (float a, const V3& b) { \
        V3 result; \
        result.x = a op b.x; \
        result.y = a op b.y; \
        result.z = a op b.z; \
        return result; \
    } \
    inline V3& \
    operator op##= (V3& a, float b) { \
        a = a op b; \
        return a; \
    }

// IMPLEMENT_SCALAR_OPERATORS(+)
// IMPLEMENT_SCALAR_OPERATORS(-)
IMPLEMENT_VECTOR_OPERATORS(+)
IMPLEMENT_VECTOR_OPERATORS(-)
IMPLEMENT_SCALAR_OPERATORS(*)
IMPLEMENT_SCALAR_OPERATORS(/)
IMPLEMENT_VECTOR_OPERATORS(*)
IMPLEMENT_VECTOR_OPERATORS(/)

#undef IMPLEMENT_VECTOR_OPERATORS
#undef IMPLEMENT_SCALAR_OPERATORS


// NOTE: "Constructor" functions

inline V3 
v3(float x, float y, float z) {
    V3 result;
    result.x = x;
    result.y = y;
    result.z = z;
    return result;
}

inline V3 
v3(float x) {
    V3 result;
    result.x = x;
    result.y = x;
    result.z = x;
    return result;
}

inline V3 
v3(V2 xy, float z) {
    V3 result;
    result.x = xy.e[0];
    result.y = xy.e[1];
    result.z = z;
    return result;
}

inline V3 
v3(float x, V2 yz) {
    V3 result;
    result.x = x;
    result.y = yz.e[0];
    result.z = yz.e[1];
    return result;
}

// NOTE: Other functions

inline V3 
lerp(const V3& a, const V3& b, float t) {
    return a*(1.0f - t) + b*t;
}

inline float 
dot(const V3& a, const V3& b) {
    return a.x*b.x + a.y*b.y + a.z*b.z;
}

inline V3 
cross(const V3& a, const V3& b) {
    return v3(a.y*b.z - a.z*b.y,
              a.z*b.x - a.x*b.z,
              a.x*b.y - a.y*b.x);
}

inline float
scalar_triple(const V3& a, const V3& b, const V3& c) {
    float result = dot(cross(a, b), c);
    return result;
}

inline V3 
reflect(const V3& v, const V3& n) {
    return v - 2.0f*dot(v, n)*n;
}

inline float 
length_sq(const V3& a) {
    return dot(a, a);
}

inline float 
length(const V3& a) {
    return square_root(dot(a, a));
}

inline V3 
normalize(const V3& a) {
    float rcp_len = 1.0f / length(a);
    return a*rcp_len;
}

inline V3 
noz(const V3& a) {
    V3 result = {};
    float lsq = length_sq(a);
    if ((lsq > 0.0001f) && (lsq < INFINITY)) {
        result = a / square_root(lsq);
    }
    return result;
}

inline V3
absolute_value(const V3& a) {
    V3 result;
    result.x = absolute_value(a.x);
    result.y = absolute_value(a.y);
    result.z = absolute_value(a.z);
    return result;
}

inline V3
min(const V3& a, const V3& b) {
    V3 result;
    result.x = min(a.x, b.x);
    result.y = min(a.y, b.y);
    result.z = min(a.z, b.z);
    return result;
}

inline V3
max(const V3& a, const V3& b) {
    V3 result;
    result.x = max(a.x, b.x);
    result.y = max(a.y, b.y);
    result.z = max(a.z, b.z);
    return result;
}

inline float
min3(const V3& a) {
    float result = min(a.x, min(a.y, a.z));
    return result;
}

inline float
max3(const V3& a) {
    float result = max(a.x, max(a.y, a.z));
    return result;
}

//
// Note: V4
//

// NOTE: Operator overloads

// TEST: covered
inline V4 
operator - (const V4& a) {
    V4 result;
    result.x = -a.x;
    result.y = -a.y;
    result.z = -a.z;
    result.w = -a.w;
    return result;
}

#define IMPLEMENT_VECTOR_OPERATORS(op) \
    inline V4  \
    operator op (const V4& a, const V4& b) { \
        V4 result; \
        result.x = a.x op b.x; \
        result.y = a.y op b.y; \
        result.z = a.z op b.z; \
        result.w = a.w op b.w; \
        return result; \
    } \
    inline V4& \
    operator op##= (V4& a, const V4& b) { \
        a = a op b; \
        return a; \
    }

#define IMPLEMENT_SCALAR_OPERATORS(op) \
    inline V4  \
    operator op (const V4& a, float b) { \
        V4 result; \
        result.x = a.x op b; \
        result.y = a.y op b; \
        result.z = a.z op b; \
        result.w = a.w op b; \
        return result; \
    } \
    inline V4  \
    operator op (float a, const V4& b) { \
        V4 result; \
        result.x = a op b.x; \
        result.y = a op b.y; \
        result.z = a op b.z; \
        result.w = a op b.w; \
        return result; \
    } \
    inline V4& \
    operator op##= (V4& a, float b) { \
        a = a op b; \
        return a; \
    }

// IMPLEMENT_SCALAR_OPERATORS(+)
// IMPLEMENT_SCALAR_OPERATORS(-)
// TEST: covered
IMPLEMENT_VECTOR_OPERATORS(+)
IMPLEMENT_VECTOR_OPERATORS(-)
IMPLEMENT_SCALAR_OPERATORS(*)
IMPLEMENT_SCALAR_OPERATORS(/)
IMPLEMENT_VECTOR_OPERATORS(*)
IMPLEMENT_VECTOR_OPERATORS(/)

#undef IMPLEMENT_VECTOR_OPERATORS
#undef IMPLEMENT_SCALAR_OPERATORS


// NOTE: "Constructor" functions

// TEST: covered
inline V4 
v4(float x, float y, float z, float w) {
    V4 result;
    result.x = x;
    result.y = y;
    result.z = z;
    result.w = w;
    return result;
}

inline V4 
v4(float x) {
    V4 result;
    result.x = x;
    result.y = x;
    result.z = x;
    result.w = x;
    return result;
}

// TEST: covered
inline V4 
v4(const V3& xyz, float w) {
    V4 result;
    result.x = xyz.e[0];
    result.y = xyz.e[1];
    result.z = xyz.e[2];
    result.w = w;
    return result;
}

// TEST: covered
inline V4 
v4(float x, const V3& yzw) {
    V4 result;
    result.x = x;
    result.y = yzw.e[0];
    result.z = yzw.e[1];
    result.w = yzw.e[2];
    return result;
}

// TEST: covered
inline V4 
v4(const V2& xy, float z, float w) {
    V4 result;
    result.x = xy.e[0];
    result.y = xy.e[1];
    result.z = z;
    result.w = w;
    return result;
}

// TEST: covered
inline V4 
v4(float x, float y, const V2& zw) {
    V4 result;
    result.x = x;
    result.y = y;
    result.z = zw.e[0];
    result.w = zw.e[1];
    return result;
}

// TEST: covered
inline V4 
v4(const V2& xy, const V2& zw) {
    V4 result;
    result.x = xy.e[0];
    result.y = xy.e[1];
    result.z = zw.e[0];
    result.w = zw.e[1];
    return result;
}

// NOTE: Other functions

inline V4 
lerp(const V4& a, const V4& b, float t) {
    return a*(1.0f - t) + b*t;
}

inline float 
dot(const V4& a, const V4& b) {
    return a.x*b.x + a.y*b.y + a.z*b.z + a.w*b.w;
}

// NOTE: 4D reflect... I believe the math is sound, but I surely can't conceive of what this would be like.
inline V4 
reflect(const V4& v, const V4& n) {
    return v - 2.0f*dot(v, n)*n;
}

inline float 
length_sq(const V4& a) {
    return dot(a, a);
}

inline float 
length(const V4& a) {
    return square_root(dot(a, a));
}

inline V4 
normalize(const V4& a) {
    float rcp_len = 1.0f / length(a);
    return a*rcp_len;
}

inline V4
noz(const V4& a) {
    V4 result = {};
    float lsq = length_sq(a);
    if ((lsq > 0.0001f) && (lsq < FLT_MAX)) {
        result = a / square_root(lsq);
    }
    return result;
}

inline V4
absolute_value(const V4& a) {
    V4 result;
    result.x = absolute_value(a.x);
    result.y = absolute_value(a.y);
    result.z = absolute_value(a.z);
    result.w = absolute_value(a.w);
    return result;
}

//
// NOTE: M4x4 (TODO: Split up into separate file?)
//

inline M4x4
m4x4(float a, float b, float c, float d, 
     float e, float f, float g, float h,
     float i, float j, float k, float l,
     float m, float n, float o, float p)
 {
    M4x4 result = {
        {
            { a, b, c, d, }, 
            { e, f, g, h, },
            { i, j, k, l, },
            { m, n, o, p, },
        },
    };
    return result;
}

inline M4x4
m4x4_identity() {
    M4x4 result = {
        {
            { 1, 0, 0, 0, }, 
            { 0, 1, 0, 0, },
            { 0, 0, 1, 0, },
            { 0, 0, 0, 1, },
        },
    };
    return result;
}

inline M4x4
m4x4_columns(const V3& x, const V3& y, const V3& z) {
    M4x4 result = {
        {
            { x.x, y.x, z.x, 0 },
            { x.y, y.y, z.y, 0 },
            { x.z, y.z, z.z, 0 },
            {   0,   0,   0, 1 },
        },
    };
    return result;
}

inline M4x4
m4x4_rows(const V3& x, const V3& y, const V3& z) {
    M4x4 result = {
        {
            { x.x, x.y, x.z, 0 },
            { y.x, y.y, y.z, 0 },
            { z.x, z.y, z.z, 0 },
            {   0,   0,   0, 1 },
        },
    };
    return result;
}

inline M4x4
m4x4_rotate_x_axis(float c, float s) {
    M4x4 result = {
        {
            { 1, 0, 0, 0, },
            { 0, c,-s, 0, },
            { 0, s, c, 0, },
            { 0, 0, 0, 1, },
        },
    };
    return result;
}

inline M4x4
m4x4_rotate_x_axis(float angle) {
    M4x4 result = m4x4_rotate_x_axis(cosf(angle), sinf(angle));
    return result;
}

inline M4x4
m4x4_rotate_y_axis(float c, float s) {
    M4x4 result = {
        {
            { c, 0, s, 0, },
            { 0, 1, 0, 0, },
            {-s, 0, c, 0, },
            { 0, 0, 0, 1, },
        },
    };
    return result;
}

inline M4x4
m4x4_rotate_y_axis(float angle) {
    M4x4 result = m4x4_rotate_y_axis(cosf(angle), sinf(angle));
    return result;
}

inline M4x4
m4x4_rotate_z_axis(float c, float s) {
    M4x4 result = {
        {
            { c,-s, 0, 0, },
            { s, c, 0, 0, },
            { 0, 0, 1, 0, },
            { 0, 0, 0, 1, },
        },
    };
    return result;
}

inline M4x4
m4x4_rotate_z_axis(float angle) {
    M4x4 result = m4x4_rotate_z_axis(cosf(angle), sinf(angle));
    return result;
}

inline M4x4
m4x4_scale(V3 scale) {
    float x = scale.x;
    float y = scale.y;
    float z = scale.z;

    M4x4 result = {
        {
            { x, 0, 0, 0, },
            { 0, y, 0, 0, },
            { 0, 0, z, 0, },
            { 0, 0, 0, 1, },
        },
    };

    return result;
}

inline M4x4
m4x4_translate(V3 translation) {
    float x = translation.x;
    float y = translation.y;
    float z = translation.z;

    M4x4 result = {
        {
            { 1, 0, 0, x, },
            { 0, 1, 0, y, },
            { 0, 0, 1, z, },
            { 0, 0, 0, 1, },
        },
    };

    return result;
}

inline M4x4
translate(M4x4 m, V3 translation) {
    M4x4 result = m;

    result.e[3][0] += translation.x;
    result.e[3][1] += translation.y;
    result.e[3][2] += translation.z;

    return result;
}

inline M4x4
operator*(const M4x4& a, const M4x4& b) {
    M4x4 result = {};
    // TODO: Flatten this loop
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            for (int i = 0; i < 4; ++i) {
                result.e[r][c] += a.e[r][i]*b.e[i][c];
            }
        }
    }
    return result;
}

inline bool
are_equal(const M4x4& a, const M4x4& b, float slop = 0.0f) {
    bool result = true;
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            if (absolute_value(a.e[r][c] - b.e[r][c]) > slop) {
                result = false;
                break;
            }
        }
    }
    return result;
}

inline V4
transform(const M4x4& a, const V4& p) {
    V4 result;
    result.x = p.x*a.e[0][0] + p.y*a.e[0][1] + p.z*a.e[0][2] + p.w*a.e[0][3];
    result.y = p.x*a.e[1][0] + p.y*a.e[1][1] + p.z*a.e[1][2] + p.w*a.e[1][3];
    result.z = p.x*a.e[2][0] + p.y*a.e[2][1] + p.z*a.e[2][2] + p.w*a.e[2][3];
    result.w = p.x*a.e[3][0] + p.y*a.e[3][1] + p.z*a.e[3][2] + p.w*a.e[3][3];
    return result;
}

inline V3
transform(const M4x4& a, const V3& p, float pw = 1.0f) {
    V3 result;
    result.x = p.x*a.e[0][0] + p.y*a.e[0][1] + p.z*a.e[0][2] + pw*a.e[0][3];
    result.y = p.x*a.e[1][0] + p.y*a.e[1][1] + p.z*a.e[1][2] + pw*a.e[1][3];
    result.z = p.x*a.e[2][0] + p.y*a.e[2][1] + p.z*a.e[2][2] + pw*a.e[2][3];
    return result;
}

inline V3
transform_normal(const M4x4& a, const V3& n) {
    V3 result;
    result.x = n.x*a.e[0][0] + n.y*a.e[0][1] + n.z*a.e[2][0];
    result.y = n.x*a.e[0][1] + n.y*a.e[1][1] + n.z*a.e[2][1];
    result.z = n.x*a.e[0][2] + n.y*a.e[1][2] + n.z*a.e[2][2];
    return result;
}

inline V3
operator*(const M4x4& a, V3 p) {
    V3 result = transform(a, p, 1.0f);
    return result;
}

inline V4
operator*(const M4x4& a, V4 p) {
    V4 result = transform(a, p);
    return result;
}

inline V3
translation(const M4x4& a) {
    V3 result = {
        a.e[0][3],
        a.e[1][3],
        a.e[2][3],
    };
    return result;
}

inline M4x4
transposed(const M4x4& a) {
    M4x4 result;
    for (int j = 0; j < 4; ++j)
    for (int i = 0; i < 4; ++i) {
        result.e[j][i] = a.e[i][j];
    }
    return result;
}

inline M4x4
transpose(M4x4* a) {
    for (int j = 0; j < 4; ++j)
    for (int i = 0; i < 4; ++i) {
        a->e[j][i] = a->e[i][j];
    }
}

//
// NOTE: M4x4Inv
//

inline M4x4Inv
operator * (M4x4Inv a, M4x4Inv b) {
    M4x4Inv result;
    result.forward = a.forward*b.forward;
    result.inverse = b.inverse*a.inverse;
    return result;
}

inline void
operator *= (M4x4Inv& a, M4x4Inv b) {
    a.forward = a.forward*b.forward;
    a.inverse = b.inverse*a.inverse;
}

inline M4x4Inv
transform_identity() {
    M4x4Inv result;
    result.forward = m4x4_identity();
    result.inverse = m4x4_identity();
    return result;
}

inline M4x4Inv
transform_translate(V3 translation) {
    M4x4Inv result;
    result.forward = m4x4_translate( translation);
    result.inverse = m4x4_translate(-translation);
    return result;
};

inline M4x4Inv
transform_scale(V3 scale) {
    M4x4Inv result;
    result.forward = m4x4_scale(scale);
    result.inverse = m4x4_scale(1.0f / scale);
    return result;
}

inline M4x4Inv
transform_rotate_x_axis(float rotation) {
    M4x4Inv result;
    result.forward = m4x4_rotate_x_axis( rotation);
    result.inverse = m4x4_rotate_x_axis(-rotation);
    return result;
}

inline M4x4Inv
transform_rotate_y_axis(float rotation) {
    M4x4Inv result;
    result.forward = m4x4_rotate_y_axis( rotation);
    result.inverse = m4x4_rotate_y_axis(-rotation);
    return result;
}

inline M4x4Inv
transform_rotate_z_axis(float rotation) {
    M4x4Inv result;
    result.forward = m4x4_rotate_z_axis( rotation);
    result.inverse = m4x4_rotate_z_axis(-rotation);
    return result;
}

//
// NOTE: AABB
//

inline AABB
aabb_center_radius(V3 p, V3 r) {
    AABB result;
    result.min = p - r;
    result.max = p + r;
    return result;
}

inline AABB
union_of(const AABB& a, const AABB& b) {
    AABB result;
    result.min = min(a.min, b.min);
    result.max = max(a.max, b.max);
    return result;
}

inline AABB
grow_to_contain(const AABB& a, V3 p) {
    AABB result;
    result.min = min(a.min, p);
    result.max = max(a.max, p);
    return result;
}

inline AABB
inverted_infinity_aabb() {
    AABB result;
    result.min = v3( FLT_MAX);
    result.max = v3(-FLT_MAX);
    return result;
}

inline V3
get_dim(const AABB& a) {
    V3 result = (a.max - a.min);
    return result;
}

inline unsigned int
get_largest_axis(const AABB& a) {
    V3 dim = get_dim(a);

    unsigned int largest_axis = 0;
    float max_axis = dim.x;
    if (max_axis < dim.y) {
        max_axis = dim.y;
        largest_axis = 1;
    }
    if (max_axis < dim.z) {
        max_axis = dim.z;
        largest_axis = 2;
    }

    return largest_axis;
}

inline float
get_surface_area(const AABB& a) {
    V3 dim = get_dim(a);
    float result = 2.0f*(dim.x*dim.y +
                         dim.x*dim.z +
                         dim.y*dim.z);
    return result;
}

} // namespace math
