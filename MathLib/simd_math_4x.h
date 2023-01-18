#pragma once

#include "math_types.h" // V3

#include <xmmintrin.h>
#include <stddef.h> // size_t

namespace math {

//
// NOTE: f32_4x
//

struct f32_4x {
    __m128 v;

    static f32_4x
    from(float scalar) {
        f32_4x result;
        result.v = _mm_set1_ps(scalar);
        return result;
    }

    static f32_4x
    from(float a, float b, float c, float d) {
        f32_4x result;
        result.v = _mm_setr_ps(a, b, c, d);
        return result;
    }

    inline
    operator __m128() {
        return v;
    }

    inline float&
    operator [] (size_t i) {
        float& result = ((float*)&v)[i];
        return result;
    }

    inline float
    operator [] (size_t i) const {
        float result = ((float*)&v)[i];
        return result;
    }
};

// NOTE: Operator Overloads

inline f32_4x
operator - (f32_4x a) {
    f32_4x result;
    result.v = _mm_sub_ps(_mm_set1_ps(0.0f), a);
    return result;
}

inline f32_4x
operator + (f32_4x a, f32_4x b) {
    f32_4x result;
    result.v = _mm_add_ps(a, b);
    return result;
}

inline f32_4x
operator - (f32_4x a, f32_4x b) {
    f32_4x result;
    result.v = _mm_sub_ps(a, b);
    return result;
}

inline f32_4x
operator * (f32_4x a, f32_4x b) {
    f32_4x result;
    result.v = _mm_mul_ps(a, b);
    return result;
}

inline f32_4x
operator / (f32_4x a, f32_4x b) {
    f32_4x result;
    result.v = _mm_div_ps(a, b);
    return result;
}

inline f32_4x
operator < (f32_4x a, f32_4x b) {
    f32_4x result;
    result.v = _mm_cmplt_ps(a, b);
    return result;
}

inline f32_4x
operator > (f32_4x a, f32_4x b) {
    f32_4x result;
    result.v = _mm_cmpgt_ps(a, b);
    return result;
}

inline f32_4x
operator <= (f32_4x a, f32_4x b) {
    f32_4x result;
    result.v = _mm_cmple_ps(a, b);
    return result;
}

inline f32_4x
operator >= (f32_4x a, f32_4x b) {
    f32_4x result;
    result.v = _mm_cmpge_ps(a, b);
    return result;
}

inline f32_4x
operator & (f32_4x a, f32_4x b) {
    f32_4x result;
    result.v = _mm_and_ps(a, b);
    return result;
}

inline f32_4x
operator | (f32_4x a, f32_4x b) {
    f32_4x result;
    result.v = _mm_or_ps(a, b);
    return result;
}

// NOTE: Functions

inline bool
mask_is_zero(f32_4x a) {
    bool result = (_mm_movemask_ps(a) == 0);
    return result;
}

inline bool
extract_mask(f32_4x a, size_t i) {
    bool result = (bool&)a[i];
    return result;
}

inline void
conditional_assign(f32_4x* dest, f32_4x mask, f32_4x source) {
    dest->v = _mm_or_ps(_mm_andnot_ps(mask.v, dest->v), _mm_and_ps(mask.v, source.v));
}

inline f32_4x
select(f32_4x prev, f32_4x mask, f32_4x next) {
    f32_4x result;
    result.v = _mm_or_ps(_mm_andnot_ps(mask.v, prev.v), _mm_and_ps(mask.v, next.v));
    return result;
}

//
// NOTE: V3_4x
//

struct V3_4x {
    f32_4x x;
    f32_4x y;
    f32_4x z;

    static V3_4x
    from(V3 a) {
        V3_4x result;
        result.x = f32_4x::from(a.x);
        result.y = f32_4x::from(a.y);
        result.z = f32_4x::from(a.z);
        return result;
    }

    static V3_4x
    from(float* x, float* y, float* z) {
        V3_4x result;
        result.x = f32_4x::from(x[0], x[1], x[2], x[3]);
        result.y = f32_4x::from(y[0], y[1], y[2], y[3]);
        result.z = f32_4x::from(z[0], z[1], z[2], z[3]);
        return result;
    }

    static V3_4x
    from (__m128 x, __m128 y, __m128 z) {
        V3_4x result;
        result.x.v = x;
        result.y.v = y;
        result.z.v = z;
        return result;
    }
};

// NOTE: Operator Overloads

inline V3_4x
operator + (V3_4x a, V3_4x b) {
    V3_4x result;
    result.x = a.x + b.x;
    result.y = a.y + b.y;
    result.z = a.z + b.z;
    return result;
}

inline V3_4x
operator - (V3_4x a, V3_4x b) {
    V3_4x result;
    result.x = a.x - b.x;
    result.y = a.y - b.y;
    result.z = a.z - b.z;
    return result;
}

inline V3_4x
operator * (V3_4x a, V3_4x b) {
    V3_4x result;
    result.x = a.x*b.x;
    result.y = a.y*b.y;
    result.z = a.z*b.z;
    return result;
}

inline V3_4x
operator / (V3_4x a, V3_4x b) {
    V3_4x result;
    result.x = a.x / b.x;
    result.y = a.y / b.y;
    result.z = a.z / b.z;
    return result;
}

inline f32_4x
dot(V3_4x a, V3_4x b) {
    f32_4x result = a.x*b.x + a.y*b.y + a.z*b.z;
    return result;
}

inline V3_4x
cross(V3_4x a, V3_4x b) {
    V3_4x result;
    result.x = a.y*b.z - a.z*b.y;
    result.y = a.z*b.x - a.x*b.z;
    result.z = a.x*b.y - a.y*b.x;
    return result;
}

inline V3
extract(V3_4x a, size_t i) {
    V3 result;
    result.x = a.x[i];
    result.y = a.y[i];
    result.z = a.z[i];
    return result;
}

inline void
set(V3_4x* a, size_t i, V3 v) {
    a->x[i] = v.x;
    a->y[i] = v.y;
    a->z[i] = v.z;
}

//
// NOTE: V4_4x
//

struct V4_4x {
    f32_4x x;
    f32_4x y;
    f32_4x z;
    f32_4x w;

    static V4_4x
    from(f32_4x a) {
        V4_4x result;
        result.x = a;
        result.y = a;
        result.z = a;
        result.w = a;
        return result;
    }

    static V4_4x
    from(V4 a) {
        V4_4x result;
        result.x = f32_4x::from(a.x);
        result.y = f32_4x::from(a.y);
        result.z = f32_4x::from(a.z);
        result.w = f32_4x::from(a.w);
        return result;
    }

    static V4_4x
    transposed(V4 a) {
        V4_4x result;
        result.x = f32_4x::from(a.x, a.y, a.z, a.w);
        result.y = f32_4x::from(a.x, a.y, a.z, a.w);
        result.z = f32_4x::from(a.x, a.y, a.z, a.w);
        result.w = f32_4x::from(a.x, a.y, a.z, a.w);
        return result;
    }

    static V4_4x
    from(float* x, float* y, float* z, float* w) {
        V4_4x result;
        result.x = f32_4x::from(x[0], x[1], x[2], x[3]);
        result.y = f32_4x::from(y[0], y[1], y[2], y[3]);
        result.z = f32_4x::from(z[0], z[1], z[2], z[3]);
        result.w = f32_4x::from(w[0], w[1], w[2], w[3]);
        return result;
    }

    static V4_4x
    from (__m128 x, __m128 y, __m128 z, __m128 w) {
        V4_4x result;
        result.x.v = x;
        result.y.v = y;
        result.z.v = z;
        result.w.v = w;
        return result;
    }
};

// NOTE: Operator Overloads

inline V4_4x
operator + (V4_4x a, V4_4x b) {
    V4_4x result;
    result.x = a.x + b.x;
    result.y = a.y + b.y;
    result.z = a.z + b.z;
    return result;
}

inline V4_4x
operator - (V4_4x a, V4_4x b) {
    V4_4x result;
    result.x = a.x - b.x;
    result.y = a.y - b.y;
    result.z = a.z - b.z;
    result.w = a.w - b.w;
    return result;
}

inline V4_4x
operator * (V4_4x a, V4_4x b) {
    V4_4x result;
    result.x = a.x*b.x;
    result.y = a.y*b.y;
    result.z = a.z*b.z;
    result.w = a.w*b.w;
    return result;
}

inline V4_4x
operator * (f32_4x a, V4_4x b) {
    V4_4x result;
    result.x = a*b.x;
    result.y = a*b.y;
    result.z = a*b.z;
    result.w = a*b.w;
    return result;
}

inline V4_4x
operator * (V4_4x a, f32_4x b) {
    V4_4x result;
    result.x = a.x*b;
    result.y = a.y*b;
    result.z = a.z*b;
    result.w = a.w*b;
    return result;
}

inline V4_4x
operator / (V4_4x a, V4_4x b) {
    V4_4x result;
    result.x = a.x / b.x;
    result.y = a.y / b.y;
    result.z = a.z / b.z;
    result.w = a.w / b.w;
    return result;
}

inline f32_4x
dot(V4_4x a, V4_4x b) {
    f32_4x result = a.x*b.x + a.y*b.y + a.z*b.z + a.w*b.w;
    return result;
}

inline V4
extract(V4_4x a, size_t i) {
    V4 result;
    result.x = a.x[i];
    result.y = a.y[i];
    result.z = a.z[i];
    result.w = a.w[i];
    return result;
}

inline void
set(V4_4x* a, size_t i, V4 v) {
    a->x[i] = v.x;
    a->y[i] = v.y;
    a->z[i] = v.z;
    a->w[i] = v.w;
}

};
