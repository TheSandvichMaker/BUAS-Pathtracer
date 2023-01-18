#pragma once

#ifdef _DEBUG
#define RAY_DEBUG 1
#else
#define RAY_DEBUG 0
#endif

#ifndef RAY_USE_ABC
#define RAY_USE_ABC RAY_DEBUG
#endif

typedef int8_t  b8;
typedef int16_t  b16;
typedef int32_t  b32;

typedef float    f32;
typedef double   f64;

typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;

typedef int8_t   s8;
typedef int16_t  s16;
typedef int32_t  s32;
typedef int64_t  s64;

typedef uintptr_t usize;
typedef intptr_t  ssize;

#define force_inline __forceinline

const f32 EPSILON = 0.001f;

#undef NDEBUG
#include <assert.h>

#ifdef _DEBUG
#define assert_slow(x) assert(x)
#else
#define assert_slow(x)
#endif

#define INVALID_CODE_PATH assert(!"Reached an invalid code path!");
#define INVALID_DEFAULT_CASE default: { assert(!"Reached an invalid default case!"); } break;

#define ArrayCount(array) (sizeof(array) / sizeof((array)[0]))
#define ArrayEnd(array) (array) + ArrayCount(array)

#define Paste__(a, b) a##b
#define Paste_(a, b) Paste__(a, b)
#define Paste(a, b) Paste_(a, b)
#define Stringify__(x) #x
#define Stringify_(x) Stringify__(x)
#define Stringify(x) Stringify_(x)
#define Expand_(x) x
#define Expand(x) Expand(x)

#define BitIsSet(mask, bit) ((mask) & ((u64)1 << bit))
#define SetBit(mask, bit)   ((mask) |= ((u64)1 << bit))
#define UnsetBit(mask, bit) ((mask) &= ~((u64)1 << bit))

#define AlignPow2(value, align) (((value) + ((align) - 1)) & ~((align) - 1))
#define Align4(value) ((value + 3) & ~3)
#define Align8(value) ((value + 7) & ~7)
#define Align16(value) ((value + 15) & ~15)

#define Kilobytes(num) ((num)*(u64)1024)
#define Megabytes(num) (Kilobytes(num)*(u64)1024)
#define Gigabytes(num) (Megabytes(num)*(u64)1024)
#define Terabytes(num) (Gigabytes(num)*(u64)1024)

#define Milliseconds(seconds) ((seconds) / 1000.0f)

#define Abs(val) ((val) < 0 ? -(val) : (val))
#define Min(a, b) ((a) < (b) ? (a) : (b))
#define Max(a, b) ((a) > (b) ? (a) : (b))
#define Clamp(x, lo, hi) (Max((lo), Min((hi), (x))))

#define Swap(a, b) { auto swap_temp_ = a; a = b; b = swap_temp_; }

#define zero_struct(x) memset(x, 0, sizeof(*x))
#define zero_array(count, data) memset(data, 0, sizeof(*data)*count)
#define zero_size(size, data) memset(data, 0, size)
#define copy_struct(a, b) memcpy(b, a, sizeof(*(a)))
#define copy_array(count, a, b) memcpy(b, a, (count)*sizeof(*(a)))
#define copy_size(size, a, b) memcpy(b, a, size)

#define ForStructAsArrayOf(type, it, strct) for (type* it = (type*)(strct); it < (type*)((char*)(strct) + sizeof(*(strct))); ++(it))

template <typename T> b32
structs_are_equal(T* src, T* dst) {
    b32 result = (0 == memcmp(src, dst, sizeof(T)));
    return result;
}

#define Defer(f) DeferHelper Paste(defer_, __LINE__)([&]() { f; })
struct DeferHelper {
    std::function<void()> f;

    DeferHelper(std::function<void()> f_init) {
        f = f_init;
    }

    ~DeferHelper() {
        f();
    }
};

template <typename Tag, typename IDType>
struct TypesafeID {
    IDType id;

    static inline TypesafeID
    from(IDType untyped_id) {
        TypesafeID result;
        result.id = untyped_id;
        return result;
    }

    operator u32() { return id; }
    bool operator == (TypesafeID rhs) { return id == rhs.id; }
    bool operator != (TypesafeID rhs) { return id != rhs.id; }
};

typedef TypesafeID<struct MaterialTag, u32> MaterialID;
typedef TypesafeID<struct PrimitiveTag, u32> PrimitiveID;

force_inline f32
extract(__m128 v, usize i) {
    return ((f32*)&v)[i];
}

force_inline u32
extract(__m128i v, usize i) {
    return ((u32*)&v)[i];
}

inline f32
luma(V3 l) {
    // NOTE: Rec. 601 coefficients
    return 0.299f*l.x + 0.587f*l.y + 0.114f*l.z;
}

#include "platform.h"
#include "memory_arena.h"
#include "stretchy_buffer.h"
