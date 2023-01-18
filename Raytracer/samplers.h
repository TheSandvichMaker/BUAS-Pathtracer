#pragma once

force_inline u32
wang_hash(u32 key) {
    key += ~(key << 15);
    key ^=  (key >> 10);
    key +=  (key << 3);
    key ^=  (key >> 6);
    key += ~(key << 11);
    key ^=  (key >> 16);
    return key;
}

static u32
hash_coordinate(u32 x, u32 y, u32 z) {
    u32 result = ((x*73856093)^(y*83492791)^(z*871603259));
    return result;
}

static u32
hash_coordinate(u32 x, u32 y) {
    // SOURCE: https://www.shadertoy.com/view/4tXyWN
    u32 qx = 1103515245U*((x >> 1)^y);
    u32 qy = 1103515245U*((y >> 1)^x);
    u32 result = 1103515245U*(qx^(qy >> 3));
    return result;
}

struct RandomSeries {
    union {
        __m128i state;
        u32 e[4];
    };
};

static __m128i
next_set(RandomSeries* series) {
    // NOTE: https://en.wikipedia.org/wiki/Xorshift
    __m128i result = series->state;
    result = _mm_xor_si128(result, _mm_slli_epi32(result, 13));
    result = _mm_xor_si128(result, _mm_srli_epi32(result, 17));
    result = _mm_xor_si128(result, _mm_slli_epi32(result, 5));
    series->state = result;
    return result;
}

static u32
next_scalar(RandomSeries* series) {
    u32 result = series->e[0];
    result ^= result << 13;
    result ^= result >> 17;
    result ^= result << 5;
    series->e[0] = result;
    return result;
}

static u32
random_range(RandomSeries* series, u32 min, u32 max) {
    u32 result = min;
    if (max > min) {
        u32 x = next_scalar(series);
        u32 range = max - min;
        result = min + (x % range);
    }
    return result;
}

force_inline __m128
random_unilaterals_internal(RandomSeries* series) {
    // NOTE: Stolen from rnd.h, courtesy of Jonatan Hedborg
    //       https://github.com/mattiasgustavsson/libs/blob/main/rnd.h
    __m128i exponent = _mm_set1_epi32(127);
    __m128i mantissa = _mm_srli_epi32(next_set(series), 9);
    __m128i bits     = _mm_or_si128(_mm_slli_epi32(exponent, 23), mantissa);
    __m128  result   = _mm_sub_ps(_mm_castsi128_ps(bits), _mm_set1_ps(1.0f));
    return result;
}

force_inline V4
random_unilaterals(RandomSeries* series) {
    __m128 result = random_unilaterals_internal(series);
    return v4(extract(result, 0), extract(result, 1), extract(result, 2), extract(result, 3));
}

force_inline V4
random_bilaterals(RandomSeries* series) {
    __m128 result = random_unilaterals_internal(series);
    result = _mm_sub_ps(_mm_mul_ps(result, _mm_set1_ps(2.0f)), _mm_set1_ps(1.0f));
    return v4(extract(result, 0), extract(result, 1), extract(result, 2), extract(result, 3));
}

inline RandomSeries 
random_seed(u32 seed) {
    RandomSeries result;
    if (seed == 0) {
        seed = 0xFFFFFFFF;
    }
    result.state = _mm_set1_epi32(wang_hash(seed));
    // NOTE: Seed the state properly
    __m128i a = next_set(&result);
    __m128i b = next_set(&result);
    __m128i c = next_set(&result);
                next_set(&result);
    result.e[0] = wang_hash(extract(a, 0));
    result.e[1] = wang_hash(extract(b, 1));
    result.e[2] = wang_hash(extract(c, 2));
    return result;
}

enum SamplingStrategy {
    SamplingStrategy_Uniform,
    SamplingStrategy_OptimizedBlueNoise,
    SamplingStrategy_Stratified,
    SamplingStrategy_COUNT,

    SamplingStrategy_Default, // NOTE: Uses the scene strategy
};

inline const char*
to_string(SamplingStrategy x) {
    switch (x) {
        case SamplingStrategy_Uniform: return "Uniform";
        case SamplingStrategy_OptimizedBlueNoise: return "Optimized Blue Noise";
        case SamplingStrategy_Stratified: return "Stratified";
    }
    return "Unknown Sampling Strategy";
}

enum SampleDimension {
    Sample_DirectLighting,
    Sample_IndirectLighting,
    Sample_LightSelection,
    Sample_Reflectance,
    Sample_DOF,
    Sample_AA,
    Sample_Roulette,
    Sample_COUNT,
};

struct Sampler {
    RandomSeries* entropy;
    SamplingStrategy strategy;
    u32 sample_index;
    u32 x, y;
};

//

V2  get_next_sample_2d(Sampler* sampler, SampleDimension dimension, usize bounce_index);
f32 get_next_sample_1d(Sampler* sampler, SampleDimension dimension, usize bounce_index);
