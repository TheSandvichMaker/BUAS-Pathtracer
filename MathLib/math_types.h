#pragma once

namespace math {

union V2 {
    struct {
        float x, y;
    };
    float e[2];
};

union V3 {
    struct {
        float x, y, z;
    };
    float e[3];
};

union V4 {
    struct {
        float x, y, z, w;
    };
    struct {
        V3 xyz;
        float ignored0;
    };
    struct {
        V2 xy;
        V2 zw;
    };
    struct {
        float ignored1;
        V3 yzw;
    };
    struct {
        float ignored2;
        V2 yz;
        float ignored3;
    };
    float e[4];
};

struct M4x4 {
    float e[4][4];
};

struct M4x4Inv {
    M4x4 forward;
    M4x4 inverse;
};

struct AABB {
    V3 min;
    V3 max;
};

};
