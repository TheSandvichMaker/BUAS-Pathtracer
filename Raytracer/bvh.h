#pragma once

struct Arena;
struct Triangle;
struct Triangle_4x;

enum BVHConstructionMethod {
    BVH_MidpointSplit,
    BVH_SAHBinned,
    BVH_SAHFull,
};

inline const char*
to_string(BVHConstructionMethod method) {
    switch (method) {
        case BVH_MidpointSplit: return "Midpoint Split";
        case BVH_SAHBinned    : return "Surface Area Heuristic - Binned";
        case BVH_SAHFull      : return "Surfcae Area Heuristic - Full";
    }
    return "Uknown BVH Construction Method";
}

const u32 MIN_PRIMITIVES_PER_BVH_LEAF = 4;

struct BVHSortEntry {
    u32 index;
    V3 bv_p;
    V3 bv_r;
};

struct BVHNode {
    V3 bv_p;
    V3 bv_r;
    u32 left_first;
    u16 count;
    u16 split_axis;
};

struct BVH {
    u32 node_count;
    u32 index_count;

    u32* indices;
    BVHNode* nodes;
};

enum BVHStorageType {
    BVHStorage_Scalar,
    BVHStorage_4x,
};

struct MeshBVH : BVH {
    BVHStorageType storage;
    union {
        Triangle* triangles;
        Triangle_4x* triangles_4x;
    };
};

//

BVH*     create_bvh         (u32 input_count, BVHSortEntry* input_data, Arena* arena, BVHConstructionMethod method);
MeshBVH* create_bvh_for_mesh(u32 triangle_count, Triangle* triangles, Arena* arena, Arena* temp_arena, BVHConstructionMethod method, BVHStorageType storage);
