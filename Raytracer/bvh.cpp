#include "precomp.h"
#include "common.h"
#include "primitives.h"
#include "bvh.h"

static void
compute_bounding_volume(u32 entry_count, BVHSortEntry* entries, AABB* out_bv, AABB* out_cr) {
    AABB bv = inverted_infinity_aabb();
    AABB cr = inverted_infinity_aabb();
    for (usize entry_index = 0; entry_index < entry_count; ++entry_index) {
        BVHSortEntry* entry = &entries[entry_index];
        bv = union_of(bv, aabb_center_radius(entry->bv_p, entry->bv_r));
        cr = grow_to_contain(cr, entry->bv_p);
    }
    *out_bv = bv;
    *out_cr = cr;
}

struct PartitionResult {
    u32 split_axis;
    u32 split_index;
    AABB left_bv;
    AABB right_bv;
};

static PartitionResult
partition_objects(u32 entry_count, BVHSortEntry* entries, f32 split_p, u32 split_axis) {
    ssize i = -1;
    ssize j = (ssize)entry_count;
    for (;;) {
        do {
            ++i;
            // assert_slow(i < entry_count);
        } while (entries[i].bv_p.e[split_axis] < split_p);

        do {
            --j;
            // assert_slow(j >= 0);
        } while (entries[j].bv_p.e[split_axis] > split_p);

        if (i >= j) {
            break;
        }
        Swap(entries[i], entries[j]);
    }

    PartitionResult result;
    result.split_axis = split_axis;
    result.split_index = (u32)i;
    return result;
}

static PartitionResult
partition_objects_midpoint_split(BVHSortEntry* sort_data, AABB bv, u32 entry_count, u32 entry_offset) {
    u32 split_axis = get_largest_axis(bv);
    f32 pivot = (0.5f*(bv.min + bv.max)).e[split_axis];

    BVHSortEntry* entries = sort_data + entry_offset;
    PartitionResult result = partition_objects(entry_count, entries, pivot, split_axis);
    return result;
}

static f32
evaluate_sah(u32 entry_count, BVHSortEntry* entries, f32 split_p, u32 split_axis) {
    // TODO: Use the sweep method the binned technique uses, this would require this routine
    //       to be passed a temporary memory arena.
    u32 l_count = 0;
    V3 l_min = v3( FLT_MAX,  FLT_MAX,  FLT_MAX);
    V3 l_max = v3(-FLT_MAX, -FLT_MAX, -FLT_MAX);

    u32 r_count = 0;
    V3 r_min = v3( FLT_MAX,  FLT_MAX,  FLT_MAX);
    V3 r_max = v3(-FLT_MAX, -FLT_MAX, -FLT_MAX);

    for (usize entry_index = 0; entry_index < entry_count; ++entry_index) {
        BVHSortEntry* entry = &entries[entry_index];
        if (entry->bv_p.e[split_axis] <= split_p) {
            ++l_count;
            l_min = min(l_min, entry->bv_p - entry->bv_r);
            l_max = max(l_max, entry->bv_p + entry->bv_r);
        } else {
            ++r_count;
            r_min = min(r_min, entry->bv_p - entry->bv_r);
            r_max = max(r_max, entry->bv_p + entry->bv_r);
        }
    }

    V3 l_dim = (l_max - l_min);
    V3 r_dim = (r_max - r_min);

    f32 l_surface_area = 2.0f*(l_dim.x*l_dim.y + l_dim.x*l_dim.z + l_dim.y*l_dim.z);
    f32 r_surface_area = 2.0f*(r_dim.x*r_dim.y + r_dim.x*r_dim.z + r_dim.y*r_dim.z);

    f32 l_sah = l_surface_area*(f32)l_count;
    f32 r_sah = r_surface_area*(f32)r_count;

    f32 result = l_sah + r_sah;

    return result;
}

static PartitionResult
partition_objects_sah(BVHSortEntry* sort_data, AABB bv, AABB cr, u32 entry_count, u32 entry_offset) {
    f32 parent_sah = (f32)entry_count*get_surface_area(bv);

    f32 best_sah = parent_sah;
    f32 best_split_p = 0.0f;
    u32 best_split_axis = 0;

    u32 split_axis = get_largest_axis(cr);

    BVHSortEntry* entries = sort_data + entry_offset;
    for (usize entry_index = 0; entry_index < entry_count; ++entry_index) {
        BVHSortEntry* entry = &entries[entry_index];
        f32 split_p = entry->bv_p.e[split_axis];
        f32 result_sah = evaluate_sah(entry_count, entries, split_p, split_axis);
        if (best_sah > result_sah) {
            best_sah = result_sah;
            best_split_p = split_p;
            best_split_axis = split_axis;
        }
    }

    PartitionResult result = {};

    if (best_sah < parent_sah) {
        result = partition_objects(entry_count, entries, best_split_p, best_split_axis);
    }

    return result;
}

f32 compute_sah(u32 entry_count, f32 surface_area) {
    f32 result = (f32)entry_count*surface_area;
    return result;
}

static PartitionResult
partition_objects_sah_binned(BVHSortEntry* sort_data, AABB bv, AABB cr, u32 entry_count, u32 entry_offset) {
    // REFERENCE: On fast Construction of SAH-based Bounding Volume Hierarchies, Wald, 2007
    f32 parent_sah = compute_sah(entry_count, get_surface_area(bv));
    f32 best_sah = parent_sah;

    f32 split_p = 0.0f;
    u32 split_axis = get_largest_axis(cr);

    const ssize bin_count = 16;

    struct Bin {
        u32 entry_count = 0;
        AABB bounds     = inverted_infinity_aabb();
    };
    
    Bin bins[bin_count] = {};

    f32 k0 = cr.min.e[split_axis];
    f32 k1 = ((f32)bin_count*(1.0f - EPSILON)) / (cr.max.e[split_axis] - cr.min.e[split_axis]);

    BVHSortEntry* entries = sort_data + entry_offset;
    for (usize entry_index = 0; entry_index < entry_count; ++entry_index) {
        BVHSortEntry* entry = &entries[entry_index];
        u32 bin_index = (u32)(k1*(entry->bv_p.e[split_axis] - k0));
        Bin* bin = &bins[bin_index];
        bin->entry_count += 1;
        bin->bounds.min = min(bin->bounds.min, entry->bv_p - entry->bv_r);
        bin->bounds.max = max(bin->bounds.max, entry->bv_p + entry->bv_r);
    }

    Bin empty_bin = {};

    Bin l_splits[bin_count] = {};
    Bin r_splits[bin_count] = {};

    for (ssize bin_index = 0; bin_index < (bin_count - 1); ++bin_index) {
        Bin* bin = &bins[bin_index];

        Bin* l_split = &l_splits[bin_index];
        Bin* prev_split = (bin_index > 0 ? &l_splits[bin_index - 1]
                                         : &empty_bin);

        l_split->entry_count = prev_split->entry_count + bin->entry_count;
        l_split->bounds = union_of(prev_split->bounds, bin->bounds);
    }

    for (ssize bin_index = bin_count - 1; bin_index >= 1; --bin_index) {
        Bin* bin = &bins[bin_index];

        Bin* r_split = &r_splits[bin_index];
        Bin* prev_split = (bin_index < bin_count - 1 ? &r_splits[bin_index + 1]
                                                     : &empty_bin);

        r_split->entry_count = prev_split->entry_count + bin->entry_count;
        r_split->bounds = union_of(prev_split->bounds, bin->bounds);

        Bin* l_split = &l_splits[bin_index];
        f32 l_sah = compute_sah(l_split->entry_count, get_surface_area(l_split->bounds));
        f32 r_sah = compute_sah(r_split->entry_count, get_surface_area(r_split->bounds));

        f32 sah = l_sah + r_sah;

        if ((sah > 0.0f) && (sah < best_sah)) {
            best_sah = sah;
            split_p = k0 + ((f32)bin_index / k1);
        }
    }

    PartitionResult result = {};
    if (best_sah < parent_sah) {
        result = partition_objects(entry_count, entries, split_p, split_axis);
    }

    return result;
}

struct BVHConstructionState {
    u32 node_count;
    BVHNode* nodes;
    BVHSortEntry* sort_data;
    StretchyBuffer<u32> leaf_primitive_count_buckets;
};

static void
construct_top_down_bvh_internal(BVHConstructionState* state,
                                BVHNode* parent,
                                u32 count, u32 first,
                                BVHConstructionMethod method)
{
    BVHSortEntry* sort_data = state->sort_data;

    AABB bv, cr;
    compute_bounding_volume(count, sort_data + first, &bv, &cr);

    parent->bv_p = 0.5f*(bv.min + bv.max);
    parent->bv_r = 0.5f*(bv.max - bv.min);

    b32 make_leaf = count <= MIN_PRIMITIVES_PER_BVH_LEAF;

    if (!make_leaf) {
        PartitionResult partition = {};
        switch (method) {
            case BVH_MidpointSplit: {
                partition = partition_objects_midpoint_split(sort_data, bv, count, first);
            } break;

            case BVH_SAHFull: {
                partition = partition_objects_sah(sort_data, bv, cr, count, first);
            } break;

            case BVH_SAHBinned: {
                partition = partition_objects_sah_binned(sort_data, bv, cr, count, first);
            } break;
        }

        if ((partition.split_index == 0) || (partition.split_index > (count - 1))) {
            make_leaf = true;
        } else {
			parent->split_axis = (u16)partition.split_axis;

			u32  left_node_index = state->node_count++;
			u32 right_node_index = state->node_count++;

			parent->left_first = left_node_index;

			BVHNode* left = &state->nodes[left_node_index];
            u32 left_split_count = partition.split_index;
            u32 left_split_index = first;
			construct_top_down_bvh_internal(state, left, left_split_count, left_split_index, method);

			BVHNode* right = &state->nodes[right_node_index];
            u32 right_split_count = count - partition.split_index;
            u32 right_split_index = first + partition.split_index;
			construct_top_down_bvh_internal(state, right, right_split_count, right_split_index, method);
        }
    }

    if (make_leaf) {
        parent->left_first = first;
        parent->count      = (u16)count;

        if (parent->count >= state->leaf_primitive_count_buckets.count) {
            u32 required_count = 1 + (parent->count - state->leaf_primitive_count_buckets.count);
            state->leaf_primitive_count_buckets.add_n(required_count);
        }

        state->leaf_primitive_count_buckets[parent->count] += 1;
    }
}

static void
construct_bvh_internal(u32                   input_data_count,
                       BVHSortEntry*         input_data,
                       Arena*                arena,
                       BVHConstructionMethod method,
                       u32*                  out_node_count,
                       BVHNode**             out_nodes)
{
    usize required_node_count = 2*(usize)input_data_count;

    u32 node_count = 0;
    BVHNode* nodes = push_array(arena, required_node_count, BVHNode, clear(64));

    BVHNode* root = &nodes[node_count++];
    node_count++; // Make sure node pairs end up on cache lines

    BVHConstructionState state = {};
    state.node_count = node_count;
    state.nodes      = nodes;
    state.sort_data  = input_data;

    TemporaryMemory leaf_primitive_temp = begin_temporary_memory(arena);
    state.leaf_primitive_count_buckets.arena = arena;

    construct_top_down_bvh_internal(&state, root, input_data_count, 0, method);

    fprintf(stderr, "Constructed BVH for %u primitives:\n", input_data_count);
    for (u32 i = 0; i < state.leaf_primitive_count_buckets.count; ++i) {
        u32 count = state.leaf_primitive_count_buckets[i];
        if (count > 0) {
            fprintf(stderr, "Nodes with %u primitives: %u\n", i, count);
        }
    }
    end_temporary_memory(leaf_primitive_temp);

    *out_node_count = state.node_count;
    *out_nodes      = state.nodes;
}

BVH*
create_bvh(u32 input_count, BVHSortEntry* input_data, Arena* arena, BVHConstructionMethod method) {
    BVH* result = push_struct(arena, BVH);
    construct_bvh_internal(input_count, input_data, arena, method, &result->node_count, &result->nodes);

    result->index_count = input_count;
    result->indices     = push_array(arena, result->index_count, u32, no_clear());
    for (usize i = 0; i < input_count; ++i) {
        result->indices[i] = input_data[i].index;
    }

    return result;
}

MeshBVH*
create_bvh_for_mesh(u32 triangle_count,
                    Triangle* triangles,
                    Arena* arena,
                    Arena* temp_arena,
                    BVHConstructionMethod method,
                    BVHStorageType storage)
{
    ScopedMemory scoped_memory(temp_arena);

    u32 input_count = triangle_count;
    BVHSortEntry* input_data = push_array(temp_arena, input_count, BVHSortEntry, no_clear());

    for (usize i = 0; i < input_count; ++i) {
        Triangle* triangle = &triangles[i];

		V3 a = triangle->a;
		V3 b = triangle->b;
		V3 c = triangle->c;

		V3 bv_min = min(a, min(b, c));
		V3 bv_max = max(a, max(b, c));

		V3 bv_p = 0.5f*(bv_min + bv_max);
		V3 bv_r = 0.5f*(bv_max - bv_min);

        BVHSortEntry* entry = &input_data[i];
        entry->index = (u32)i;
        entry->bv_p = bv_p;
        entry->bv_r = bv_r;
    }

    MeshBVH* result = push_struct(arena, MeshBVH);
    construct_bvh_internal(input_count, input_data, arena, method, &result->node_count, &result->nodes);

    result->storage = storage;
    switch (result->storage) {
        case BVHStorage_Scalar: {
            result->index_count = input_count;
            result->indices     = push_array(arena, result->index_count, u32, no_clear());
            for (usize i = 0; i < input_count; ++i) {
                result->indices[i] = input_data[i].index;
            }

            result->triangles = push_array(arena, triangle_count, Triangle, no_clear());
            for (usize i = 0; i < triangle_count; ++i) {
                u32 src_index = result->indices[i];
                result->triangles[i] = triangles[src_index];
            }
        } break;

        case BVHStorage_4x: {
            StretchyBuffer<u32> indices_4x = StretchyBuffer<u32>::on_arena(temp_arena);
            StretchyBuffer<Triangle_4x> triangles_4x = StretchyBuffer<Triangle_4x>::on_arena(temp_arena);

            for (usize node_index = 0; node_index < result->node_count; ++node_index) {
                BVHNode* node = &result->nodes[node_index];

                if (node->count) {
                    u32 first = node->left_first;
                    node->left_first = triangles_4x.count;

                    for (usize i = 0; i < node->count; i += 4) {
                        u32* indices = indices_4x.add_n(4);
                        Triangle_4x* dst = triangles_4x.add();
                        for (usize j = 0; j < Min(4, (usize)node->count - i); ++j) {
                            u32 src_index = input_data[first + i + j].index;
                            Triangle* src = &triangles[src_index];
                            indices[j] = src_index;
                            set(&dst->a, j, src->a);
                            set(&dst->b, j, src->b);
                            set(&dst->c, j, src->c);
                        }
                    }
                }
            }

            result->index_count  = indices_4x.count;
            result->indices      = push_copy_array(arena, indices_4x.count, indices_4x.data);
            result->triangles_4x = push_copy_array(arena, triangles_4x.count, triangles_4x.data);
        } break;
    }

    return result;
}
