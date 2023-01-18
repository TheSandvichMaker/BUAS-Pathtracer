#include "precomp.h"
#include "common.h"

void*
push_size_(Arena* arena, usize size, usize fallback_align, ArenaAllocationParams params) {
    if (!arena->capacity) {
        assert(!arena->base);
        arena->capacity = Gigabytes(128);
    }

    if (!arena->base) {
        // NOTE: Let's align up to page size because that's the minimum allocation granularity anyway,
        //       and the code doing the commit down below assumes our capacity is page aligned.
        arena->capacity = AlignPow2(arena->capacity, g_platform_page_size);
        arena->base = (char*)platform_reserve(arena->capacity);
    }

    usize align = params.align;
    if (align == 0) align = fallback_align;

    usize align_offset = arena->get_align_offset(align);
    usize aligned_size = size + align_offset;

    assert((arena->used + aligned_size) <= arena->capacity);

    char* unaligned_base = arena->base + arena->used;

    if (arena->committed < (arena->used + aligned_size)) {
        usize commit_size = AlignPow2(aligned_size, g_platform_page_size);
        platform_commit(arena->base + arena->committed, commit_size);
        arena->committed += commit_size;
        assert(arena->committed >= (arena->used + aligned_size));
    }

    void* result = unaligned_base + align_offset;
    arena->used += aligned_size;

    if (params.clear) {
        zero_size(aligned_size, result);
    }

    return result;
}
