#pragma once

struct ArenaAllocationParams {
    u32 align = 1;
    b32 clear = true;
};

inline ArenaAllocationParams
clear(u32 align = 0) {
    ArenaAllocationParams result = {};
    result.align = align;
    result.clear = true;
    return result;
}

inline ArenaAllocationParams
no_clear(u32 align = 0) {
    ArenaAllocationParams result = {};
    result.align = align;
    result.clear = false;
    return result;
}

//
// NOTE: These are macros because I want to be able to do tagged allocations, which means
//       passing additional info like __FILE__ and __LINE__. That's also why push_size_
//       isn't a method even though I decided to write the rest of the Arena using methods.
//       Not that it couldn't be a method, but then the macros would still be macros, so
//       it would just be inconsistent.
//

#define push_struct(arena, type, ...)       (type*)push_size_(arena, sizeof(type), alignof(type), ##__VA_ARGS__)
#define push_array(arena, count, type, ...) (type*)push_size_(arena, sizeof(type)*count, alignof(type), ##__VA_ARGS__)
#define push_size(arena, size, ...)                push_size_(arena, size, 1, ##__VA_ARGS__)

void* push_size_(struct Arena* arena, usize size, usize fallback_align, ArenaAllocationParams params = {});

struct Arena {
    usize capacity;
    usize committed;
    usize used;
    char* base;

    u32 temp_count;

    //

	inline usize
	get_align_offset(usize align) {
		usize offset = (usize)base & (align - 1);
		if (offset) {
			offset = align - offset; 
		}
		return offset;
	}

	inline char*
	get_next_allocation_location(usize align) {
		usize align_offset = get_align_offset(align);
		char* result = base + used + align_offset;
		return result;
	}

	inline usize
	get_size_remaining(usize align = 1) {
		usize align_offset = get_align_offset(align);
		usize result = capacity - (used + align_offset);
		return result;
	}

	inline void
	clear() {
		used = 0;
		temp_count = 0;
	}

	inline void
	reset_to(char* target_ptr) {
		assert((target_ptr >= base) && (target_ptr <= (base + used)));
		used = (target_ptr - base);
	}

	inline void
	init_with_memory(usize memory_size, void* memory) {
		zero_struct(this);
		capacity  = memory_size;
		// NOTE: There's an assumption here that the memory passed in is valid, committed memory.
		//       If you want an arena that exploits virtual memory to progressively commit, you
		//       shouldn't init it with any existing memory.
		committed = memory_size;
		base = (char*)memory;
	}

	inline void
	init_child_arena(Arena* child_arena, usize child_capacity, ArenaAllocationParams params = {}) {
        child_arena->init_with_memory(child_capacity, push_size_(this, child_capacity, 1, params));
	}

	inline void
	check() {
		assert(temp_count == 0);
	}
};

#define push_copy_size(arena, size, data, ...)                    push_copy_size_(arena, size, data, 1, ##__VA_ARGS__)
#define push_copy_struct(arena, src, ...)          (decltype(src))push_copy_size_(arena, sizeof(*(src)), src, alignof(decltype(*(src))), ##__VA_ARGS__)
#define push_copy_array(arena, count, data, ...)  (decltype(data))push_copy_size_(arena, sizeof(*(data))*(count), data, alignof(decltype(*(data))), ##__VA_ARGS__)

inline void*
push_copy_size_(Arena* arena, usize size, void* data, usize fallback_align, ArenaAllocationParams params = {}) {
	params.clear = false;

	void* result = push_size_(arena, size, fallback_align, params);
	copy_size(size, data, result);

	return result;
}

struct TemporaryMemory {
    Arena* arena;
    usize used;
};

inline TemporaryMemory
begin_temporary_memory(Arena* arena) {
    ++arena->temp_count;

    TemporaryMemory result;
    result.arena = arena;
    result.used  = arena->used;
    return result;
}

inline void
end_temporary_memory(TemporaryMemory temp) {
    Arena* arena = temp.arena;
	if (arena) {
		arena->used  = temp.used;
		--arena->temp_count;
	}
}

inline void
commit_temporary_memory(TemporaryMemory* temp) {
    --temp->arena->temp_count;
	zero_struct(temp);
}

struct ScopedMemory {
    TemporaryMemory temp;

    ScopedMemory(Arena* arena) {
        temp = begin_temporary_memory(arena);
    }

    ~ScopedMemory() {
        end_temporary_memory(temp);
    }

	force_inline
	operator TemporaryMemory*() { return &temp; }
};
