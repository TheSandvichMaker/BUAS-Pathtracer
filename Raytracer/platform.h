#pragma once

extern usize g_platform_page_size;

void  platform_init();
void* platform_allocate(usize size);
void* platform_reserve(usize size);
b32   platform_commit(void* address, usize size);
void  platform_free(void* memory);

// NOTE: Returns null-terminated memory
char* platform_read_entire_file(struct Arena* arena, const char* file_name, usize* out_file_size = 0);
b32   platform_write_entire_file(const char* file_name, usize size, const void* data);

struct PlatformHighResTime {
	u64 data_;
};

PlatformHighResTime platform_get_timestamp();
f64 platform_get_seconds_elapsed(PlatformHighResTime start, PlatformHighResTime end);

struct PlatformSemaphore {
	void* opaque;
};

PlatformSemaphore platform_create_semaphore(u32 start_count, u32 max_count);
void platform_destroy_semaphore(PlatformSemaphore semaphore);
void platform_release_semaphore(PlatformSemaphore semaphore, u32 count = 1, u32* previous_count = 0);
void platform_wait_on_semaphore(PlatformSemaphore semaphore);

typedef void (*PlatformThreadProc)(void* userdata, PlatformSemaphore semaphore);

// NOTE: Once the user has copied out their thread parameters, they must resume the main thread
//       by releasing the passed in semaphore.
b32 platform_create_thread(PlatformThreadProc proc, void* userdata);

//
// NOTE: Intrinsics
//

#if _WIN32
#define WRITE_BARRIER _WriteBarrier()
#define  READ_BARRIER  _ReadBarrier()

inline u32
atomic_add(volatile u32* addend, s32 value) {
	u32 result = (u32)_InlineInterlockedAdd((volatile LONG*)addend, value);
	return result;
}

inline s32
atomic_add(volatile s32* addend, s32 value) {
	s32 result = (s32)_InlineInterlockedAdd((volatile LONG*)addend, value);
	return result;
}

inline u64
atomic_add(volatile u64* addend, u64 value) {
	u64 result = (u64)_InlineInterlockedAdd64((volatile LONG64*)addend, value);
	return result;
}

inline s64
atomic_add(volatile s64* addend, s64 value) {
	s64 result = (s64)_InlineInterlockedAdd64((volatile LONG64*)addend, value);
	return result;
}
#endif
