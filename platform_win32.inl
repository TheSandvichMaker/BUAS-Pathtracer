static u64 g_win32_perf_freq;

void
platform_init() {
    SYSTEM_INFO system_info;
    GetSystemInfo(&system_info);

    g_platform_page_size = system_info.dwPageSize;

    LARGE_INTEGER perf_freq;
	QueryPerformanceFrequency(&perf_freq);

    g_win32_perf_freq = perf_freq.QuadPart;
}

void*
platform_allocate(usize size) {
    void* result = VirtualAlloc(0, size, MEM_RESERVE|MEM_COMMIT, PAGE_READWRITE);
    return result;
}

void*
platform_reserve(usize size) {
    void* result = VirtualAlloc(0, size, MEM_RESERVE, PAGE_NOACCESS);
    return result;
}

b32
platform_commit(void* address, usize size) {
    void* result = VirtualAlloc(address, size, MEM_COMMIT, PAGE_READWRITE);
    return (result != 0);
}

void
platform_free(void* memory) {
    if (memory) {
        VirtualFree(memory, 0, MEM_RELEASE);
    }
}

char*
platform_read_entire_file(Arena* arena, const char* file_name, usize* out_file_size) {
    char* result = 0;

	LARGE_INTEGER file_size;
    HANDLE file_handle = CreateFileA(file_name, GENERIC_READ, FILE_SHARE_READ, 0, OPEN_EXISTING, 0, 0);
    if (file_handle != INVALID_HANDLE_VALUE) {
		if (GetFileSizeEx(file_handle, &file_size)) {
            ScopedMemory scoped_memory(arena);
            result = push_array(arena, file_size.QuadPart + 1, char, no_clear());

            LONGLONG got_size;
			if (win32_sync_read(file_handle, 0, file_size.QuadPart, result, &got_size)) {
                if (out_file_size) *out_file_size = (usize)got_size;
                result[got_size] = 0;
                commit_temporary_memory(scoped_memory);
            }
        }

        CloseHandle(file_handle);
    }

    return result;
}

b32 
platform_write_entire_file(const char* file_name, usize size, const void* data) {
    b32 result = false;

    HANDLE file_handle = CreateFileA(file_name, GENERIC_WRITE, 0, 0, OPEN_ALWAYS, 0, 0);
    if (file_handle != INVALID_HANDLE_VALUE) {
        LONGLONG got_size;
        if (win32_sync_write(file_handle, 0, size, data, &got_size)) {
            if ((usize)got_size == size) {
				result = true;
            }
        }

        CloseHandle(file_handle);
    }

    return result;
}

PlatformHighResTime
platform_get_timestamp() {
    LARGE_INTEGER time;
    QueryPerformanceCounter(&time);

    PlatformHighResTime result;
    result.data_ = time.QuadPart;

    return result;
}

f64
platform_get_seconds_elapsed(PlatformHighResTime start, PlatformHighResTime end) {
    u64 diff = (end.data_ - start.data_);
    f64 result = (f64)diff / (f64)g_win32_perf_freq;
    return result;
}

PlatformSemaphore
platform_create_semaphore(u32 start_count, u32 max_count) {
    HANDLE semaphore = CreateSemaphoreA(0, start_count, max_count, 0);

    PlatformSemaphore result;
    result.opaque = (void*)semaphore;
    return result;
}

void
platform_destroy_semaphore(PlatformSemaphore semaphore) {
    /* ... */
    INVALID_CODE_PATH;
}

void
platform_release_semaphore(PlatformSemaphore semaphore, u32 count, u32* previous_count) {
    HANDLE win32_semaphore = (HANDLE)semaphore.opaque;

    LONG previous_count_;
    ReleaseSemaphore(win32_semaphore, count, &previous_count_);

    if (previous_count) *previous_count = (u32)previous_count_;
}

void
platform_wait_on_semaphore(PlatformSemaphore semaphore) {
    HANDLE win32_semaphore = (HANDLE)semaphore.opaque;
    WaitForSingleObject(win32_semaphore, INFINITE);
}

//
// NOTE: My thread creation passes a semaphore so that we can block the calling code
//       until we've safely copied out the thread parameters onto the stack, avoiding
//       having to allocate them. Thanks Martins.
// TODO: Consider WaitOnAddress (but it's Windows 8+)
//

struct Win32ThreadParameters {
    PlatformSemaphore semaphore;
    PlatformThreadProc proc;
    void* userdata;
};

static DWORD WINAPI
win32_thread_proc(void* win32_userdata) {
    Win32ThreadParameters* thread_parameters = (Win32ThreadParameters*)win32_userdata;

    PlatformSemaphore semaphore = thread_parameters->semaphore;
    PlatformThreadProc proc     = thread_parameters->proc;
    void* userdata              = thread_parameters->userdata;

    proc(userdata, semaphore);

    return EXIT_SUCCESS;
}

b32
platform_create_thread(PlatformThreadProc proc, void* userdata) {
    static PlatformSemaphore thread_creation_semaphore = platform_create_semaphore(0, 1);

    b32 result = false;

    Win32ThreadParameters thread_parameters = {};
    thread_parameters.semaphore = thread_creation_semaphore;
    thread_parameters.proc      = proc;
    thread_parameters.userdata  = userdata;

	HANDLE handle = CreateThread(0, 0, win32_thread_proc, &thread_parameters, 0, 0);
    platform_wait_on_semaphore(thread_creation_semaphore);

	if (handle && (handle != INVALID_HANDLE_VALUE)) {
		CloseHandle(handle);
		result = true;
	}

    return result;
}
