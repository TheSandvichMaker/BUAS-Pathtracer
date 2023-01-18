#include "precomp.h"

#include "common.h"
#include "platform.h"
#include "memory_arena.h"

usize g_platform_page_size;

#ifdef _WIN32
#include "platform_win32.inl"
#else
#error Only Win32 is supported right now.
#endif