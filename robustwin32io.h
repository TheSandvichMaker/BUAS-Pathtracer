#pragma once

/**

Robust Win32 File IO
Copyright (C) 2020 Charles Bloom
Public domain.

**/

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN 1
#endif

#include <windows.h>

#define win32_io_notaligned	1

// returns FALSE if an error was encountered
//	fills *pGotSize for partial IO size
//	if you expect only full IO's then check size == *pGotSize outside
// note you may get a return of FALSE even if some bytes were successfully read/written
// EOF does not count as an error (may return TRUE)
// return == TRUE/FALSE and size == *pGotSize are independent
// alignment == 1 (win32_io_notaligned) for buffered files  (0 changes to 1)
// alignment == WINDOWS_UNBUFFERED_ALIGNMENT for unbuffered
extern BOOL win32_sync_read( HANDLE handle, LONGLONG pos, LONGLONG size, void * memory, LONGLONG * pGotSize, DWORD alignment = win32_io_notaligned);
extern BOOL win32_sync_write(HANDLE handle, LONGLONG pos, LONGLONG size, const void * memory, LONGLONG * pGotSize, DWORD alignment = win32_io_notaligned);

//====================================================================

// if you open with FILE_FLAG_NO_BUFFERING , then reads & writes must be at this alignment :
#define WINDOWS_UNBUFFERED_ALIGNMENT 4096

/**

win32_async_read returns a trinary status :

**/

enum win32_io_status
{
	win32_io_started_async,
	win32_io_done_sync,
	win32_io_done_sync_eof,
	win32_io_error
};

win32_io_status win32_async_read(HANDLE handle, LONGLONG position, DWORD size, 
							void * toMemory,
							OVERLAPPED * data,
							DWORD * pSize );
							
win32_io_status win32_async_write(HANDLE handle, LONGLONG position, DWORD size, 
							const void * memory,
							OVERLAPPED * data,
							DWORD * pSize );
							
// only call if you got win32_io_started_async :
win32_io_status win32_get_async_result(HANDLE handle,
							OVERLAPPED * data,
							DWORD * pSize);

//====================================================================
