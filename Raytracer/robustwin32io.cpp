/**

Robust Win32 File IO
Copyright (C) 2020 Charles Bloom
Public domain.

**/

#pragma warning(disable: 26812)
#include "robustwin32io.h"

//---------------------------------------------

#define MAX_RETRIES	10

#define MAX_SINGLE_IO_SIZE (1<<22)

#ifndef MIN
#define MIN(a,b) ( (a) < (b) ? (a) : (b) )
#endif

//-----------------------------------------

/**

Demonstration of robust IO on Win32

Some things you need to do for robust IO :

1. Check every error code.

2. Cut large IOs into pieces.

3. Retry IOs in case of failure.

4. Do GetOverLappedResult(FALSE) (no wait) before calling GetOverLappedResult(TRUE) (wait)

5. SetLastError(0) to make sure it's blanked


-------------

Common errors you might see :

	1816 : ERROR_NOT_ENOUGH_QUOTA	 : not enough process space pages available
	1784 : ERROR_INVALID_USER_BUFFER
	8	 : ERROR_NOT_ENOUGH_MEMORY	 : too many asyncs already pending 
	1450 : ERROR_NO_SYSTEM_RESOURCES  : failure to alloc pages in kernel address spage

	87 : INVALID_PARAMETER : if you're not aligned when you should be

**/


/**

Windows docs :

An application can determine a volume sector size by calling the GetDiskFreeSpace function.

Because buffer addresses for read and write operations must be sector-aligned, the application must 
have direct control of how these buffers are allocated. One way to sector-align buffers is to 
use the VirtualAlloc function to allocate the buffers. Consider the following:

    * VirtualAlloc allocates memory that is aligned on addresses that are integer multiples of the 
	system's page size. Page size is 4,096 bytes on x64 and x86 or 8,192 bytes for the Intel Itanium 
	processor family. For additional information, see the GetSystemInfo function.
    * Sector size is typically 512 to 4,096 bytes for direct-access storage devices (hard drives) 
	and 2,048 bytes for CD-ROMs.
    * Both page and sector sizes are powers of 2.

#define XBOX_HD_SECTOR_SIZE   512
#define XBOX_DVD_SECTOR_SIZE  2048
#define XBOX_MU_SECTOR_SIZE   4096

->

CB : 4096 is almost gauranteed to work for unbuffered IO regardless of sector size
because lots of legacy apps hard-code it.  (so we will do the same)

**/

/**

Large IO's and too many IO's can both fail due to failure to allocate resources in the kernel/driver
(this is rarely a problem in 64-bit, it was a common problem in 32-bit)

We limit the size of single IO calls to MAX_SINGLE_IO_SIZE

-----

ERROR_NOT_ENOUGH_MEMORY = too many AsyncIO 's pending

	-> I could spin until some finish

ERROR_NOT_ENOUGH_QUOTA  = single IO call too large

	-> SetProcessWorkingSetSize
	
	hmm this is not it

ERROR_NO_SYSTEM_RESOURCES = 

	!! this is it
	
	failure to alloc pages in the kernel address space for the IO

----

MAX_SINGLE_IO_SIZE should be somewhere around 1<<20 to 1<<24
	if you make this too small you generate too many IO requests (which can make windows ERROR_NOT_ENOUGH_MEMORY)
	too big and you fail to alloc in the kernel (ERROR_NO_SYSTEM_RESOURCES)
	
**/

win32_io_status win32_async_read(HANDLE handle, LONGLONG position, DWORD size, 
							void * toMemory,
							OVERLAPPED * data,
							DWORD * pSize )
{
	memset(data,0,sizeof(OVERLAPPED));
	
	LARGE_INTEGER offset;
	offset.QuadPart = position;
	data->Offset = offset.LowPart;
	data->OffsetHigh = offset.HighPart;

	/*
	
	There's a bug in Windows XP where ReadFile/WriteFile returns error but doesn't set LastError
	  so we must blank it before calling ReadFile :
	(fixed in Vista+)
		
	*/
	SetLastError(0);
	
	/*BOOL ret =*/
	DWORD actualRead = 0;
	BOOL ok1 = ::ReadFile(	handle,
				toMemory,
				size,
				&actualRead,
				data);

	*pSize = actualRead;
		
	if ( ok1 )
	{
		//  weird case - ReadFile returned done immediately, not overlapped !?
		// NOT ASYNC
		memset(data,0,sizeof(OVERLAPPED));
		
		DWORD err = GetLastError();
		
		if ( err == ERROR_HANDLE_EOF )
			return win32_io_done_sync_eof; // @@ ?? can this ever happen ?
		else
			return win32_io_done_sync;
	}
	else
	{
		DWORD err = GetLastError();
		
		// 998 == ERROR_NOACCESS
		
		if ( err == ERROR_IO_PENDING )
		{
			// normal good async read
			// drop through and wait on it
			return win32_io_started_async;
		}
		else if ( err == ERROR_HANDLE_EOF )
		{
			// okay, sort of ? (we were at EOF at start of Read)
			// NOT ASYNC
			memset(data,0,sizeof(OVERLAPPED));
			return win32_io_done_sync_eof;
		}
		else
		{
			// bad error, 
			return win32_io_error;
		}
	}
}

win32_io_status win32_async_write(HANDLE handle, LONGLONG position, DWORD size, 
							const void * memory,
							OVERLAPPED * data,
							DWORD * pSize )
{
	memset(data,0,sizeof(OVERLAPPED));
	
	LARGE_INTEGER offset;
	offset.QuadPart = position;
	data->Offset = offset.LowPart;
	data->OffsetHigh = offset.HighPart;

	// wipe last error; see previous note
	SetLastError(0);

	/*

	The WriteFile function may fail with ERROR_INVALID_USER_BUFFER or
	 ERROR_NOT_ENOUGH_MEMORY whenever there are too many outstanding 
	 asynchronous I/O requests. 

	*/

	/*BOOL ret =*/
	DWORD actualWrote = 0;
	BOOL ok1 = ::WriteFile(	handle,
				memory,
				size,
				&actualWrote,
				data);

	*pSize = (DWORD)actualWrote;
		
	if ( ok1 )
	{
		// WriteFile returned done immediately, not overlapped !?
		
		// almost ALL writes go through here
		// http://support.microsoft.com/kb/156932
	
		// for Writes to actually be async you must have :
		//	used unbuffered IO
		//	pre-extended the size of file
		//	validated that range with SetFileValidData
		// and it still may not actually be async
	
		// NOT ASYNC
		memset(data,0,sizeof(OVERLAPPED));
		return win32_io_done_sync;
	}
	else
	{
		DWORD err = GetLastError();
		
		if ( err == ERROR_IO_PENDING )
		{
			// normal good async write
			// drop through and wait on it
			return win32_io_started_async;
		}
		else
		{
			// bad error, 
			return win32_io_error;
		}
	}
}

win32_io_status win32_get_async_result(HANDLE handle,
							OVERLAPPED * data,
							DWORD * pSize)
{
	// only call this if you got "win32_io_started_async"
	// so you know IO is actually pending
			
	DWORD dwSize = 0;
	
	// first check result with no wait
	//	this also resets the event so that the next call to GOR works :
	
	if ( GetOverlappedResult(handle,data,&dwSize,FALSE) )
	{
		// @@ if dwSize == 0 ?

		if ( dwSize > 0 )
		{
			*pSize = dwSize;
			return win32_io_done_sync;
		}
	}	
	
	// if you don't do the GOR(FALSE)
	//	then the GOR(TRUE) call here can return even though the IO is not actually done
	
	// call GOR with TRUE -> this yields our thread if the IO is still pending
	if ( ! GetOverlappedResult(handle,data,&dwSize,TRUE) )
	{
		DWORD err = GetLastError();
		
		if ( err == ERROR_HANDLE_EOF )
		{
			*pSize = dwSize;
			return win32_io_done_sync_eof;
		}
		
		return win32_io_error;		
	}
		
	*pSize = dwSize;
	
	return win32_io_done_sync;	
}			

static win32_io_status win32_sync_read_sub(HANDLE handle, LONGLONG pos, DWORD size, void * memory, DWORD * pGotSize)
{
	if ( pGotSize )
		*pGotSize = 0;
		
	if ( size == 0 ) return win32_io_done_sync;
		
	for(int numRetries=0;numRetries<MAX_RETRIES;numRetries++)
	{
		OVERLAPPED async = { 0 };
		DWORD gotSize = 0;
		
		win32_io_status st = win32_async_read( handle, pos , size, memory, &async , &gotSize);
		
		if ( st == win32_io_started_async )
		{
			// if started async, wait for it now
			st = win32_get_async_result(handle,&async,&gotSize);
		}

		// st now != win32_io_started_async

		if ( st == win32_io_done_sync_eof )
		{
			if ( pGotSize ) *pGotSize = gotSize;
			return st;
		}
		else if ( st == win32_io_done_sync )
		{
			if ( gotSize == 0 )
			{
				continue; // retry
			}

			// successful read returns here :
			if ( pGotSize ) *pGotSize = gotSize;
			return st;
		}

		// st == win32_io_error;
		
		// some error :
		DWORD err = GetLastError();
		
		// see comments about these error codes elsewhere :
		if ( err == ERROR_NO_SYSTEM_RESOURCES ||	
			err == ERROR_NOT_ENOUGH_MEMORY )
		{
			DWORD millis = 1 + numRetries*10;
			millis = MIN(millis,50);
			
			Sleep(millis);
			
			// retry ...
		}
		else
		{
			// unexpected error, don't retry
			return win32_io_error;
		}
	}
	
	// hit retry limit
	return win32_io_error;	
}


static win32_io_status win32_sync_write_sub(HANDLE handle, LONGLONG pos, DWORD size, const void * memory, DWORD * pGotSize)
{
	if ( pGotSize )
		*pGotSize = 0;
		
	if ( size == 0 ) return win32_io_done_sync;

	for(int numRetries=0;numRetries<MAX_RETRIES;numRetries++)
	{
		OVERLAPPED async = { 0 };
		DWORD gotSize = 0;
		
		win32_io_status st = win32_async_write( handle, pos , size, memory, &async, &gotSize);
		
		if ( st == win32_io_started_async )
		{
			// if started async, wait for it now
			st = win32_get_async_result(handle,&async,&gotSize);
		}

		// st now != win32_io_started_async

		if ( st == win32_io_done_sync || st == win32_io_done_sync_eof )
		{
			if ( pGotSize ) *pGotSize = gotSize;
			return st;
		}
		
		// some error :
		DWORD err = GetLastError();
		
		// see comments about these error codes elsewhere :
		// ERROR_INVALID_USER_BUFFER ?
		// ERROR_NOT_ENOUGH_QUOTA ?
		if ( err == ERROR_NO_SYSTEM_RESOURCES ||	
			err == ERROR_NOT_ENOUGH_MEMORY )
		{			
			DWORD millis = 1 + numRetries*10;
			millis = MIN(millis,50);
			
			Sleep(millis);
			
			// retry ...
		}
		else
		{
			// unexpected error, don't retry
			return win32_io_error;
		}
	}
	
	// hit retry limit
	return win32_io_error;
}

/**

win32_sync_read
for a partial read, *pGotSize is filled and return value is TRUE

alignment == 1 for buffered files  (0 changes to 1)
alignment == WINDOWS_UNBUFFERED_ALIGNMENT for unbuffered

**/

BOOL win32_sync_read(HANDLE handle, LONGLONG pos, LONGLONG size, void * memory, LONGLONG * pGotSize, DWORD alignment /*= 1*/)
{
	if ( pGotSize ) *pGotSize = 0;

	char * pmem = (char *)memory;

	if ( alignment == 0 ) alignment = 1;
	// starting off alignment :
	if ( (pos&(alignment-1)) != 0 ) return FALSE;

	while(size > 0 )
	{
		DWORD cursize = (DWORD) MIN(size,MAX_SINGLE_IO_SIZE);
	
		DWORD curgotsize = 0;
		win32_io_status st = win32_sync_read_sub(handle,pos,cursize,pmem,&curgotsize);
		
		if ( st == win32_io_error )
			return FALSE;
		
		if ( st == win32_io_done_sync_eof )
		{
			// hit eof, not a failure
			// however note that *pGotSize may be less than original requested size
			if ( pGotSize ) *pGotSize += curgotsize;
			return TRUE;
		}

		// st == win32_io_done_sync

		// got a success, but no bytes when we wanted some; error
		if ( curgotsize == 0 )
		{
			return FALSE;
		}

		pos += curgotsize;
		pmem += curgotsize;
		size -= curgotsize;
		if ( pGotSize ) *pGotSize += curgotsize;
		
		// if got off alignment due to a partial IO, it must be due to EOF, return true
		// @@ ??
		if ( (pos&(alignment-1)) != 0 ) return TRUE;
	}

	return TRUE;
}

BOOL win32_sync_write(HANDLE handle, LONGLONG pos, LONGLONG size, const void * memory, LONGLONG * pGotSize, DWORD alignment /*= 1*/)
{
	if ( pGotSize ) *pGotSize = 0;

	char * pmem = (char *)memory;

	if ( alignment == 0 ) alignment = 1;
	// starting off alignment :
	if ( (pos&(alignment-1)) != 0 ) return FALSE;
	
	while(size > 0 )
	{
		DWORD cursize = (DWORD) MIN(size,MAX_SINGLE_IO_SIZE);
	
		DWORD curgotsize = 0;
		win32_io_status st = win32_sync_write_sub(handle,pos,cursize,pmem,&curgotsize);
		
		if ( st == win32_io_error )
			return FALSE;
			
		if ( st == win32_io_done_sync_eof )
		{
			// @@ EOF during write? possible with read-write file in non-append mode?
			size -= curgotsize;

			// hit eof, not a failure
			// however note that *pGotSize may be less than original requested size
			if ( pGotSize ) *pGotSize += curgotsize;

			if ( size == 0 )
				return TRUE;
			else
				return FALSE; // couldn't write all requested bytes
		}
		
		// st == win32_io_done_sync

		// got a success, but no bytes when we wanted some; error
		if ( curgotsize == 0 )
		{
			return FALSE;
		}
		
		pos += curgotsize;
		pmem += curgotsize;
		size -= curgotsize;
		if ( pGotSize ) *pGotSize += curgotsize;
		
		// if got off alignment due to a partial IO, it must be due to EOF, return true
		// @@ ??
		if ( (pos&(alignment-1)) != 0 ) return TRUE;
	}

	return TRUE;
}

//=========================================================================================

#ifdef DO_TEST

BOOL do_test(
	const char * fm_name,BOOL fm_buffered,
	const char * to_name,BOOL to_buffered)
{
	DWORD fm_flags = FILE_FLAG_OVERLAPPED;

	// WINDOWS_UNBUFFERED_ALIGNMENT

	if ( ! fm_buffered )
	{
		fm_flags |= FILE_FLAG_NO_BUFFERING;
	}
			
	HANDLE fm_handle = CreateFileA(	fm_name, 
									GENERIC_READ, 
									FILE_SHARE_READ,
									//0,
									NULL, 
									OPEN_EXISTING,
									fm_flags,
									NULL );

	if ( fm_handle == INVALID_HANDLE_VALUE )
		return FALSE;
	
	DWORD to_flags = FILE_FLAG_OVERLAPPED ;
		
	if ( ! to_buffered )
	{
		to_flags |= FILE_FLAG_NO_BUFFERING;
	}
	
	HANDLE to_handle = CreateFileA(	to_name,
									GENERIC_WRITE, 
									0, // share
									NULL, 
									CREATE_ALWAYS, 
									to_flags,
									NULL );
		
	if ( to_handle == INVALID_HANDLE_VALUE )
		return FALSE;

	//----------------------------------------
	
	LARGE_INTEGER fs;
	if ( ! GetFileSizeEx(fm_handle,&fs) )
	{
		return FALSE;
	}
	LONGLONG size = fs.QuadPart;
	
	DWORD alignment = 1;
	
	if ( ! fm_buffered || ! to_buffered)
	{
		alignment= WINDOWS_UNBUFFERED_ALIGNMENT;
	}
	
	size = (size+alignment-1)&(~(LONGLONG)(alignment-1));
	
	void * mem = VirtualAlloc(NULL,(SIZE_T)size,MEM_COMMIT|MEM_RESERVE,PAGE_READWRITE);
	if ( ! mem ) 
	{
		return FALSE;
	}
	
	LONGLONG read_size;
	BOOL read_ok = win32_sync_read(fm_handle,0,size,mem,&read_size,alignment);
	
	if ( ! read_ok )
		return FALSE;
	
	if ( read_size != size )
	{
		if ( (size - read_size) >= alignment )
			return FALSE;
	}
	
	LONGLONG write_size;
	BOOL write_ok = win32_sync_write(to_handle,0,size,mem,&write_size,alignment);
	
	if ( ! write_ok )
		return FALSE;
	
	if ( write_size != size )
	{
		if ( (size - write_size) >= alignment )
			return FALSE;
	}
			
	VirtualFree(mem,0,MEM_RELEASE);
	
	//----------------------------------------			

	if ( ! CloseHandle(fm_handle) )
		return FALSE;
		
	if ( ! CloseHandle(to_handle) )
		return FALSE;
		
	return TRUE;
}

#include <stdio.h>

int main(int argc,char *argv[])
{
	argc; argv;
	
	if ( argc < 3 )
	{
		fprintf(stderr,"not enough args\n");
		return 10;
	}
	
	const char * fmn = argv[1];
	const char * ton = argv[2];
	
	fprintf(stderr,"fm: %s to: %s\n",fmn,ton);

	// try all permutations of fm & to buffered & unbuffered :
	for(int fmb=0;fmb<=1;fmb++)
	{
		for(int tob=0;tob<=1;tob++)
		{
			if ( ! do_test(fmn,fmb,ton,tob) )
			{
				fprintf(stderr,"test failed\n");
				return 10;
			}
		}
	}
	
	fprintf(stderr,"test succeeded\n");
				
	return 0;
}
			
#endif
