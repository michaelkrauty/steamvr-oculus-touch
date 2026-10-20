// Copyright 2013, Fredrik Hultin.
// Copyright 2013, Jakob Bornecrantz.
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */

/* Platform Specific Functions, Win32 Implementation */


#ifdef _WIN32

#define WIN32_LEAN_AND_MEAN
#define WIN32_EXTRA_LEAN

#include <windows.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <wctype.h>
#include "platform-win32-cache.h"

#include "platform.h"
#include "openhmdi.h"

/* cv1touch: the monotonic clock is QueryPerformanceCounter in nanoseconds, computed exactly as
 * cv1touchd's now_ns (ticks * 1e9 / frequency), so engine timestamps share the daemon's clock as
 * CLOCK_MONOTONIC does on Linux. The upstream code used the wall clock in 100 ns units. */
static uint64_t qpc_ns(void)
{
	/* fixed at boot and cheap to read: no cache to share between threads */
	LARGE_INTEGER f, t;
	QueryPerformanceFrequency(&f);
	QueryPerformanceCounter(&t);
	const uint64_t freq = f.QuadPart > 0 ? (uint64_t)f.QuadPart : 1;
	const uint64_t ticks = (uint64_t)t.QuadPart;
	return ticks / freq * 1000000000ull + ticks % freq * 1000000000ull / freq;
}

double ohmd_get_tick()
{
	return (double)qpc_ns() / 1e9;
}

static const uint64_t NUM_1_000_000_000 = 1000000000;

void ohmd_monotonic_init(ohmd_context* ctx)
{
	ctx->monotonic_ticks_per_sec = NUM_1_000_000_000;
}

uint64_t ohmd_monotonic_get(ohmd_context* ctx)
{
	(void)ctx;
	return qpc_ns();
}

#ifndef CREATE_WAITABLE_TIMER_HIGH_RESOLUTION
#define CREATE_WAITABLE_TIMER_HIGH_RESOLUTION 0x00000002
#endif

/* cv1touch: sub-millisecond sleeps through a high-resolution waitable timer (Windows 10 1803+);
 * Sleep() rounds up to the scheduler tick. */
OHMD_APIENTRYDLL void OHMD_APIENTRY ohmd_sleep(double seconds)
{
	if (seconds <= 0)
		return;
	HANDLE timer = CreateWaitableTimerExW(NULL, NULL, CREATE_WAITABLE_TIMER_HIGH_RESOLUTION, TIMER_ALL_ACCESS);
	if (timer) {
		LARGE_INTEGER due;
		due.QuadPart = -(LONGLONG)(seconds * 1e7); /* relative, 100 ns units */
		if (SetWaitableTimer(timer, &due, 0, NULL, NULL, FALSE)) {
			WaitForSingleObject(timer, INFINITE);
			CloseHandle(timer);
			return;
		}
		CloseHandle(timer);
	}
	Sleep((DWORD)(seconds * 1000 + 0.5));
}

// threads

struct ohmd_thread {
	HANDLE handle;
	void* arg;
	unsigned int (*routine)(void* arg);
};

struct ohmd_mutex {
	CRITICAL_SECTION lock;
};

struct ohmd_cond {
	CONDITION_VARIABLE cond;
};

DWORD __stdcall ohmd_thread_wrapper(void* t)
{
	ohmd_thread* thread = (ohmd_thread*)t;
	return thread->routine(thread->arg);
}

ohmd_thread* ohmd_create_thread(ohmd_context* ctx, unsigned int (*routine)(void* arg), void* arg)
{
	ohmd_thread* thread = ohmd_alloc(ctx, sizeof(ohmd_thread));
	if(!thread)
		return NULL;

	thread->routine = routine;
	thread->arg = arg;

	thread->handle = CreateThread(NULL, 0, ohmd_thread_wrapper, thread, 0, NULL);

	return thread;
}

void ohmd_destroy_thread(ohmd_thread* thread)
{
	/* cv1touch: the upstream 3 s sleep before joining delayed every engine shutdown */
	WaitForSingleObject(thread->handle, INFINITE);
	CloseHandle(thread->handle);
	free(thread);
}

ohmd_mutex* ohmd_create_mutex(ohmd_context* ctx)
{
	ohmd_mutex* mutex = ohmd_alloc(ctx, sizeof(ohmd_mutex));
	if(!mutex)
		return NULL;
	
	InitializeCriticalSection(&mutex->lock);

	return mutex;
}

void ohmd_destroy_mutex(ohmd_mutex* mutex)
{
	DeleteCriticalSection(&mutex->lock);
	free(mutex);
}

void ohmd_lock_mutex(ohmd_mutex* mutex)
{
	if(mutex)
		EnterCriticalSection(&mutex->lock);
}

void ohmd_unlock_mutex(ohmd_mutex* mutex)
{
	if(mutex)
		LeaveCriticalSection (&mutex->lock);
}

ohmd_cond* ohmd_create_cond(ohmd_context* ctx)
{
	ohmd_cond* cond = ohmd_alloc(ctx, sizeof(ohmd_cond));
	if(!cond)
		return NULL;
	
	InitializeConditionVariable (&cond->cond);

	return cond;
}

void ohmd_destroy_cond(ohmd_cond* cond)
{
  /* The condition variable must not be in use. In that case,
   * there's no need to do anything to clear condition variables, just free it */
	free(cond);
}

void ohmd_cond_wait(ohmd_cond* cond, ohmd_mutex* mutex)
{
	if(cond && mutex)
		SleepConditionVariableCS(&cond->cond, &mutex->lock, INFINITE);
}

void ohmd_cond_signal(ohmd_cond* cond)
{
	if (cond)
		WakeConditionVariable(&cond->cond);
}

void ohmd_cond_broadcast(ohmd_cond* cond)
{
	if (cond)
		WakeAllConditionVariable(&cond->cond);
}

int findEndPoint(char* path, int endpoint)
{
	char comp[8];
	sprintf(comp,"mi_0%d",endpoint);

	if (strstr(path, comp) != NULL) {
		return 1;
	}

	return 0;
}

/* SetThreadDescription is Windows 10 1607 and later: looked up once, not linked */
typedef HRESULT (WINAPI *set_description_fn)(HANDLE, PCWSTR);
static set_description_fn set_description;
static INIT_ONCE set_description_once = INIT_ONCE_STATIC_INIT;

static BOOL CALLBACK look_up_set_description(PINIT_ONCE once, PVOID parameter, PVOID *context)
{
	(void)once;
	(void)parameter;
	(void)context;
	HMODULE kernel = GetModuleHandleW(L"kernel32.dll");
	if (kernel)
		set_description = (set_description_fn)(void (*)(void))GetProcAddress(kernel, "SetThreadDescription");
	return TRUE;
}

void ohmd_set_thread_name(const char* name)
{
	wchar_t wide[64];
	if (InitOnceExecuteOnce(&set_description_once, look_up_set_description, NULL, NULL) && set_description &&
	    MultiByteToWideChar(CP_UTF8, 0, name, -1, wide, 64) > 0)
		set_description(GetCurrentThread(), wide);
}

/* cv1touch: never stop or start Oculus services (upstream stopped OVRService here, which needs
 * administrator rights). cv1touchd detects a running Oculus runtime itself and asks the user. */
void ohmd_toggle_ovr_service(int state)
{
	(void)state;
}

/* cv1touch: the file helpers the POSIX layer has, which openhmd.c's config cache needs. Paths are
 * UTF-8, as on Linux, and wide at the Windows API, so an account name outside the ANSI code page
 * works. */
static wchar_t *widen(const char *utf8)
{
	const int n = MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, utf8, -1, NULL, 0);
	if (n <= 0) {
		errno = EINVAL;
		return NULL;
	}
	wchar_t *wide = malloc((size_t)n * sizeof *wide);
	if (!wide) {
		errno = ENOMEM;
		return NULL;
	}
	if (MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, utf8, -1, wide, n) != n) {
		free(wide);
		errno = EINVAL;
		return NULL;
	}
	ohmd_win32_path_separators(wide);
	return wide;
}

static FILE *open_utf8(const char *filename, const wchar_t *mode)
{
	wchar_t *wide = widen(filename);
	if (!wide)
		return NULL; /* widen() has set errno: EINVAL for bad UTF-8, ENOMEM */
	FILE *f = _wfopen(wide, mode);
	free(wide);
	return f;
}

int ohmd_read_file(const char* filename, char **out_buf, unsigned long *out_len)
{
	FILE* f = open_utf8(filename, L"rb");
	if (f == NULL)
		return errno ? errno : EIO;
	int ret = 0;
	char *buffer = NULL;
	long file_len;
	if (fseek(f, 0, SEEK_END) != 0 || (file_len = ftell(f)) < 0 || fseek(f, 0, SEEK_SET) != 0) {
		ret = errno ? errno : EIO;
		goto fail;
	}
	buffer = malloc((size_t)file_len + 1);
	if (!buffer) {
		ret = ENOMEM;
		goto fail;
	}
	if (fread(buffer, 1, (size_t)file_len, f) != (size_t)file_len) {
		ret = ferror(f) ? EIO : 0;
		if (ret)
			goto fail;
	}
	buffer[file_len] = 0;
	*out_buf = buffer;
	*out_len = (unsigned long)file_len;
	buffer = NULL;
fail:
	free(buffer);
	fclose(f);
	return ret;
}

/* Creates missing components after the drive/UNC root, preserving Win32 errors. */
int ohmd_ensure_path(const char* pathname)
{
	char pathbuf[OHMD_STR_SIZE + 1];
	if (strlen(pathname) < 2 || strlen(pathname) > OHMD_STR_SIZE)
		return -EINVAL;
	strcpy(pathbuf, pathname);
	int start = ohmd_win32_path_start(pathbuf);
	if (start < 0) return -EINVAL;
	if (!pathbuf[start]) return 0;
	for (char *cur = pathbuf + start;; cur++) {
		const char c = *cur;
		if (!ohmd_win32_separator(c) && c != 0)
			continue;
		if (cur > pathbuf + start && !ohmd_win32_separator(cur[-1])) {
			*cur = 0;
			wchar_t *wide = widen(pathbuf);
			if (!wide)
				return -(errno ? errno : EINVAL);
			DWORD attributes = GetFileAttributesW(wide);
			int error = 0;
			if (attributes == INVALID_FILE_ATTRIBUTES) {
				DWORD reason = GetLastError();
				if (reason != ERROR_FILE_NOT_FOUND && reason != ERROR_PATH_NOT_FOUND) {
					error = ohmd_win32_error(reason);
				} else if (!CreateDirectoryW(wide, NULL)) {
					reason = GetLastError();
					if (reason == ERROR_ALREADY_EXISTS) {
						attributes = GetFileAttributesW(wide);
						error = attributes == INVALID_FILE_ATTRIBUTES ? ohmd_win32_error(GetLastError()) :
							(attributes & FILE_ATTRIBUTE_DIRECTORY ? 0 : ENOTDIR);
					} else error = ohmd_win32_error(reason);
				}
			} else if (!(attributes & FILE_ATTRIBUTE_DIRECTORY)) {
				error = ENOTDIR;
			}
			free(wide);
			if (error) return -error;
			*cur = c;
		}
		if (c == 0) return 0;
	}
}

/* Publishes only a completely written cache: a reader sees the old file or the whole new one. No
 * FlushFileBuffers and no write-through move: this runs on the engine thread, where waiting for the
 * disk can stall tracking, and a cache torn by a power cut fails to parse and is read from the
 * controller again. */
int ohmd_write_file(const char* filename, char *buf, unsigned long buf_len)
{
	static LONG sequence;
	wchar_t *wide = widen(filename), *temporary = NULL;
	HANDLE file = INVALID_HANDLE_VALUE;
	bool created = false;
	int ret = 0;
	if (!wide) return -(errno ? errno : EINVAL);
	const size_t count = wcslen(wide) + 48;
	temporary = malloc(count * sizeof *temporary);
	if (!temporary) {
		ret = -ENOMEM;
		goto out;
	}
	/* CREATE_NEW owns a unique sibling without ever truncating an existing file. */
	for (;;) {
		int n = swprintf(temporary, count, L"%ls.tmp-%lu-%lu", wide, GetCurrentProcessId(),
			(unsigned long)InterlockedIncrement(&sequence));
		if (n < 0 || (size_t)n >= count) {
			ret = -ENAMETOOLONG;
			goto out;
		}
		file = CreateFileW(temporary, GENERIC_WRITE, 0, NULL, CREATE_NEW, FILE_ATTRIBUTE_NORMAL, NULL);
		if (file != INVALID_HANDLE_VALUE) break;
		DWORD error = GetLastError();
		if (error != ERROR_FILE_EXISTS && error != ERROR_ALREADY_EXISTS) {
			ret = -ohmd_win32_error(error);
			goto out;
		}
	}
	created = true;
	DWORD written = 0;
	if (!WriteFile(file, buf, (DWORD)buf_len, &written, NULL))
		ret = -ohmd_win32_error(GetLastError());
	else if (written != buf_len)
		ret = -EIO;
	if (!CloseHandle(file) && ret == 0)
		ret = -ohmd_win32_error(GetLastError());
	file = INVALID_HANDLE_VALUE;
	if (ret == 0 && !MoveFileExW(temporary, wide, MOVEFILE_REPLACE_EXISTING))
		ret = -ohmd_win32_error(GetLastError());
	if (ret == 0) created = false;
out:
	if (file != INVALID_HANDLE_VALUE && !CloseHandle(file) && ret == 0)
		ret = -ohmd_win32_error(GetLastError());
	if (created) DeleteFileW(temporary);
	free(temporary);
	free(wide);
	return ret;
}

static int absolute(const wchar_t *path)
{
	return (iswalpha(path[0]) && path[1] == L':' && (path[2] == L'\\' || path[2] == L'/')) ||
	       (path[0] == L'\\' && path[1] == L'\\');
}

/* %LOCALAPPDATA%\openhmd (the temp directory's openhmd when LOCALAPPDATA is unset or relative),
 * in UTF-8: where cv1touch looks for the cached sensor and controller configs (src/platform/mod.rs,
 * dirs::openhmd). */
const char *ohmd_get_config_dir(ohmd_context *ctx)
{
	if (ctx->config_dir[0] != 0)
		return ctx->config_dir;
	wchar_t dir[MAX_PATH + 16];
	DWORD n = GetEnvironmentVariableW(L"LOCALAPPDATA", dir, MAX_PATH + 1);
	if (n == 0 || n > MAX_PATH || !absolute(dir))
		n = GetTempPathW(MAX_PATH + 1, dir);
	if (n == 0 || n > MAX_PATH)
		n = 0;
	if (n > 0 && (dir[n - 1] == L'\\' || dir[n - 1] == L'/'))
		n--;
	wcscpy(dir + n, n > 0 ? L"\\openhmd" : L"openhmd");
	if (WideCharToMultiByte(CP_UTF8, 0, dir, -1, ctx->config_dir, OHMD_STR_SIZE, NULL, NULL) == 0)
		strcpy(ctx->config_dir, "openhmd");
	return ctx->config_dir;
}
#endif
