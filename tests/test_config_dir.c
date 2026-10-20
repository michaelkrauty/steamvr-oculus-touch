// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty
/* The engine caches sensor and controller configuration in the directory cv1touch reads it from:
 * the same cases as src/platform/mod.rs (linux_directories_follow_xdg_and_fall_back_under_home,
 * windows_directories_live_in_local_app_data), resolved by a unity build of the platform layer. */
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define WIN32_EXTRA_LEAN
#include <windows.h>
/* Everything platform-win32.c includes is included first, so that only its own calls see the shims. */
#include <errno.h>
#include <malloc.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <wchar.h>
#include <wctype.h>
static DWORD directory_failure, attribute_failure;
static int attribute_calls, create_calls, existing_is_file, fail_allocation;
static DWORD WINAPI test_attributes(LPCWSTR path);
static BOOL WINAPI test_create_directory(LPCWSTR path, LPSECURITY_ATTRIBUTES attributes);
static void *test_malloc(size_t size);
#define GetFileAttributesW test_attributes
#define CreateDirectoryW test_create_directory
#define malloc test_malloc
#include "platform-win32.c"
#undef GetFileAttributesW
#undef CreateDirectoryW
#undef malloc
/* A failed allocation leaves errno alone, as the C library is allowed to. */
static void *test_malloc(size_t size)
{
	if (fail_allocation) { fail_allocation = 0; errno = 0; return NULL; }
	return malloc(size);
}
static DWORD WINAPI test_attributes(LPCWSTR path)
{
	if (attribute_failure) { SetLastError(attribute_failure); return INVALID_FILE_ATTRIBUTES; }
	if (directory_failure) {
		attribute_calls++;
		if (directory_failure == ERROR_ALREADY_EXISTS && attribute_calls % 2 == 0)
			return existing_is_file ? FILE_ATTRIBUTE_ARCHIVE : FILE_ATTRIBUTE_DIRECTORY;
		SetLastError(ERROR_FILE_NOT_FOUND);
		return INVALID_FILE_ATTRIBUTES;
	}
	return GetFileAttributesW(path);
}
static BOOL WINAPI test_create_directory(LPCWSTR path, LPSECURITY_ATTRIBUTES attributes)
{
	if (directory_failure) { create_calls++; SetLastError(directory_failure); return FALSE; }
	return CreateDirectoryW(path, attributes);
}
#else
#define _DEFAULT_SOURCE /* setenv, beside the platform layer's own _POSIX_C_SOURCE */
#include "platform-posix.c"
#endif

#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* the rest of the engine, which the platform layer's thread and clock helpers call into */
void *ohmd_allocfn(ohmd_context *ctx, const char *e_msg, size_t size)
{
	(void)ctx;
	(void)e_msg;
	return calloc(1, size);
}

uint64_t ohmd_monotonic_conv(uint64_t ticks, uint64_t srcTicksPerSecond, uint64_t dstTicksPerSecond)
{
	return ticks / srcTicksPerSecond * dstTicksPerSecond + ticks % srcTicksPerSecond * dstTicksPerSecond / srcTicksPerSecond;
}

static const char *config_dir(void)
{
	static ohmd_context ctx;
	memset(&ctx, 0, sizeof ctx);
	return ohmd_get_config_dir(&ctx);
}

#ifdef _WIN32
static void set(const wchar_t *name, const wchar_t *value)
{
	assert(SetEnvironmentVariableW(name, value));
}

static void lives_in_local_app_data(void)
{
	set(L"LOCALAPPDATA", L"C:\\Users\\a b\\AppData\\Local");
	assert(strcmp(config_dir(), "C:\\Users\\a b\\AppData\\Local\\openhmd") == 0);
	set(L"LOCALAPPDATA", L"C:\\Users\\a b\\AppData\\Local\\");
	assert(strcmp(config_dir(), "C:\\Users\\a b\\AppData\\Local\\openhmd") == 0);
	/* an account name outside the ANSI code page, in UTF-8 */
	set(L"LOCALAPPDATA", L"C:\\Users\\J\u00fcrgen \u5f20\\AppData\\Local");
	assert(strcmp(config_dir(), "C:\\Users\\J\xc3\xbcrgen \xe5\xbc\xa0\\AppData\\Local\\openhmd") == 0);

	/* unset or relative: the temp directory, as Rust's std::env::temp_dir() */
	wchar_t temp[MAX_PATH + 1];
	DWORD n = GetTempPathW(MAX_PATH + 1, temp);
	assert(n > 1 && n <= MAX_PATH && temp[n - 1] == L'\\');
	wcscpy(temp + n, L"openhmd");
	char expected[OHMD_STR_SIZE];
	assert(WideCharToMultiByte(CP_UTF8, 0, temp, -1, expected, sizeof expected, NULL, NULL) > 0);
	set(L"LOCALAPPDATA", NULL);
	assert(strcmp(config_dir(), expected) == 0);
	set(L"LOCALAPPDATA", L"AppData\\Local");
	assert(strcmp(config_dir(), expected) == 0);
}

/* Synthetic Win32 failures reach the directory caller without touching these paths. */
static void directory_failures_keep_their_error_code(void)
{
	const struct { DWORD code; int error; } cases[] = {
		{ERROR_ACCESS_DENIED,EACCES}, {ERROR_DISK_FULL,ENOSPC},
		{ERROR_PATH_NOT_FOUND,ENOENT}, {ERROR_WRITE_PROTECT,EROFS},
	};
	for (size_t i = 0; i < sizeof cases / sizeof cases[0]; i++) {
		directory_failure = cases[i].code; attribute_calls = create_calls = 0;
		assert(ohmd_ensure_path("C:\\fictional-cache\\sub") == -cases[i].error);
		assert(create_calls == 1);
	}
	directory_failure = ERROR_ALREADY_EXISTS; attribute_calls = create_calls = 0;
	existing_is_file = 1;
	assert(ohmd_ensure_path("C:\\fictional-cache\\sub") == -ENOTDIR && create_calls == 1);
	attribute_calls = create_calls = existing_is_file = 0;
	assert(ohmd_ensure_path("C:\\fictional-cache\\sub") == 0 && create_calls == 2);
	directory_failure = 0; create_calls = 0; attribute_failure = ERROR_ACCESS_DENIED;
	assert(ohmd_ensure_path("C:\\fictional-cache\\sub") == -EACCES && create_calls == 0);
	attribute_failure = 0;
}

/* A path that cannot be converted fails with the reason, for the reader as for the writer. */
static void path_conversion_failures_keep_their_reason(void)
{
	char *text = NULL;
	unsigned long length = 0;
	char data[] = "{}";
	fail_allocation = 1;
	assert(ohmd_read_file("C:\\fictional-cache\\config.json", &text, &length) == ENOMEM && text == NULL);
	fail_allocation = 1;
	assert(ohmd_write_file("C:\\fictional-cache\\config.json", data, 2) == -ENOMEM);
	fail_allocation = 1;
	assert(ohmd_ensure_path("C:\\fictional-cache\\sub") == -ENOMEM);
	assert(ohmd_read_file("C:\\fictional-cache\\\xff.json", &text, &length) == EINVAL);
	assert(ohmd_write_file("C:\\fictional-cache\\\xff.json", data, 2) == -EINVAL);
	assert(fail_allocation == 0);
}

/* The config cache writes and reads back under a directory named outside the ANSI code page. */
static void round_trips_a_file_under_a_unicode_path(void)
{
	wchar_t temp[MAX_PATH + 1];
	DWORD n = GetTempPathW(MAX_PATH + 1, temp);
	assert(n > 0 && n <= MAX_PATH);
	char dir[OHMD_STR_SIZE];
	assert(WideCharToMultiByte(CP_UTF8, 0, temp, -1, dir, sizeof dir, NULL, NULL) > 0);
	char config[OHMD_STR_SIZE], file[OHMD_STR_SIZE];
	snprintf(config, sizeof config, "%scv1t-config-%lu-J\xc3\xbcrgen \xe5\xbc\xa0\\openhmd", dir, GetCurrentProcessId());
	snprintf(file, sizeof file, "%s\\cache.json", config);
	assert(ohmd_ensure_path(config) == 0);
	char text[] = "{\"leds\": []}";
	assert(ohmd_write_file(file, text, sizeof text - 1) == 0);
	char *read = NULL;
	unsigned long len = 0;
	assert(ohmd_read_file(file, &read, &len) == 0);
	assert(len == sizeof text - 1 && memcmp(read, text, len) == 0);
	free(read);

	/* The cache API joins names with '/', even under an extended namespace. */
	wchar_t *normal = widen(file); assert(normal);
	wchar_t extended[OHMD_STR_SIZE + 16];
	if (wcsncmp(normal, L"\\\\?\\", 4) == 0) wcscpy(extended, normal);
	else assert(swprintf(extended, sizeof extended / sizeof *extended,
		normal[0] == L'\\' && normal[1] == L'\\' ? L"\\\\?\\UNC\\%ls" : L"\\\\?\\%ls",
		normal[0] == L'\\' && normal[1] == L'\\' ? normal + 2 : normal) > 0);
	free(normal);
	char extended_file[OHMD_STR_SIZE + 32], extended_dir[OHMD_STR_SIZE + 32];
	assert(WideCharToMultiByte(CP_UTF8, 0, extended, -1, extended_file, sizeof extended_file, NULL, NULL) > 0);
	char *join = strrchr(extended_file, '\\'); assert(join); *join = '/';
	strcpy(extended_dir, extended_file); extended_dir[join - extended_file] = 0;
	assert(ohmd_ensure_path(extended_dir) == 0);
	assert(ohmd_write_file(extended_file, text, sizeof text - 1) == 0);
	read = NULL; len = 0;
	assert(ohmd_read_file(extended_file, &read, &len) == 0 && len == sizeof text - 1);
	assert(memcmp(read, text, len) == 0); free(read);

	/* An existing file cannot be a cache directory or contain another cache file. */
	assert(ohmd_ensure_path(file) == -ENOTDIR);
	char child[OHMD_STR_SIZE];
	snprintf(child, sizeof child, "%s\\child.json", file);
	assert(ohmd_write_file(child, text, sizeof text - 1) < 0);

	wchar_t wide[MAX_PATH + 1];
	assert(MultiByteToWideChar(CP_UTF8, 0, file, -1, wide, MAX_PATH + 1) > 0);
	/* Replacing a read-only final file fails without truncating its valid contents. */
	DWORD attributes = GetFileAttributesW(wide);
	assert(attributes != INVALID_FILE_ATTRIBUTES && SetFileAttributesW(wide, attributes | FILE_ATTRIBUTE_READONLY));
	char replacement[] = "{\"new\": true}";
	assert(ohmd_write_file(file, replacement, sizeof replacement - 1) == -EACCES);
	read = NULL; len = 0;
	assert(ohmd_read_file(file, &read, &len) == 0 && len == sizeof text - 1);
	assert(memcmp(read, text, len) == 0); free(read);
	assert(SetFileAttributesW(wide, attributes));
	wchar_t pattern[OHMD_STR_SIZE + 64];
	assert(swprintf(pattern, sizeof pattern / sizeof *pattern, L"%ls.tmp-*", wide) > 0);
	WIN32_FIND_DATAW found;
	HANDLE search = FindFirstFileW(pattern, &found);
	if (search != INVALID_HANDLE_VALUE) FindClose(search);
	assert(search == INVALID_HANDLE_VALUE && GetLastError() == ERROR_FILE_NOT_FOUND);
	assert(DeleteFileW(wide));
	for (int up = 0; up < 2; up++) {
		*wcsrchr(wide, L'\\') = 0;
		assert(RemoveDirectoryW(wide));
	}
}
#else
static void set(const char *name, const char *value)
{
	assert((value ? setenv(name, value, 1) : unsetenv(name)) == 0);
}

static void follows_xdg_and_falls_back_under_home(void)
{
	set("XDG_CONFIG_HOME", NULL);
	set("HOME", "/home/u");
	assert(strcmp(config_dir(), "/home/u/.config/openhmd") == 0);
	set("XDG_CONFIG_HOME", "/c");
	assert(strcmp(config_dir(), "/c/openhmd") == 0);
	/* the specification: an empty or relative value counts as unset */
	set("XDG_CONFIG_HOME", "");
	assert(strcmp(config_dir(), "/home/u/.config/openhmd") == 0);
	set("XDG_CONFIG_HOME", "relative");
	assert(strcmp(config_dir(), "/home/u/.config/openhmd") == 0);
	/* no home: relative, as cv1touch's own directories, never the shared /tmp */
	set("XDG_CONFIG_HOME", NULL);
	set("HOME", NULL);
	assert(strcmp(config_dir(), ".config/openhmd") == 0);
}
#endif

int main(void)
{
#ifdef _WIN32
	directory_failures_keep_their_error_code();
	path_conversion_failures_keep_their_reason();
	lives_in_local_app_data();
	round_trips_a_file_under_a_unicode_path();
#else
	follows_xdg_and_falls_back_under_home();
#endif
	printf("test_config_dir ok\n");
	return 0;
}
