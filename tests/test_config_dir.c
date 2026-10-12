// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty
/* The engine caches sensor and controller configuration in the directory cv1touch reads it from:
 * the same cases as src/platform/mod.rs (linux_directories_follow_xdg_and_fall_back_under_home,
 * windows_directories_live_in_local_app_data), resolved by a unity build of the platform layer. */
#ifdef _WIN32
#include "platform-win32.c"
#else
#define _DEFAULT_SOURCE /* setenv, beside the platform layer's own _POSIX_C_SOURCE */
#include "platform-posix.c"
#endif

#include <assert.h>
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

	wchar_t wide[MAX_PATH + 1];
	assert(MultiByteToWideChar(CP_UTF8, 0, file, -1, wide, MAX_PATH + 1) > 0);
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
	lives_in_local_app_data();
	round_trips_a_file_under_a_unicode_path();
#else
	follows_xdg_and_falls_back_under_home();
#endif
	printf("test_config_dir ok\n");
	return 0;
}
