// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty
/* Pure path/error tests; no network shares or Windows APIs are accessed.
 * Windows uses SDK constants. On Linux the fixtures use their documented Win32 values. */
#include <errno.h>
#include <stdio.h>
#include <string.h>
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#else
enum {
	ERROR_FILE_NOT_FOUND = 2, ERROR_PATH_NOT_FOUND = 3, ERROR_TOO_MANY_OPEN_FILES = 4,
	ERROR_ACCESS_DENIED = 5, ERROR_INVALID_HANDLE = 6, ERROR_NOT_ENOUGH_MEMORY = 8,
	ERROR_OUTOFMEMORY = 14, ERROR_INVALID_DRIVE = 15, ERROR_NOT_SAME_DEVICE = 17,
	ERROR_WRITE_PROTECT = 19, ERROR_SHARING_VIOLATION = 32, ERROR_LOCK_VIOLATION = 33,
	ERROR_HANDLE_DISK_FULL = 39, ERROR_BAD_NETPATH = 53, ERROR_BAD_NET_NAME = 67,
	ERROR_FILE_EXISTS = 80, ERROR_INVALID_PARAMETER = 87, ERROR_BUFFER_OVERFLOW = 111,
	ERROR_DISK_FULL = 112, ERROR_INVALID_NAME = 123, ERROR_ALREADY_EXISTS = 183,
	ERROR_FILENAME_EXCED_RANGE = 206, ERROR_DIRECTORY = 267, ERROR_OPERATION_ABORTED = 995
};
#endif
#include "platform-win32-cache.h"
#include "test_util.h"

static void windows_roots_are_not_directory_components(void)
{
	const struct { const char *path, *suffix; } cases[] = {
		{"C:\\cache\\sub", "cache\\sub"}, {"C:/cache/sub", "cache/sub"}, {"C:\\", ""},
		{"relative\\cache", "relative\\cache"}, {"\\cache", "cache"},
		{"\\\\server\\share\\cache\\sub", "cache\\sub"}, {"//server/share/cache", "cache"},
		{"\\\\server\\share", ""}, {"\\\\server\\share\\", ""},
		{"\\\\?\\C:\\cache\\sub", "cache\\sub"}, {"\\\\?\\C:\\", ""},
		{"\\\\?\\UNC\\server\\share\\cache", "cache"},
		{"\\\\?\\UNC\\server\\share", ""}, {"\\\\?\\unc\\server\\share\\cache", "cache"},
		{"\\\\?\\Volume{fictional}\\cache", "cache"},
	};
	for (size_t i = 0; i < sizeof cases / sizeof cases[0]; i++) {
		int start = ohmd_win32_path_start(cases[i].path);
		CHECK(start >= 0 && strcmp(cases[i].path + start, cases[i].suffix) == 0);
	}
	const char *bad[] = {"\\\\server", "\\\\server\\", "\\\\", "\\\\?\\UNC\\server", "\\\\?\\"};
	for (size_t i = 0; i < sizeof bad / sizeof bad[0]; i++)
		CHECK(ohmd_win32_path_start(bad[i]) == -1);
}
static void windows_errors_keep_their_specific_reason(void)
{
	const struct { unsigned long windows; int posix; } cases[] = {
		{ERROR_ACCESS_DENIED,EACCES}, {ERROR_SHARING_VIOLATION,EACCES}, {ERROR_LOCK_VIOLATION,EACCES},
		{ERROR_DISK_FULL,ENOSPC}, {ERROR_HANDLE_DISK_FULL,ENOSPC}, {ERROR_WRITE_PROTECT,EROFS},
		{ERROR_FILE_NOT_FOUND,ENOENT}, {ERROR_PATH_NOT_FOUND,ENOENT}, {ERROR_INVALID_DRIVE,ENOENT},
		{ERROR_BAD_NETPATH,ENOENT}, {ERROR_BAD_NET_NAME,ENOENT}, {ERROR_DIRECTORY,ENOTDIR},
		{ERROR_FILE_EXISTS,EEXIST}, {ERROR_ALREADY_EXISTS,EEXIST}, {ERROR_NOT_SAME_DEVICE,EXDEV},
		{ERROR_NOT_ENOUGH_MEMORY,ENOMEM}, {ERROR_OUTOFMEMORY,ENOMEM},
		{ERROR_INVALID_NAME,EINVAL}, {ERROR_INVALID_PARAMETER,EINVAL},
		{ERROR_FILENAME_EXCED_RANGE,ENAMETOOLONG}, {ERROR_BUFFER_OVERFLOW,ENAMETOOLONG},
		{ERROR_TOO_MANY_OPEN_FILES,EMFILE}, {ERROR_INVALID_HANDLE,EBADF},
		{ERROR_OPERATION_ABORTED,EINTR}, {0,EIO}, {0xfffffffful,EIO},
	};
	for (size_t i = 0; i < sizeof cases / sizeof cases[0]; i++)
		CHECK(ohmd_win32_error(cases[i].windows) == cases[i].posix);
}
static void extended_paths_receive_canonical_separators(void)
{
	wchar_t drive[] = L"\\\\?\\C:/fictional-cache/config.json";
	ohmd_win32_path_separators(drive);
	CHECK(wcscmp(drive, L"\\\\?\\C:\\fictional-cache\\config.json") == 0);
	wchar_t unc[] = L"\\\\?\\UNC\\server/share/config.json";
	ohmd_win32_path_separators(unc);
	CHECK(wcscmp(unc, L"\\\\?\\UNC\\server\\share\\config.json") == 0);
	wchar_t unicode[] = L"C:/fictional-\u00fc/config.json";
	ohmd_win32_path_separators(unicode);
	CHECK(wcscmp(unicode, L"C:\\fictional-\u00fc\\config.json") == 0);
}
int main(int argc, char **argv)
{
	CHECK(argc <= 2);
	if (argc == 1 || strcmp(argv[1], "paths") == 0) windows_roots_are_not_directory_components();
	if (argc == 1 || strcmp(argv[1], "errors") == 0) windows_errors_keep_their_specific_reason();
	if (argc == 1 || strcmp(argv[1], "separators") == 0) extended_paths_receive_canonical_separators();
	CHECK(argc == 1 || strcmp(argv[1], "paths") == 0 || strcmp(argv[1], "errors") == 0 || strcmp(argv[1], "separators") == 0);
	puts("test_win32_cache: ok");
	return 0;
}
