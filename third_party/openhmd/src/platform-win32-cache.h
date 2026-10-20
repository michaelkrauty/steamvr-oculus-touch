// Copyright 2026 michaelkrauty.
// SPDX-License-Identifier: BSL-1.0
/* Pure Windows cache path/error helpers; the caller supplies Win32 error constants. */
#ifndef OHMD_PLATFORM_WIN32_CACHE_H
#define OHMD_PLATFORM_WIN32_CACHE_H
#include <errno.h>
#include <string.h>
#include <wchar.h>

static int ohmd_win32_separator(char c)
{
	return c == '\\' || c == '/';
}

/* First directory component after a drive, UNC share or extended namespace root.
 * A bare share is never passed to GetFileAttributesW, which rejects that form. */
static int ohmd_win32_path_start(const char *path)
{
	size_t length = strlen(path), i = 0;
	int unc = 0;
	if (length >= 4 && ohmd_win32_separator(path[0]) && ohmd_win32_separator(path[1]) &&
	    path[2] == '?' && ohmd_win32_separator(path[3])) {
		i = 4;
		if (length >= 8 && (path[4] == 'U' || path[4] == 'u') &&
		    (path[5] == 'N' || path[5] == 'n') && (path[6] == 'C' || path[6] == 'c') && ohmd_win32_separator(path[7])) {
			unc = 1; i = 8;
		} else if (length >= 6 && path[5] == ':') {
			i = 6;
		} else {
			while (path[i] && !ohmd_win32_separator(path[i])) i++;
			if (i == 4) return -1;
		}
	} else if (length >= 2 && ohmd_win32_separator(path[0]) && ohmd_win32_separator(path[1])) {
		unc = 1; i = 2;
	} else if (length >= 2 && path[1] == ':') {
		i = 2;
	} else if (ohmd_win32_separator(path[0])) {
		i = 1;
	}
	if (unc) {
		size_t start = i;
		while (path[i] && !ohmd_win32_separator(path[i])) i++;
		if (i == start || !path[i]) return -1;
		while (ohmd_win32_separator(path[i])) i++;
		start = i;
		while (path[i] && !ohmd_win32_separator(path[i])) i++;
		if (i == start) return -1;
	}
	while (ohmd_win32_separator(path[i])) i++;
	return (int)i;
}

/* The shared cache joins use '/'; extended Win32 namespaces require '\\'. */
static void ohmd_win32_path_separators(wchar_t *path)
{
	for (; *path; path++) {
		if (*path == L'/') *path = L'\\';
	}
}

static int ohmd_win32_error(unsigned long error)
{
	switch (error) {
	case ERROR_ACCESS_DENIED: case ERROR_SHARING_VIOLATION: case ERROR_LOCK_VIOLATION: return EACCES;
	case ERROR_FILE_NOT_FOUND: case ERROR_PATH_NOT_FOUND: case ERROR_INVALID_DRIVE:
	case ERROR_BAD_NETPATH: case ERROR_BAD_NET_NAME: return ENOENT;
	case ERROR_DISK_FULL: case ERROR_HANDLE_DISK_FULL: return ENOSPC;
	case ERROR_WRITE_PROTECT: return EROFS;
	case ERROR_DIRECTORY: return ENOTDIR;
	case ERROR_FILE_EXISTS: case ERROR_ALREADY_EXISTS: return EEXIST;
	case ERROR_NOT_SAME_DEVICE: return EXDEV;
	case ERROR_NOT_ENOUGH_MEMORY: case ERROR_OUTOFMEMORY: return ENOMEM;
	case ERROR_INVALID_NAME: case ERROR_INVALID_PARAMETER: return EINVAL;
	case ERROR_FILENAME_EXCED_RANGE: case ERROR_BUFFER_OVERFLOW: return ENAMETOOLONG;
	case ERROR_TOO_MANY_OPEN_FILES: return EMFILE;
	case ERROR_INVALID_HANDLE: return EBADF;
	case ERROR_OPERATION_ABORTED: return EINTR;
	default: return EIO;
	}
}
#endif
