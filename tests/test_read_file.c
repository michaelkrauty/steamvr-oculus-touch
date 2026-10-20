// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty
/* The POSIX file reader unwinds seek, allocation and read failures without
 * publishing a partial buffer. All file operations below are synthetic. */
#define _POSIX_C_SOURCE 199309L
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "test_util.h"

enum failure {
	FAIL_NONE, FAIL_OPEN, FAIL_SEEK_END, FAIL_TELL, FAIL_SEEK_START, FAIL_ALLOC,
	FAIL_READ, FAIL_ZERO_READ, FAIL_EARLY_EOF, FAIL_INTERRUPTED, FAIL_PARTIAL, FAIL_EMPTY
};
static enum failure fail;
static int seeks, closes, reads, clears, owned;
static size_t offset;
static FILE synthetic_stream;
static FILE *stream = &synthetic_stream;

static void *read_malloc(size_t size)
{
	if (fail == FAIL_ALLOC)
		return NULL;
	void *p = malloc(size);
	CHECK(p != NULL);
	owned++;
	return p;
}
static void read_free(void *p)
{
	if (p) {
		CHECK(owned == 1);
		owned--;
		free(p);
	}
}
static FILE *read_open(const char *path, const char *mode)
{
	CHECK(strcmp(path, "synthetic.json") == 0 && strcmp(mode, "rb") == 0);
	if (fail == FAIL_OPEN) { errno = ENOENT; return NULL; }
	return stream;
}
static int read_seek(FILE *f, long pos, int whence)
{
	CHECK(f == stream && pos == 0);
	seeks++;
	CHECK(whence == (seeks == 1 ? SEEK_END : SEEK_SET));
	if ((fail == FAIL_SEEK_END && seeks == 1) || (fail == FAIL_SEEK_START && seeks == 2)) {
		errno = ESPIPE;
		return -1;
	}
	return 0;
}
static long read_tell(FILE *f)
{
	CHECK(f == stream);
	if (fail == FAIL_TELL) { errno = EIO; return -1; }
	return fail == FAIL_EMPTY ? 0 : 3;
}
static size_t read_bytes(void *buf, size_t size, size_t count, FILE *f)
{
	CHECK(f == stream && size > 0 && count > 0);
	reads++;
	if (fail == FAIL_READ) { errno = EIO; return 0; }
	if (fail == FAIL_ZERO_READ || fail == FAIL_EARLY_EOF) { errno = 0; return 0; }
	if (fail == FAIL_INTERRUPTED && reads == 1) { errno = EINTR; return 0; }
	if (fail == FAIL_INTERRUPTED) CHECK(clears == 1);
	size_t bytes = size * count;
	if (bytes > 3 - offset) bytes = 3 - offset;
	if (fail == FAIL_PARTIAL && bytes > 1) bytes = 1;
	memcpy(buf, "abc" + offset, bytes);
	offset += bytes;
	return bytes / size;
}
static inline int read_error(FILE *f)
{
	CHECK(f == stream);
	return fail == FAIL_READ || (fail == FAIL_INTERRUPTED && reads == 1 && clears == 0);
}
static inline int read_eof(FILE *f) { CHECK(f == stream); return fail == FAIL_EARLY_EOF; }
static inline void read_clear(FILE *f) { CHECK(f == stream); clears++; }
static int read_close(FILE *f) { CHECK(f == stream); closes++; return 0; }

#define malloc read_malloc
#define free read_free
#define fopen read_open
#define fseek read_seek
#define ftell read_tell
#define fread read_bytes
#define feof read_eof
#define ferror read_error
#define clearerr read_clear
#define fclose read_close
#undef _POSIX_C_SOURCE
#include "platform-posix.c"
#undef malloc
#undef free
#undef fopen
#undef fseek
#undef ftell
#undef fread
#undef feof
#undef ferror
#undef clearerr
#undef fclose

static void reset(enum failure failure)
{
	CHECK(owned == 0);
	fail = failure;
	seeks = closes = reads = clears = 0;
	offset = 0;
	errno = 0;
}
static void fails_without_publishing(enum failure failure, int expected)
{
	reset(failure);
	char sentinel;
	char *out = &sentinel;
	unsigned long length = 123;
	CHECK(ohmd_read_file("synthetic.json", &out, &length) == expected);
	CHECK(out == &sentinel && length == 123);
	CHECK(owned == 0);
	CHECK(closes == (failure == FAIL_OPEN ? 0 : 1));
}
int main(void)
{
	fails_without_publishing(FAIL_READ, EIO);
	fails_without_publishing(FAIL_OPEN, ENOENT);
	fails_without_publishing(FAIL_SEEK_END, ESPIPE);
	fails_without_publishing(FAIL_TELL, EIO);
	fails_without_publishing(FAIL_SEEK_START, ESPIPE);
	fails_without_publishing(FAIL_ALLOC, ENOMEM);
	fails_without_publishing(FAIL_ZERO_READ, EIO);
	fails_without_publishing(FAIL_EARLY_EOF, EIO);
	for (int k = 0; k < 4; k++) {
		const enum failure cases[] = {FAIL_NONE, FAIL_INTERRUPTED, FAIL_PARTIAL, FAIL_EMPTY};
		reset(cases[k]);
		char *out = NULL;
		unsigned long length = 123;
		CHECK(ohmd_read_file("synthetic.json", &out, &length) == 0);
		CHECK(out != NULL && length == (fail == FAIL_EMPTY ? 0 : 3));
		CHECK(strcmp(out, fail == FAIL_EMPTY ? "" : "abc") == 0);
		CHECK(owned == 1 && closes == 1);
		read_free(out);
		CHECK(owned == 0);
	}
	puts("test_read_file: ok");
	return 0;
}
