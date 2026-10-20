// Copyright 2013, Fredrik Hultin.
// Copyright 2013, Jakob Bornecrantz.
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */

/* Platform Specific Functions, Unix/Posix Implementation */


#if defined(__unix__) || defined(__unix) || defined(__APPLE__) || defined(__MACH__)

#ifdef __CYGWIN__
#define CLOCK_MONOTONIC (clockid_t)4
#endif

#define _POSIX_C_SOURCE 200809L

#include <time.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <string.h>
#include <errno.h>
#ifdef __linux__
#include <sys/prctl.h>
#endif

#include "platform.h"
#include "openhmdi.h"

// Use clock_gettime if the system implements posix realtime timers
#ifndef CLOCK_MONOTONIC
double ohmd_get_tick()
{
	struct timeval now;
	gettimeofday(&now, NULL);
	return (double)now.tv_sec * 1.0 + (double)now.tv_usec / 1000000.0;
}
#else
double ohmd_get_tick()
{
	struct timespec now;
	clock_gettime(CLOCK_MONOTONIC, &now);
	return (double)now.tv_sec * 1.0 + (double)now.tv_nsec / 1000000000.0;
}
#endif

#ifndef CLOCK_MONOTONIC

static const uint64_t NUM_1_000_000 = 1000000;

void ohmd_monotonic_init(ohmd_context* ctx)
{
	ctx->monotonic_ticks_per_sec = NUM_1_000_000;
}

uint64_t ohmd_monotonic_get(ohmd_context* ctx)
{
	struct timeval now;
	gettimeofday(&now, NULL);
	return now.tv_sec * NUM_1_000_000 + now.tv_usec;
}

#else

static const uint64_t NUM_1_000_000_000 = 1000000000;

void ohmd_monotonic_init(ohmd_context* ctx)
{
		struct timespec ts;
		if (clock_getres(CLOCK_MONOTONIC, &ts) !=  0) {
			ctx->monotonic_ticks_per_sec = NUM_1_000_000_000;
			return;
		}

		ctx->monotonic_ticks_per_sec =
			ts.tv_nsec >= 1000 ?
			NUM_1_000_000_000 :
			NUM_1_000_000_000 / ts.tv_nsec;
}

uint64_t ohmd_monotonic_get(ohmd_context* ctx)
{
	struct timespec now;
	clock_gettime(CLOCK_MONOTONIC, &now);

	return ohmd_monotonic_conv(
		now.tv_sec * NUM_1_000_000_000 + now.tv_nsec,
		NUM_1_000_000_000,
		ctx->monotonic_ticks_per_sec);
}

#endif

OHMD_APIENTRYDLL void OHMD_APIENTRY ohmd_sleep(double seconds)
{
	struct timespec sleepfor;

	sleepfor.tv_sec = (time_t)seconds;
	sleepfor.tv_nsec = (long)((seconds - sleepfor.tv_sec) * 1000000000.0);

	nanosleep(&sleepfor, NULL);
}

// threads
struct ohmd_thread
{
	pthread_t thread;
	unsigned int (*routine)(void* arg);
	void* arg;
};

static void* pthread_wrapper(void* arg)
{
	ohmd_thread* my_thread = (ohmd_thread*)arg;
	my_thread->routine(my_thread->arg);
	return NULL;
}

ohmd_thread* ohmd_create_thread(ohmd_context* ctx, unsigned int (*routine)(void* arg), void* arg)
{
	ohmd_thread* thread = ohmd_alloc(ctx, sizeof(ohmd_thread));
	if(thread == NULL)
		return NULL;

	thread->arg = arg;
	thread->routine = routine;

	int ret = pthread_create(&thread->thread, NULL, pthread_wrapper, thread);

	if(ret != 0){
		free(thread);
		thread = NULL;
	}

	return thread;
}

ohmd_mutex* ohmd_create_mutex(ohmd_context* ctx)
{
	pthread_mutex_t* mutex = ohmd_alloc(ctx, sizeof(pthread_mutex_t));
	if(mutex == NULL)
		return NULL;

	int ret = pthread_mutex_init(mutex, NULL);

	if(ret != 0){
		free(mutex);
		mutex = NULL;
	}

	return (ohmd_mutex*)mutex;
}

void ohmd_destroy_thread(ohmd_thread* thread)
{
	pthread_join(thread->thread, NULL);
	free(thread);
}

void ohmd_destroy_mutex(ohmd_mutex* mutex)
{
	pthread_mutex_destroy((pthread_mutex_t*)mutex);
	free(mutex);
}

void ohmd_lock_mutex(ohmd_mutex* mutex)
{
	if(mutex)
		pthread_mutex_lock((pthread_mutex_t*)mutex);
}

void ohmd_unlock_mutex(ohmd_mutex* mutex)
{
	if(mutex)
		pthread_mutex_unlock((pthread_mutex_t*)mutex);
}

ohmd_cond* ohmd_create_cond(ohmd_context* ctx)
{
	pthread_cond_t* cond = ohmd_alloc(ctx, sizeof(pthread_cond_t));
	if(cond == NULL)
		return NULL;

	int ret = pthread_cond_init(cond, NULL);

	if(ret != 0){
		free(cond);
		cond = NULL;
	}

	return (ohmd_cond*)cond;
}

void ohmd_destroy_cond(ohmd_cond* cond)
{
	pthread_cond_destroy((pthread_cond_t*)cond);
	free(cond);
}

void ohmd_cond_wait(ohmd_cond* cond, ohmd_mutex* mutex)
{
	if (cond && mutex)
		pthread_cond_wait((pthread_cond_t*)cond, (pthread_mutex_t*)mutex);
}

void ohmd_cond_signal(ohmd_cond* cond)
{
	if (cond)
		pthread_cond_signal((pthread_cond_t*)cond);
}

void ohmd_cond_broadcast(ohmd_cond* cond)
{
	if (cond)
		pthread_cond_broadcast((pthread_cond_t*)cond);
}


/// Handling ovr service
void ohmd_set_thread_name(const char* name)
{
#ifdef __linux__
	prctl(PR_SET_NAME, name);
#else
	(void)name;
#endif
}

void ohmd_toggle_ovr_service(int state) //State is 0 for Disable, 1 for Enable
{
	//Empty implementation
}

int findEndPoint(char* path, int endpoint)
{
	char comp[6];
	sprintf(comp,":0%d",endpoint);
	if (strstr(path, comp) != NULL) {
		return 1;
	}
	return 0;
}

int ohmd_read_file(const char* filename, char **out_buf, unsigned long *out_len)
{
	FILE* f = fopen(filename, "rb");
	long file_len, total_read = 0;
	char *buffer = NULL;
	int ret = 0;

	if (f == NULL)
		return errno ? errno : EIO;

	if (fseek(f, 0, SEEK_END) != 0 ||
	    (file_len = ftell(f)) < 0 || fseek(f, 0, SEEK_SET) != 0) {
		ret = errno ? errno : EIO;
		goto out;
	}

	buffer = malloc((size_t)file_len + 1);
	if (buffer == NULL) {
		ret = ENOMEM;
		goto out;
	}

	while (total_read < file_len) {
		errno = 0;
		size_t n_read = fread(buffer + total_read, 1, (size_t)(file_len - total_read), f);
		total_read += (long)n_read;
		if (ferror(f)) {
			ret = errno ? errno : EIO;
			if (ret != EINTR)
				goto out;
			clearerr(f);
		} else if (n_read == 0) {
			ret = EIO;
			goto out;
		}
	}

	buffer[file_len] = 0;
	*out_buf = buffer;
	*out_len = (unsigned long)file_len;
	buffer = NULL;
	ret = 0;
out:
	free(buffer);
	fclose(f);
	return ret;
}

/* Another process starting up can create the directory first. That is fine if it made one. */
static int ohmd_make_directory(const char *path)
{
	struct stat sb;
	if (mkdir(path, 0777) == 0)
		return 0;
	int error = errno ? errno : EIO;
	if (error == EEXIST) {
		if (stat(path, &sb) == 0)
			return S_ISDIR(sb.st_mode) ? 0 : -ENOTDIR;
		error = errno ? errno : EIO;
	}
	return -error;
}

int ohmd_ensure_path(const char* pathname)
{
	int ret;
	struct stat sb;
	char pathbuf[OHMD_STR_SIZE+1];
	char *separator, *cur;
	size_t length = strlen(pathname);

	if (length < 2 || length > OHMD_STR_SIZE)
		return -EINVAL;

	/* Check if the path exists */
	ret = stat(pathname, &sb);
	if (ret < 0 && errno != ENOENT)
		return -(errno ? errno : EIO);
	if (ret == 0)
		return S_ISDIR(sb.st_mode) ? 0 : -ENOTDIR;

	/* Recursively create all components of a path name */
	strcpy(pathbuf, pathname);
	cur = pathbuf + 1;
	while ((separator = strstr(cur, "/")) != NULL) {
		*separator = '\0'; /* Shorten the string to just the path so far */

		ret = stat(pathbuf, &sb);
		if (ret != 0) {
			if (errno != ENOENT)
				return -(errno ? errno : EIO);
			if ((ret = ohmd_make_directory(pathbuf)) != 0)
				return ret;
		} else if (!S_ISDIR(sb.st_mode)) {
			return -ENOTDIR;
		}

		*separator = '/';
		cur = separator + 1;
	}

	return ohmd_make_directory(pathname);
}

/* Publish only a completely written cache: a reader sees the old file or the whole new one.
 * The temporary file is a sibling, so rename cannot cross filesystems or expose a partially
 * written final name. There is no sync: this runs on the engine thread, and waiting for the disk
 * can stall tracking for tens of milliseconds or more. A cache torn by a power cut fails to parse
 * and is read from the controller again. O_EXCL makes the name unique without touching an existing
 * file, and the kernel applies the umask as fopen("w") did. */
int ohmd_write_file(const char* filename, char *buf, unsigned long buf_len)
{
	static unsigned int sequence;
	const size_t size = strlen(filename) + 48;
	char *temporary = malloc(size);
	FILE *f = NULL;
	int fd = -1, ret = 0;
	bool created = false;
	if (!temporary)
		return -ENOMEM;
	for (int attempt = 0;; attempt++) {
		int n = snprintf(temporary, size, "%s.tmp-%ld-%u", filename, (long)getpid(),
			__atomic_add_fetch(&sequence, 1, __ATOMIC_RELAXED));
		if (n < 0 || (size_t)n >= size) {
			ret = -ENAMETOOLONG;
			goto out;
		}
		fd = open(temporary, O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0666);
		if (fd >= 0)
			break;
		if (errno != EEXIST || attempt >= 99) {
			ret = -(errno ? errno : EIO);
			goto out;
		}
	}
	created = true;
	f = fdopen(fd, "wb");
	if (!f) {
		ret = -(errno ? errno : EIO);
		goto out;
	}
	fd = -1; /* the stream now owns it */
	errno = 0;
	if (fwrite(buf, 1, buf_len, f) != buf_len)
		ret = -(errno ? errno : EIO);
	errno = 0;
	if (ret == 0 && fflush(f) != 0)
		ret = -(errno ? errno : EIO);
	errno = 0;
	if (fclose(f) != 0 && ret == 0)
		ret = -(errno ? errno : EIO);
	f = NULL;
	if (ret == 0 && rename(temporary, filename) != 0)
		ret = -(errno ? errno : EIO);
	if (ret == 0)
		created = false;
out:
	if (fd >= 0 && close(fd) != 0 && ret == 0)
		ret = -(errno ? errno : EIO);
	if (created) unlink(temporary);
	free(temporary);
	return ret;
}

/* cv1touch: the XDG rule cv1touch resolves this directory with (src/platform/mod.rs,
 * dirs::openhmd): an empty or relative XDG_CONFIG_HOME is ignored, as the specification says, and
 * without HOME the directory is relative rather than the shared /tmp/openhmd. */
const char *ohmd_get_config_dir(ohmd_context *ctx)
{
  const char *env;

  if (ctx->config_dir[0] != 0)
    return ctx->config_dir;

  env = getenv("XDG_CONFIG_HOME");
  if (env && env[0] == '/') {
    snprintf (ctx->config_dir, OHMD_STR_SIZE, "%s/openhmd", env);
    return ctx->config_dir;
  }
  env = getenv("HOME");
  if (env && env[0] != 0)
    snprintf (ctx->config_dir, OHMD_STR_SIZE, "%s/.config/openhmd", env);
  else
    snprintf (ctx->config_dir, OHMD_STR_SIZE, ".config/openhmd");
  return ctx->config_dir;
}

#endif
