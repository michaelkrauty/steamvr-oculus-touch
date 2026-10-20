// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

#pragma once
#ifndef _WIN32
#include <cassert>
#include <cstdlib>
#include <fcntl.h>
#include <filesystem>
#include <string>
#include <sys/stat.h>
#include <unistd.h>

/* All files remain in a private tree under the selected temp directory. Its open directory
 * gives sockets a short spelling; the owner's PID also works for forked or exec'd clients.
 * The final r is a real directory, so private-directory lstat checks still apply. */
class SocketDir {
public:
	SocketDir()
	{
		real_ = (std::filesystem::temp_directory_path() / "cv1t-s-XXXXXX").string();
		assert(mkdtemp(real_.data()));
		assert(chmod(real_.c_str(), 0700) == 0);
		directory_ = open(real_.c_str(), O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC);
		assert(directory_ >= 0);
		path_ = "/proc/" + std::to_string(getpid()) + "/fd/" + std::to_string(directory_) + "/r";
		assert(mkdir(path_.c_str(), 0700) == 0);
		assert(chmod(path_.c_str(), 0700) == 0);
	}
	~SocketDir()
	{
		std::error_code error;
		std::filesystem::remove_all(real_, error);
		close(directory_);
		assert(!error);
	}
	SocketDir(const SocketDir &) = delete;
	SocketDir &operator=(const SocketDir &) = delete;
	const std::string &path() const { return path_; }

private:
	std::string real_, path_;
	int directory_ = -1;
};
#endif
