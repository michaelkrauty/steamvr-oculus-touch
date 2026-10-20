// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

#include "haptics.h"

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#ifndef _WIN32
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/un.h>
#include <unistd.h>
#endif

#include "cv1touch_shm.h"
#ifdef _WIN32
#include <aclapi.h>
#endif
#include "log.h"

namespace cv1t {

HapticCommand clamp_haptic(int hand, float d, float f, float a)
{
	HapticCommand c{hand, d, f, a};
	if (!std::isfinite(c.duration_s) || c.duration_s <= 0.0f)
		c.duration_s = 0.01f;
	if (c.duration_s < 0.001f)
		c.duration_s = 0.001f; // the command prints 4 decimals: shorter would read as 0
	if (c.duration_s > 10.0f)
		c.duration_s = 10.0f;
	if (!std::isfinite(c.frequency_hz) || c.frequency_hz <= 0.0f)
		c.frequency_hz = 320.0f;
	if (c.frequency_hz < 1.0f)
		c.frequency_hz = 1.0f; // printed with 1 decimal
	if (c.frequency_hz > 1000.0f)
		c.frequency_hz = 1000.0f;
	if (!std::isfinite(c.amplitude) || c.amplitude < 0.0f)
		c.amplitude = 0.0f;
	if (c.amplitude > 1.0f)
		c.amplitude = 1.0f;
	return c;
}

std::string haptic_line(const HapticCommand &c)
{
	char buf[96];
	snprintf(buf, sizeof buf, "haptic %s %.4f %.1f %.3f\n", c.hand == CV1T_HAND_LEFT ? "left" : "right", c.duration_s, c.frequency_hz, c.amplitude);
	return buf;
}

#ifdef _WIN32
static std::wstring widen(const std::string &s)
{
	const int n = MultiByteToWideChar(CP_UTF8, 0, s.data(), static_cast<int>(s.size()), nullptr, 0);
	std::wstring w(static_cast<size_t>(n), L'\0');
	MultiByteToWideChar(CP_UTF8, 0, s.data(), static_cast<int>(s.size()), w.data(), n);
	return w;
}

static std::string narrow(const std::wstring &w)
{
	const int n = WideCharToMultiByte(CP_UTF8, 0, w.data(), static_cast<int>(w.size()), nullptr, 0, nullptr, nullptr);
	std::string s(static_cast<size_t>(n), '\0');
	WideCharToMultiByte(CP_UTF8, 0, w.data(), static_cast<int>(w.size()), s.data(), n, nullptr, nullptr);
	return s;
}

/* cv1touchd's runtime directory: %LOCALAPPDATA%\cv1touch, else the temp directory's cv1touch. */
std::string HapticSender::default_socket_path()
{
	wchar_t buf[MAX_PATH + 1];
	DWORD n = GetEnvironmentVariableW(L"LOCALAPPDATA", buf, MAX_PATH + 1);
	if (n == 0 || n > MAX_PATH)
		n = GetTempPathW(MAX_PATH + 1, buf);
	std::wstring dir(buf, n <= MAX_PATH ? n : 0);
	if (!dir.empty() && dir.back() != L'\\' && dir.back() != L'/')
		dir += L'\\';
	return narrow(dir + L"cv1touch\\control.sock");
}

std::wstring control_pipe_name(const std::string &path)
{
	std::wstring name = L"\\\\.\\pipe\\cv1touch-";
	for (wchar_t c : widen(path))
		name += c == L'\\' ? L'/' : c;
	return name.size() <= 256 ? name : std::wstring();
}
#else
std::string HapticSender::default_socket_path()
{
	const char *rt = std::getenv("XDG_RUNTIME_DIR");
	std::string dir = rt && *rt ? rt : "/run/user/" + std::to_string(getuid());
	return dir + "/cv1touch/control.sock";
}
#endif

HapticSender::HapticSender() = default;

HapticSender::HapticSender(std::string socket_path, std::string fallback) : control_(std::move(socket_path), std::move(fallback)) {}

ControlClient::ControlClient() : ControlClient(HapticSender::default_socket_path())
{
#ifndef _WIN32
	paths_[1] = container_socket_path(geteuid());
#endif
}

ControlClient::ControlClient(std::string path, std::string fallback) : paths_{std::move(path), std::move(fallback)} {}

#ifndef _WIN32
std::string container_socket_path(uint32_t uid)
{
	const char *base = "/dev/shm";
#ifdef CV1T_DRIVER_TEST
	// Default-constructor regressions use their own sockets, never the installed daemon.
	if (const char *fixture = std::getenv("CV1T_TEST_CONTROL_SHM_DIR"); fixture && *fixture)
		base = fixture;
#endif
	return std::string(base) + "/cv1touch-control-" + std::to_string(uid) + "/control.sock";
}

bool control_directory_is_private(const std::string &dir, uint32_t uid)
{
	struct stat st {};
	if (lstat(dir.c_str(), &st) != 0)
		return false;
	if (!S_ISDIR(st.st_mode) || st.st_uid != uid || (st.st_mode & 07777) != 0700) {
		errno = EACCES;
		return false;
	}
	return true;
}
#endif

HapticSender::~HapticSender() { stop(); }

void HapticSender::start()
{
	std::lock_guard<std::mutex> g(mu_);
	if (running_)
		return;
	running_ = true;
	worker_ = std::thread([this] {
		try {
			run();
		} catch (...) {
			cv1t_log("cv1touch: haptic worker stopped by an exception");
		}
	});
}

void HapticSender::stop()
{
	{
		std::lock_guard<std::mutex> g(mu_);
		running_ = false;
	}
	cv_.notify_all();
	if (worker_.joinable())
		worker_.join();
}

void HapticSender::submit(int hand, float d, float f, float a)
{
	if (hand < 0 || hand > 1)
		return;
	{
		std::lock_guard<std::mutex> g(mu_);
		cmd_[hand] = clamp_haptic(hand, d, f, a);
		pending_[hand] = true;
	}
	cv_.notify_one();
}

bool HapticSender::take(int hand, HapticCommand *out)
{
	std::lock_guard<std::mutex> g(mu_);
	if (!pending_[hand])
		return false;
	pending_[hand] = false;
	*out = cmd_[hand];
	return true;
}

#ifdef _WIN32
/* One overlapped read or write on the pipe, given up after ms. */
static bool pipe_io(HANDLE pipe, HANDLE event, bool write, void *buf, DWORD len, DWORD *moved, DWORD ms)
{
	OVERLAPPED ov{};
	ov.hEvent = event;
	ResetEvent(event);
	const BOOL done = write ? WriteFile(pipe, buf, len, nullptr, &ov) : ReadFile(pipe, buf, len, nullptr, &ov);
	if (!done && GetLastError() != ERROR_IO_PENDING)
		return false;
	if (WaitForSingleObject(event, ms) != WAIT_OBJECT_0) {
		CancelIoEx(pipe, &ov);
		GetOverlappedResult(pipe, &ov, moved, TRUE); // the buffer is in use until the cancel lands
		return false;
	}
	return GetOverlappedResult(pipe, &ov, moved, FALSE) != 0;
}

/* Pipe names are global, so another account could create the daemon's pipe first. The daemon
 * makes its user the owner of its pipe (src/platform/windows.rs, served_by): a pipe owned by
 * anyone else is not the daemon's. */
static bool owned_by_this_user(HANDLE pipe)
{
	HANDLE token = nullptr;
	if (!OpenProcessToken(GetCurrentProcess(), TOKEN_QUERY, &token))
		return false;
	alignas(TOKEN_USER) unsigned char user[sizeof(TOKEN_USER) + SECURITY_MAX_SID_SIZE];
	DWORD size = 0;
	const bool have_user = GetTokenInformation(token, TokenUser, user, sizeof user, &size) != 0;
	CloseHandle(token);
	PSID owner = nullptr;
	PSECURITY_DESCRIPTOR descriptor = nullptr;
	if (!have_user || GetSecurityInfo(pipe, SE_KERNEL_OBJECT, OWNER_SECURITY_INFORMATION, &owner, nullptr, nullptr, nullptr,
					  &descriptor) != ERROR_SUCCESS)
		return false;
	const bool ours = EqualSid(owner, reinterpret_cast<TOKEN_USER *>(user)->User.Sid) != 0;
	LocalFree(descriptor);
	return ours;
}

/* One request, one reply line; 100 ms timeouts so a stuck daemon cannot pile commands up. */
bool send_control_line(const std::string &path, const std::string &line)
{
	const std::wstring name = control_pipe_name(path);
	if (name.empty())
		return false;
	auto open = [&] {
		// identification only: the server may check who we are, not act as us
		return CreateFileW(name.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING,
				   FILE_FLAG_OVERLAPPED | SECURITY_SQOS_PRESENT | SECURITY_IDENTIFICATION, nullptr);
	};
	HANDLE pipe = open();
	// every instance busy for a moment: the daemon offers the next one as it accepts
	if (pipe == INVALID_HANDLE_VALUE && GetLastError() == ERROR_PIPE_BUSY && WaitNamedPipeW(name.c_str(), 100))
		pipe = open();
	if (pipe == INVALID_HANDLE_VALUE)
		return false;
	if (!owned_by_this_user(pipe)) {
		CloseHandle(pipe);
		return false;
	}
	HANDLE event = CreateEventW(nullptr, TRUE, FALSE, nullptr);
	bool ok = event != nullptr;
	DWORD moved = 0;
	ok = ok && pipe_io(pipe, event, true, const_cast<char *>(line.data()), static_cast<DWORD>(line.size()), &moved, 100) &&
	     moved == line.size();
	if (ok) {
		char reply[256];
		ok = pipe_io(pipe, event, false, reply, sizeof reply - 1, &moved, 100) && moved >= 2 && std::strncmp(reply, "ok", 2) == 0;
	}
	if (event)
		CloseHandle(event);
	CloseHandle(pipe);
	return ok;
}
#else
/* Only an error from connect may select another endpoint. Once connected, a failed write or
 * reply does not say whether the command arrived, so it must never be sent a second time. */
static int connect_control_socket(const std::string &path, int *connect_error)
{
	*connect_error = 0;
	sockaddr_un addr{};
	if (path.size() >= sizeof addr.sun_path)
		return -1;
	const int fd = socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
	if (fd < 0)
		return -1;
	timeval tv{0, 100'000};
	setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof tv);
	setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
	addr.sun_family = AF_UNIX;
	std::strcpy(addr.sun_path, path.c_str());
	if (connect(fd, reinterpret_cast<sockaddr *>(&addr), sizeof addr) != 0) {
		*connect_error = errno;
		close(fd);
		return -1;
	}
	return fd;
}

/* One request, one reply line; 100 ms timeouts so a stuck daemon cannot pile commands up. */
static bool exchange_control_line(int fd, const std::string &line)
{
	bool ok = send(fd, line.data(), line.size(), MSG_NOSIGNAL) == static_cast<ssize_t>(line.size());
	if (ok) {
		char reply[256];
		const ssize_t n = read(fd, reply, sizeof reply - 1);
		ok = n >= 2 && std::strncmp(reply, "ok", 2) == 0;
	}
	close(fd);
	return ok;
}

bool send_control_line(const std::string &path, const std::string &line)
{
	int error = 0;
	const int fd = connect_control_socket(path, &error);
	return fd >= 0 && exchange_control_line(fd, line);
}
#endif

bool ControlClient::send(const std::string &line)
{
#ifdef _WIN32
	return send_control_line(paths_[0], line);
#else
	for (unsigned attempt = 0; attempt < 2; attempt++) {
		const unsigned which = (last_ + attempt) % 2;
		const auto &path = paths_[which];
		if (path.empty())
			return false;
		int error = 0, fd = -1;
		if (which == 1 && !control_directory_is_private(path.substr(0, path.rfind('/')), geteuid())) {
			if (errno != ENOENT)
				return false;
			error = ENOENT; // a vanished cached directory can fail back; never connect without validation
		} else {
			fd = connect_control_socket(path, &error);
		}
		if (fd >= 0) {
			last_ = which;
			return exchange_control_line(fd, line);
		}
		if (error != ENOENT && error != ECONNREFUSED)
			return false;
	}
	return false;
#endif
}

void HapticSender::run()
{
	for (;;) {
		{
			std::unique_lock<std::mutex> lk(mu_);
			cv_.wait(lk, [this] { return !running_ || pending_[0] || pending_[1]; });
			if (!running_)
				return;
		}
		for (int h = 0; h < 2; h++) {
			HapticCommand c{};
			if (!take(h, &c))
				continue;
			if (!control_.send(haptic_line(c))) {
				const uint64_t now = cv1t_now_ns();
				if (now - last_warn_ns_ > 60'000'000'000ull) {
					cv1t_log("cv1touch: haptic command not delivered to %s (daemon not running?)", control_.path().c_str());
					last_warn_ns_ = now;
				}
			}
		}
	}
}

} // namespace cv1t
