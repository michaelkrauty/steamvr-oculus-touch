// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <limits>
#include <string>
#ifndef _WIN32
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#endif
#ifdef _WIN32
#include <windows.h>
#include <sddl.h>
#endif

#include "cv1touch_shm.h"
#include "haptics.h"
#include "socket_dir.h"

using namespace cv1t;

/* Review Focus 5: what apps send, into what the daemon accepts. */
static void clamps_what_apps_send()
{
	const float nan = std::numeric_limits<float>::quiet_NaN();
	auto c = clamp_haptic(1, 0.0f, 0.0f, 0.5f);
	assert(c.duration_s == 0.01f && c.frequency_hz == 320.0f && c.amplitude == 0.5f);
	c = clamp_haptic(0, 30.0f, 5000.0f, 1.7f);
	assert(c.duration_s == 10.0f && c.frequency_hz == 1000.0f && c.amplitude == 1.0f);
	c = clamp_haptic(0, nan, nan, nan);
	assert(c.duration_s == 0.01f && c.frequency_hz == 320.0f && c.amplitude == 0.0f);
	c = clamp_haptic(0, 1e-5f, 0.04f, 0.5f); // review fix 4: too small to print would read as 0
	assert(c.duration_s == 0.001f && c.frequency_hz == 1.0f);
	assert(haptic_line(c) == "haptic left 0.0010 1.0 0.500\n");
	c = clamp_haptic(0, 0.2f, 160.0f, -0.3f);
	assert(c.duration_s == 0.2f && c.frequency_hz == 160.0f && c.amplitude == 0.0f);
}

static void formats_the_daemon_command()
{
	assert(haptic_line({CV1T_HAND_RIGHT, 0.25f, 160.0f, 1.0f}) == "haptic right 0.2500 160.0 1.000\n");
	assert(haptic_line({CV1T_HAND_LEFT, 0.01f, 320.0f, 0.5f}) == "haptic left 0.0100 320.0 0.500\n");
}

static void newest_command_per_hand_wins()
{
	HapticSender s("/nonexistent/control.sock");
	s.submit(0, 0.1f, 160, 0.2f);
	s.submit(0, 0.2f, 160, 0.9f);
	s.submit(1, 0.3f, 320, 0.4f);
	HapticCommand c{};
	assert(s.take(0, &c) && c.amplitude == 0.9f && c.duration_s == 0.2f);
	assert(!s.take(0, &c));
	assert(s.take(1, &c) && c.hand == 1 && c.frequency_hz == 320.0f);
}

#ifdef _WIN32
/* The same name the daemon listens on (src/platform/mod.rs, a_pipe_is_named_after_its_endpoint_path). */
static void names_the_pipe_like_the_daemon()
{
	assert(control_pipe_name("C:\\Users\\a b\\AppData\\Local\\cv1touch\\control.sock") ==
	       L"\\\\.\\pipe\\cv1touch-C:/Users/a b/AppData/Local/cv1touch/control.sock");
	assert(control_pipe_name("C:\\" + std::string(250, 'x')).empty());
}

static void finds_the_daemon_in_local_app_data()
{
	SetEnvironmentVariableW(L"LOCALAPPDATA", L"C:\\Users\\a b\\AppData\\Local");
	assert(HapticSender::default_socket_path() == "C:\\Users\\a b\\AppData\\Local\\cv1touch\\control.sock");
	SetEnvironmentVariableW(L"LOCALAPPDATA", L"C:\\Users\\a b\\AppData\\Local\\");
	assert(HapticSender::default_socket_path() == "C:\\Users\\a b\\AppData\\Local\\cv1touch\\control.sock");
}

static std::string temp_endpoint(const char *tag)
{
	char tmp[MAX_PATH + 1];
	const DWORD len = GetTempPathA(sizeof tmp, tmp);
	assert(len > 0 && len < sizeof tmp);
	return std::string(tmp, len) + "cv1t-" + tag + "-" + std::to_string(GetCurrentProcessId()) + ".sock";
}

/* What the daemon's pipe carries: owned by, and open to, this user only. */
static std::wstring this_user_only()
{
	HANDLE token = nullptr;
	assert(OpenProcessToken(GetCurrentProcess(), TOKEN_QUERY, &token));
	alignas(TOKEN_USER) unsigned char user[sizeof(TOKEN_USER) + SECURITY_MAX_SID_SIZE];
	DWORD size = 0;
	assert(GetTokenInformation(token, TokenUser, user, sizeof user, &size));
	CloseHandle(token);
	wchar_t *sid = nullptr;
	assert(ConvertSidToStringSidW(reinterpret_cast<TOKEN_USER *>(user)->User.Sid, &sid));
	const std::wstring sddl = L"O:" + std::wstring(sid) + L"D:P(A;;GA;;;" + sid + L")";
	LocalFree(sid);
	return sddl;
}

/* A one-instance pipe server at `name` with the security descriptor `sddl`. */
static HANDLE serve_pipe(const std::wstring &name, const std::wstring &sddl)
{
	PSECURITY_DESCRIPTOR descriptor = nullptr;
	assert(ConvertStringSecurityDescriptorToSecurityDescriptorW(sddl.c_str(), SDDL_REVISION_1, &descriptor, nullptr));
	SECURITY_ATTRIBUTES attributes{sizeof attributes, descriptor, FALSE};
	HANDLE srv = CreateNamedPipeW(name.c_str(), PIPE_ACCESS_DUPLEX | FILE_FLAG_FIRST_PIPE_INSTANCE,
				      PIPE_TYPE_BYTE | PIPE_READMODE_BYTE | PIPE_WAIT | PIPE_REJECT_REMOTE_CLIENTS, 1, 4096, 4096, 0,
				      &attributes);
	const DWORD error = GetLastError();
	LocalFree(descriptor);
	SetLastError(error);
	return srv;
}

/* Another account's pipe under the daemon's name gets nothing. Only an elevated administrator can
 * make another principal (the Administrators group) the owner; other tokens skip the test. */
static void refuses_a_pipe_another_account_owns()
{
	const std::string path = temp_endpoint("squat");
	HANDLE srv = serve_pipe(control_pipe_name(path), L"O:BAD:(A;;GA;;;WD)");
	if (srv == INVALID_HANDLE_VALUE) {
		assert(GetLastError() == ERROR_INVALID_OWNER);
		printf("refuses_a_pipe_another_account_owns skipped: needs an elevated administrator token\n");
		return;
	}
	assert(!send_control_line(path, "status\n"));
	// the client left without writing a byte
	ConnectNamedPipe(srv, nullptr);
	char buf[16];
	DWORD n = 0;
	assert(!ReadFile(srv, buf, sizeof buf, &n, nullptr) && n == 0);
	CloseHandle(srv);
}

/* The worker delivers the line to the daemon's pipe and reads the reply. */
static void sends_to_the_socket()
{
	const std::string path = temp_endpoint("hapt");
	const std::wstring name = control_pipe_name(path);
	assert(!name.empty());
	HANDLE srv = serve_pipe(name, this_user_only());
	assert(srv != INVALID_HANDLE_VALUE);
	HapticSender s(path);
	s.start();
	s.submit(CV1T_HAND_RIGHT, 0.25f, 160.0f, 1.0f);
	assert(ConnectNamedPipe(srv, nullptr) || GetLastError() == ERROR_PIPE_CONNECTED);
	char buf[128] = {};
	DWORD n = 0;
	assert(ReadFile(srv, buf, sizeof buf - 1, &n, nullptr) && n > 0);
	assert(std::string(buf, n) == "haptic right 0.2500 160.0 1.000\n");
	DWORD wrote = 0;
	assert(WriteFile(srv, "ok\n", 3, &wrote, nullptr) && wrote == 3);
	FlushFileBuffers(srv);
	DisconnectNamedPipe(srv);
	CloseHandle(srv);
	s.stop();
	// nobody listening: refused at once, not after a timeout
	assert(!send_control_line(path, "status\n"));
}
#else
/* The worker delivers the line to a control socket and reads the reply. */
static void sends_to_the_socket()
{
	SocketDir fixture;
	const std::string path = fixture.path() + "/control.sock";
	unlink(path.c_str());
	int srv = socket(AF_UNIX, SOCK_STREAM, 0);
	sockaddr_un addr{};
	addr.sun_family = AF_UNIX;
	assert(path.size() < sizeof addr.sun_path);
	std::strncpy(addr.sun_path, path.c_str(), sizeof addr.sun_path - 1);
	assert(bind(srv, reinterpret_cast<sockaddr *>(&addr), sizeof addr) == 0 && listen(srv, 4) == 0);
	HapticSender s(path);
	s.start();
	s.submit(CV1T_HAND_RIGHT, 0.25f, 160.0f, 1.0f);
	int conn = accept(srv, nullptr, nullptr);
	assert(conn >= 0);
	char buf[128] = {};
	ssize_t n = read(conn, buf, sizeof buf - 1);
	assert(n > 0 && std::string(buf) == "haptic right 0.2500 160.0 1.000\n");
	assert(write(conn, "ok\n", 3) == 3);
	close(conn);
	s.stop();
	close(srv);
	unlink(path.c_str());
}
#endif

int main()
{
	clamps_what_apps_send();
	formats_the_daemon_command();
	newest_command_per_hand_wins();
#ifdef _WIN32
	names_the_pipe_like_the_daemon();
	finds_the_daemon_in_local_app_data();
	refuses_a_pipe_another_account_owns();
#endif
	sends_to_the_socket();
	printf("test_haptics ok\n");
	return 0;
}
