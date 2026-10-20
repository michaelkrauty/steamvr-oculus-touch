// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

#include "haptics.h"
#include <cassert>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <string>
#include <sys/socket.h>
#include <sys/stat.h>
#include <poll.h>
#include <sys/wait.h>
#include <sys/un.h>
#include <unistd.h>

// Make the peer-close race deterministic without replacing the real socket write.
static int peer_closed_fd = -1;
static void wait_for_peer_close()
{
    if (peer_closed_fd >= 0) {
        char byte;
        assert(read(peer_closed_fd, &byte, 1) == 1);
        close(peer_closed_fd);
        peer_closed_fd = -1;
    }
}
extern "C" ssize_t __real_write(int, const void *, size_t);
extern "C" ssize_t __wrap_write(int fd, const void *p, size_t n)
{
    wait_for_peer_close();
    return __real_write(fd, p, n);
}

extern "C" ssize_t __real_send(int, const void *, size_t, int);
extern "C" ssize_t __wrap_send(int fd, const void *p, size_t n, int flags)
{
    wait_for_peer_close();
    return __real_send(fd, p, n, flags);
}

int main()
{
    const std::string dir = (std::filesystem::temp_directory_path() / ("cv1t-disconnect-" + std::to_string(getpid()))).string();
    assert(mkdir(dir.c_str(), 0700) == 0);
    const std::string path = dir + "/control.sock";
    const std::string fallback = dir + "/fallback.sock";
    const int server = socket(AF_UNIX, SOCK_STREAM, 0);
    assert(server >= 0);
    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    assert(path.size() < sizeof addr.sun_path);
    std::strncpy(addr.sun_path, path.c_str(), sizeof addr.sun_path - 1);
    assert(bind(server, reinterpret_cast<sockaddr *>(&addr), sizeof addr) == 0);
    assert(listen(server, 4) == 0);
    const int other = socket(AF_UNIX, SOCK_STREAM, 0);
    assert(other >= 0);
    assert(fallback.size() < sizeof addr.sun_path);
    std::strcpy(addr.sun_path, fallback.c_str());
    assert(bind(other, reinterpret_cast<sockaddr *>(&addr), sizeof addr) == 0);
    assert(listen(other, 4) == 0);
    int gate[2];
    assert(pipe(gate) == 0);
    const pid_t child = fork();
    assert(child >= 0);
    if (child == 0) {
        close(gate[1]);
        peer_closed_fd = gate[0];
        signal(SIGPIPE, SIG_DFL);
        cv1t::ControlClient client(path, fallback);
        const bool ok = client.send("haptic right 0.1 160 1\n");
        _exit(ok ? 1 : 0);
    }
    close(gate[0]);
    const int peer = accept(server, nullptr, nullptr);
    assert(peer >= 0);
    close(peer);
    assert(write(gate[1], "x", 1) == 1);
    close(gate[1]);
    int status = 0;
    assert(waitpid(child, &status, 0) == child);
    pollfd pending{other, POLLIN, 0};
    assert(poll(&pending, 1, 0) == 0); // EPIPE after connect must not resend to the fallback
    close(server);
    close(other);
    unlink(path.c_str());
    unlink(fallback.c_str());
    rmdir(dir.c_str());
    assert(WIFEXITED(status) && WEXITSTATUS(status) == 0);
    std::puts("test_control_disconnect ok");
}
