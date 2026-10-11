// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 michaelkrauty

#include "haptics.h"
#include <cassert>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <string>
#include <sys/socket.h>
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
    const std::string path = "/tmp/cv1t-disconnect-" + std::to_string(getpid()) + ".sock";
    const int server = socket(AF_UNIX, SOCK_STREAM, 0);
    assert(server >= 0);
    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    std::strncpy(addr.sun_path, path.c_str(), sizeof addr.sun_path - 1);
    assert(bind(server, reinterpret_cast<sockaddr *>(&addr), sizeof addr) == 0);
    assert(listen(server, 4) == 0);
    int gate[2];
    assert(pipe(gate) == 0);
    const pid_t child = fork();
    assert(child >= 0);
    if (child == 0) {
        close(gate[1]);
        peer_closed_fd = gate[0];
        signal(SIGPIPE, SIG_DFL);
        cv1t::send_control_line(path, "haptic right 0.1 160 1\n");
        _exit(0);
    }
    close(gate[0]);
    const int peer = accept(server, nullptr, nullptr);
    assert(peer >= 0);
    close(peer);
    assert(write(gate[1], "x", 1) == 1);
    close(gate[1]);
    int status = 0;
    assert(waitpid(child, &status, 0) == child);
    close(server);
    unlink(path.c_str());
    assert(WIFEXITED(status) && WEXITSTATUS(status) == 0);
    std::puts("test_control_disconnect ok");
}
