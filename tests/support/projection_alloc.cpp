// SPDX-License-Identifier: Apache-2.0
#include <cstdint>
#include <cstdlib>
#include <new>
#include <opencv2/core.hpp>
#include <opencv2/core/version.hpp>
// Independent glibc heap-entry layer. These are overlapping, NOT additive to Mat/new.
static bool counting;
static uint64_t heap_calls, heap_bytes;
#if defined(__GLIBC__)
extern "C" void *__libc_malloc(size_t);
extern "C" void *__libc_calloc(size_t, size_t);
extern "C" void *__libc_realloc(void *, size_t);
extern "C" void *__libc_memalign(size_t, size_t);
extern "C" void __libc_free(void *);
static void hit(size_t n) {
    if (counting) {
        heap_calls++;
        heap_bytes += n;
    }
}
extern "C" void *malloc(size_t n) noexcept {
    hit(n);
    return __libc_malloc(n);
}
extern "C" void *calloc(size_t a, size_t b) noexcept {
    hit(a * b);
    return __libc_calloc(a, b);
}
extern "C" void *realloc(void *p, size_t n) noexcept {
    hit(n);
    return __libc_realloc(p, n);
}
extern "C" void free(void *p) noexcept { __libc_free(p); }
extern "C" void *memalign(size_t a, size_t n) noexcept {
    hit(n);
    return __libc_memalign(a, n);
}
extern "C" void *aligned_alloc(size_t a, size_t n) noexcept {
    hit(n);
    return __libc_memalign(a, n);
}
extern "C" int posix_memalign(void **p, size_t a, size_t n) noexcept {
    hit(n);
    void *q = __libc_memalign(a, n);
    if (!q)
        return 12;
    *p = q;
    return 0;
}

#endif

static uint64_t matrices, matrix_bytes, cpp_news, cpp_bytes;
void *operator new(std::size_t n) {
    if (counting) {
        cpp_news++;
        cpp_bytes += n;
    }
    void *p = malloc(n);
    if (!p)
        throw std::bad_alloc();
    return p;
}
void *operator new[](std::size_t n) { return ::operator new(n); }
void operator delete(void *p) noexcept { free(p); }
void operator delete[](void *p) noexcept { free(p); }
void operator delete(void *p, std::size_t) noexcept { free(p); }
void operator delete[](void *p, std::size_t) noexcept { free(p); }
// OpenCV 3 used int allocation flags; 4/5 use the typed AccessFlag enum.
#if CV_VERSION_MAJOR >= 4
using AllocationFlags = cv::AccessFlag;
#else
using AllocationFlags = int;
#endif
struct Counter : cv::MatAllocator {
    cv::MatAllocator *base = cv::Mat::getDefaultAllocator();
    cv::UMatData *allocate(int dims, const int *sizes, int type, void *data, size_t *step,
                           AllocationFlags flags, cv::UMatUsageFlags usage) const override {
        auto *p = base->allocate(dims, sizes, type, data, step, flags, usage);
        if (counting && !data) {
            matrices++;
            matrix_bytes += p->size;
        }
        return p;
    }
    bool allocate(cv::UMatData *d, AllocationFlags a, cv::UMatUsageFlags u) const override {
        return base->allocate(d, a, u);
    }
    void deallocate(cv::UMatData *d) const override { base->deallocate(d); }
};
extern "C" void count_begin(void) {
    static Counter c;
    cv::Mat::setDefaultAllocator(&c);
    heap_calls = heap_bytes = matrices = matrix_bytes = cpp_news = cpp_bytes = 0;
    counting = true;
}
extern "C" void count_end(uint64_t *out) {
    counting = false;
    out[0] = matrices;
    out[1] = matrix_bytes;
    out[2] = cpp_news;
    out[3] = cpp_bytes;
    out[4] = heap_calls;
    out[5] = heap_bytes;
}

extern "C" int projection_fast_path(void) { return CV_VERSION_MAJOR == 5; }

extern "C" int projection_heap_counters(void) {
#if defined(__GLIBC__)
    return 1;
#else
    return 0;
#endif
}
