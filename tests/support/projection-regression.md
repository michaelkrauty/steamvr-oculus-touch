# Native projection regression

`meson test -C <build> projection_search` runs synthetic geometry only. It uses the actual bridge, pose helper, LED model and correspondence search; it does not open devices or read calibration files.

Each fixture runs the frozen vector projection oracle and the production bridge against the same observations. The measured region includes **both** `correspondence_search_set_blobs` and `correspondence_search_find_one_pose`, including undistortion and blob-search setup. Model/search construction, teardown, input generation and downstream label checks are outside that region. Cold means a new search with unlabelled observations and no prior, not first-ever library initialization.

The regression compares raw pose and score storage, acceptance, trial/check counts and labels before and after downstream marking. It includes successful and unsuccessful shallow/deep searches, both distortion models, clean/noisy observations and nonplanar/planar/nearly-collinear models of 7, 12, 16, 24, 44 and 64 points. Direct bridge tests cover -1, 0, 1, 4, 7, 12, 16, 24, 44, 63, 64, 65, 255 and 1024 points, nonzero distortion, exact pi rotation and near/negative depth, guard elements and legacy exception behavior. Larger models are tested directly, not through the pose helper's existing fixed-size arrays. Concurrent direct calls check reentrancy. Test-only wrappers call the real projection functions, then replace their output Mat with newly allocated row, column and noncontiguous column storage and poison the abandoned view to exercise actual returned-storage consumption.

Allocation layers are recorded separately and **must not be summed**:

- OpenCV Mat payload requests and bytes, via its default allocator.
- C++ `new`/`new[]` requests and bytes, including vector and Mat metadata allocations.
- On glibc only, exported heap-entry requests and requested bytes (`malloc`, `calloc`, `realloc`, `memalign`, `aligned_alloc`, `posix_memalign`). These include requests already counted by the other layers, not private allocator bookkeeping or resident memory. Other C libraries report this layer as unavailable (zero).

Instrumentation is confined to this test executable. Counting runs single-threaded; concurrency checks run while counting is disabled. The test requires per-layer reduction for checked poses on OpenCV 5 and equal counts on the retained vector path. It does not require or claim zero total allocations. Undistortion/setup and pinhole-internal allocations remain. There is no timing, CPU, FPS or tracking-latency claim.

Only OpenCV 5 enables the bounded Mat-input path. OpenCV 3/4 and future major versions retain the original vector implementation until their numerical and output-layout contracts are independently verified. This deliberately preserves compatibility without claiming unexecuted cross-version equivalence. The direct oracle and full-operation equivalence regression remain useful on those versions, but no allocation reduction is expected there.
