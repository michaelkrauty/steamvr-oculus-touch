# Native projection regression

`meson test -C <build> projection_search` runs synthetic geometry only. It uses the actual C camera, pose helper, LED model and correspondence search; it does not open devices or read calibration files.

The 108 scenarios in `projection_search_cases.inc` cover successful and unsuccessful shallow and deep searches, both distortion models, clean and noisy observations, and nonplanar, planar and nearly collinear models of 7, 12, 16, 24, 44 and 64 points. Their models, observations and expected outcomes were frozen from the version of this test that required the C camera and the former OpenCV 5.1 projection reference to agree. The measured region covers `correspondence_search_set_blobs` and `correspondence_search_find_one_pose`, including undistortion and blob-search setup; model and search construction, teardown and downstream label checks are outside it.

Acceptance, trial and check counts, pose bytes, the score's flags and counts, and labels before and after downstream marking must equal the frozen values exactly. The reprojection error may differ within a bound derived from the camera's 0.001 px per-coordinate tolerance. The score's orientation and position errors are not compared, because the score sets them only when a prior is given and these searches have none.

The search sources are compiled with counting allocator wrappers, and the camera with allocation poisons, so the test needs no linker interposition. The measured operation must make exactly one heap request, the point array of `correspondence_search_set_blobs`. Calls into the C library itself are not counted. The camera contracts also run concurrently on two threads to check reentrancy. There is no timing, CPU, FPS or tracking-latency claim.
