# Inspection identity recovery: follow-up design

This is a design for a later change, not an implemented recovery path. Its prerequisites are:

- Conservative containment bounds for every possible LED observation of the other hand. The existing position and rotation errors are standard deviations, not containment bounds; a covariance multiplier cannot supply this premise.
- An exposure-time peer estimate that excludes the suspect sensor. A recent update from another sensor does not establish that exclusion; the current estimator needs explicit provenance or a separate estimate.
- A certificate for each exposure, with no identity latch. Historical motion validation grants eligibility, not permission to export the new correspondence.
- Three consecutive certified exposures before output resumes. Every later output needs its own certificate until a fresh independent-motion proof establishes ordinary tracking again.
- Fitting and refinement that cannot reintroduce excluded blobs. The final labelled correspondence and every exported label must use only the certified subset.

## Proposed recovery

After correspondence loss, keep the candidate private and reset its ordinary motion proof. A hand that passed independent-motion validation earlier in the inspection may request geometric certification. The other hand must have its own independently established identity at this same sensor exposure. Two recovering candidates cannot certify each other.

Compare the relative controller transforms in camera space with those from the suspect-excluded peers. Cover every LED the other hand could produce with a conservative image envelope, including unlabelled LEDs, model and centroid error, exposure timing, and all admissible poses. Exclude every blob that intersects that envelope or belongs to the other hand. At least five distinct usable correspondences must remain, and all existing fit-quality checks must pass on that subset alone.

Missing bounds, stale evidence, overlapping envelopes, multiple admissible fits, or exhausted computation budgets mean unknown. Compute the geometry on sensor analysis workers with fixed storage and bounded iterations. Add no radio-thread work. Before publication, recheck the inspection generation, cancellation and exposure under the sensor lock.

## Current limit and required evidence

Software blob IDs follow nearest-neighbour associations. A large image jump can be rejected without changing blobwatch's general tracking, but an overlap can transfer those IDs to another physical controller inside any useful motion bound. A swap during an image overlap is caught only once the hands' rotations diverge; identical motion through an overlap can retain the swapped identity. Its lifetime is not bounded while that motion remains indistinguishable. The geometric-certificate follow-up is where physical provenance must be addressed.

Tests must keep physical-source IDs hidden from the implementation. Cover both hand orders, swaps before and after validation, large and small displacements, merges, independent and rigid motion, delayed completion, expiry, and refinement. Assert ownership at exported labels and priors, movement evidence and placement probes. Increasing uncertainty must never turn an unknown correspondence into an accepted one.

The [current one-second-cadence comparison](self-recalibration.md) provides a baseline, not a prediction for this follow-up. A policy that kept identity across correspondence loss detects a 0.4 m perturbation at 25 seconds and places it at 38 seconds, but retains that identity unsafely. The current rule requires fresh identity after a break and accumulates eighty validated observed checks across at least two windows, retracting retrospectively invalidated epochs. Three runs detect at 25 seconds and place at 48 seconds; other schedules have placed at 49 and 51 seconds. Static or indistinguishable motion can still prevent confirmation. Measure repeated native-timed replays before claiming that geometric recovery improves it.
