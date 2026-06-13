# Miri — Undefined Behavior Validation

[Miri](https://github.com/rust-lang/miri) is the Rust MIR interpreter.
It detects undefined behavior that escapes regular tests: use-after-free,
out-of-bounds reads, invalid `unsafe` invariants, uninitialized-memory
reads, and reference-aliasing violations.

WebARKitLib-rs runs Miri in CI as the second half of the post-M9 safety
net (sister to #180 clippy strictness).

---

## What Miri validates here

CI runs:

```bash
cargo +nightly miri test -p webarkitlib-rs --lib
```

That exercises the pure-Rust lib of the `webarkitlib-rs` crate, which
covers the modules where manual buffer math and `unsafe` live:

- `crates/core/src/ar/` — labeling, pattern matching, marker decoding,
  image processing (C ARToolKit port).
- `crates/core/src/ar2/` — `image_set`, `feature_map`, `byteorder` I/O
  on disk buffers.
- `crates/core/src/kpm/` — KPM pipeline.
- `crates/core/src/kpm/freak/` — `math`, `homography`, `clustering`,
  `matcher`, `visual_database` (the freshly-ported M6→M9 surface).

## What Miri does NOT validate

- **`--features ffi-backend`** — Miri cannot execute foreign C/C++
  functions, so all FFI shims would fail under it. The `webarkit_cpp_*`
  externs are intentionally outside Miri's scope.
- **`--features simd-x86-sse41` / `simd-x86-avx2`** — Miri's x86 SIMD
  intrinsic support is incomplete. Scalar fallbacks ARE exercised under
  Miri; SIMD parity is covered by the scalar/SIMD parity tests in
  CI separately.
- **Integration tests under `tests/`** that depend on FFI (e.g.
  `kpm_regression`, `cross_stack_parity`, `absolute_corner_error` in
  `dual-mode`) — these need the C++ baseline to run.
- **Heavy algorithmic / pipeline tests** annotated with
  `#[cfg_attr(miri, ignore)]` (see next section).

## Tests skipped under Miri (`#[cfg_attr(miri, ignore)]`)

Some unit tests run full pipelines — BHC tree construction over hundreds
of descriptors, DoG keypoint detection on real benchmark images, full
`ar2_gen_feature_map` runs. Native `cargo test` finishes them in
milliseconds because the code is compiled and parallelized. Miri
interprets MIR single-threaded, so the same tests can take 30+ minutes
each — and they exercise no `unsafe` boundary that targeted unit tests
don't already cover (#194).

We annotate these with `#[cfg_attr(miri, ignore)]`:

```rust
#[test]
#[cfg_attr(miri, ignore)] // #194: full pipeline — too slow under Miri
fn test_heavy_pipeline() { ... }
```

Effect: the test still runs under regular `cargo test`; it's skipped
only under `cargo miri test`. Targeted unit tests on `unsafe`
boundaries (`hamming_distance_*`, descriptor pack/unpack, byteorder
reads, image_proc indexing) stay enabled — those are the ones Miri
actually validates.

To run a Miri-ignored test locally for investigation:

```bash
cargo +nightly miri test -p webarkitlib-rs --lib <test_name> -- --ignored
```

## Running Miri locally

```bash
# One-time setup (matches the CI pin — update if CI pin changes)
rustup toolchain install nightly-2026-06-01 --component miri
cargo +nightly-2026-06-01 miri setup

# Run the full Miri suite (matches CI)
cargo +nightly-2026-06-01 miri test -p webarkitlib-rs --lib

# Run a single test
cargo +nightly-2026-06-01 miri test -p webarkitlib-rs --lib <test_name>
```

Expect the run to be **5–30× slower** than `cargo test`. This is normal
— Miri interprets MIR rather than executing native code.

## Investigating failures

When Miri reports UB, useful environment knobs:

```bash
# Full backtrace on the offending operation
MIRIFLAGS="-Zmiri-backtrace=full" cargo +nightly miri test -p webarkitlib-rs --lib

# Strict provenance (NOT enabled in CI — see note below; useful locally
# for spot-checking our own code)
MIRIFLAGS="-Zmiri-strict-provenance" cargo +nightly miri test -p webarkitlib-rs --lib

# -Zmiri-disable-isolation (already set in CI — lets tests using
# tempfile / disk I/O run; e.g. ar2::feature_set roundtrip)
MIRIFLAGS="-Zmiri-disable-isolation" cargo +nightly miri test -p webarkitlib-rs --lib

# -Zmiri-ignore-leaks (already set in CI — rayon's global thread pool
# spawns workers that aren't joined at exit; Miri flags that as a leak
# by default. Ignoring it is standard for rayon-using suites and does
# NOT weaken UB detection.)
MIRIFLAGS="-Zmiri-ignore-leaks" cargo +nightly miri test -p webarkitlib-rs --lib
```

If a finding is intentional (e.g. a known-safe `transmute` pattern that
Miri can't model), mark the test `#[ignore]` under Miri and open a
tracking issue. **Do not** add `unsafe` to silence Miri without
understanding the report.

## Bumping the nightly pin

The pin lives in `.github/workflows/ci.yml` as the `MIRI_NIGHTLY` job
env var. Bump when:

- CI hits a known nightly Miri regression (link the upstream issue in
  the PR description).
- Every ~3 months for general freshness.

Procedure:

1. Pick a recent known-good nightly from
   <https://rust-lang.github.io/rustup-components-history/> (filter on
   `miri`).
2. Update `MIRI_NIGHTLY` in `ci.yml` AND the "Running locally" snippet
   in this file.
3. Open a PR; CI will validate the new pin.

The `Swatinem/rust-cache@v2` key includes `MIRI_NIGHTLY`, so the cache
invalidates automatically on bump.

## Aliasing model: Tree Borrows (not Stacked Borrows)

CI sets `MIRIFLAGS=-Zmiri-tree-borrows`. We use the **Tree Borrows**
aliasing model instead of Miri's default **Stacked Borrows** because
Stacked Borrows trips inside `crossbeam-epoch` (a transitive dep of
`rayon`, reached from `ar2::feature_map`'s `par_chunks_mut`) on
patterns that are sound in practice. Tree Borrows is a newer
experimental aliasing model that accepts those patterns while still
catching real UB in our code.

If you want to spot-check our own `unsafe` under the stricter Stacked
Borrows locally, just drop the flag:

```bash
MIRIFLAGS="-Zmiri-backtrace=full" cargo +nightly miri test \
    -p webarkitlib-rs --lib <our_test_name>
```

## Why `-Zmiri-strict-provenance` is NOT enabled in CI

Strict provenance also trips inside `crossbeam-epoch` (it predates the
Strict Provenance APIs and still uses integer-to-pointer casts
internally). Running CI with strict provenance against unmigrated
third-party deps produces failures we cannot fix in this repo. We
revisit once the ecosystem catches up. Developers are encouraged to
spot-check their own new `unsafe` code locally with
`MIRIFLAGS="-Zmiri-strict-provenance"`.

## CI gate status

The Miri job currently runs with `continue-on-error: true` (advisory).
It will be ratcheted to a required gate once initial UB findings
surfaced by the first runs are resolved via follow-up PRs tracked under
[#182](https://github.com/webarkit/WebARKitLib-rs/issues/182).

Each finding gets its own fix PR — never a mega-PR — mirroring the #180
cleanup series pattern.
