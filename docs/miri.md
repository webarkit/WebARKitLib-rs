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

# Disable isolation if a test needs filesystem access Miri rejects
MIRIFLAGS="-Zmiri-disable-isolation" cargo +nightly miri test -p webarkitlib-rs --lib
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

## Why `-Zmiri-strict-provenance` is NOT enabled in CI

The first CI run with strict provenance enabled tripped inside
`crossbeam-epoch` (a transitive dep of `rayon`), called from
`ar2::feature_map`'s `par_chunks_mut`. That dep predates the Strict
Provenance APIs and still uses integer-to-pointer casts internally.

Running CI with strict provenance against unmigrated third-party deps
produces failures we cannot fix in this repo. We keep regular Miri (the
core UB net) on by default and revisit strict provenance once the
ecosystem catches up. Developers are encouraged to spot-check their own
new `unsafe` code locally with `MIRIFLAGS="-Zmiri-strict-provenance"`.

## CI gate status

The Miri job currently runs with `continue-on-error: true` (advisory).
It will be ratcheted to a required gate once initial UB findings
surfaced by the first runs are resolved via follow-up PRs tracked under
[#182](https://github.com/webarkit/WebARKitLib-rs/issues/182).

Each finding gets its own fix PR — never a mega-PR — mirroring the #180
cleanup series pattern.
