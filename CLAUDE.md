# CLAUDE.md — Project Conventions for WebARKitLib-rs

This file documents conventions for contributors (human and AI) working on
WebARKitLib-rs, a Rust port of ARToolKit. Keep it short, actionable, and
update it when a convention changes.

---

## 1. Idiomatic Rust

Prefer idiomatic Rust over a mechanical C-to-Rust transliteration, even
though this is a port.

- **Errors, not panics.** Library code returns `Result<T, E>`; reserve
  `panic!` / `unwrap()` / `expect()` for truly unrecoverable invariants
  (and prefer `debug_assert!` where possible). Use `?` to propagate.
- **Borrow, don't clone.** Take `&[T]` / `&str` in APIs; clone only when
  ownership is genuinely required. Use `Cow<'_, T>` when the choice is
  conditional.
- **Iterators over index loops** where it doesn't hurt clarity or perf.
- **Exhaustive `match`** on enums; avoid catch-all `_ =>` unless the
  intent is forward-compatibility.
- **`#[must_use]`** on constructors and fallible helpers whose result
  callers should not silently drop.
- **No `unsafe` without a `// SAFETY:` comment** explaining the invariant.
- **Newtypes** for units / IDs that C used as bare `int` (e.g. marker IDs,
  pattern handles) when it prevents real bugs.
- **Naming**: `snake_case` for fns/locals, `CamelCase` for types,
  `SCREAMING_SNAKE_CASE` for consts. Keep the C name in a doc comment
  (`/// C equivalent: arParamLT`) when porting, so the mapping stays
  discoverable — but the Rust name should be idiomatic.

Run `cargo clippy --all-targets --all-features` and keep it clean.

---

## 2. Logging — use the `arlog` system

**Do not use `println!` / `eprintln!` / bare `log::*` in library code.**
Use the project's `arlog_*` macros from `crates/core/src/arlog.rs`:

| Macro       | ARToolKit C equivalent | Use for                              |
|-------------|------------------------|--------------------------------------|
| `arlog_d!`  | `ARLOGd` / `ARLOGi`    | Per-frame rejections, trace details  |
| `arlog_i!`  | `ARLOGi`               | One-shot informational events        |
| `arlog_w!`  | `ARLOGw`               | Recoverable anomalies                |
| `arlog_e!`  | `ARLOGe`               | Misconfiguration / wiring errors     |

### Importing arlog macros (IMPORTANT)

The arlog macros **must be imported by name**, not by module:

```rust
// ✅ CORRECT
use crate::{arlog_d, arlog_e, arlog_i, arlog_w};
arlog_e!("error message");

// ❌ WRONG — causes "macro not found" errors
use crate::arlog;
arlog_e!("error message");  // Error: cannot find macro `arlog_e`
```

Import only the log levels you actually use in your file.

### "Log + return Err" pattern

Every error-returning site gets a matching `arlog_*!` immediately before
the `return Err(...)`:

```rust
if scales == 0 {
    arlog_e!("ar2_gen_feature_map: image set has no scales");
    return Err(Ar2Error::InvalidImageSet);
}
```

- **`arlog_e!`** for misconfiguration, null/invalid inputs, bad wiring —
  things a caller should fix.
- **`arlog_d!`** for per-frame rejections (low contrast, decode failure,
  EDC fail) — expected at runtime, noisy, debug-level only.

Canonical examples: PR #76 (matrix.rs, bch.rs) and PR #77 (marker.rs,
pattern.rs, image_set.rs, labeling.rs, ar2/feature_map.rs).

### Enabling output

The macros compile unconditionally. For output to appear, a logger must
be installed:

- **Desktop examples/tests**: enable the `log-helpers` feature and call
  `ar_log_init_default()` (installs `env_logger`).
- **Wasm**: enable `log-helpers` and call `ar_log_init_wasm()` (installs
  `console_log`).
- **Library consumers**: install any `log`-compatible backend themselves;
  our macros will route through it.

Examples that need log output must declare
`required-features = ["log-helpers"]` in `Cargo.toml`. See the
`load_nft` example as the canonical pattern.

---

## 3. SIMD and multithreading

When porting hot paths, reach for SIMD and parallelism where it is
measurable and the scalar fallback stays correct.

### SIMD

Feature flags already defined:

- `simd-x86-avx2`
- `simd-x86-sse41`
- `simd-wasm32`

Use runtime detection so binaries stay portable:

```rust
#[cfg(all(target_arch = "x86_64", feature = "simd-x86-avx2"))]
if is_x86_feature_detected!("avx2") {
    return unsafe { avx2_impl(input) };
}
// scalar fallback
scalar_impl(input)
```

Rules of thumb:
- Always keep a scalar fallback; SIMD is an optimization, not a
  requirement.
- Gate `unsafe` SIMD intrinsics behind both the `cfg(target_arch)` and
  the feature flag.
- Add a benchmark (in `benchmarks/`) alongside any new SIMD path.

### Multithreading

Use `rayon` for data-parallel loops when the workload is large enough to
amortize thread-pool overhead. Canonical example: `ar2_gen_feature_map`
Stage 3 in `crates/core/src/ar2/feature_map.rs`.

- Prefer `par_iter` / `par_iter_mut` over hand-rolled threads.
- Don't parallelize small loops — measure first.
- Keep wasm builds single-threaded unless explicitly targeting
  `wasm32-unknown-unknown` with threads.

---

## 4. Housekeeping

- **License header**: every new source file starts with the LGPL-3.0
  header. Canonical template: [`.claude/HEADER.txt`](.claude/HEADER.txt)
  — copy verbatim and substitute `<FILENAME>`, `<YEAR>`, `<AUTHOR>`,
  `<@HANDLE>`, `<URL>`.
- **Fresh branch per issue/sub-issue**: branch from `dev`, never stack
  unrelated work on the same branch — avoids merge conflicts and keeps
  PR review focused.
- **CHANGELOG.md is release-only**: never edit it in feature PRs. It is
  rewritten at release time from the merged PR history.
- **Examples**: put runnable examples under `crates/<crate>/examples/`
  with a short header comment explaining what they demonstrate and any
  required assets.
- **Tests**: unit tests live next to the code (`#[cfg(test)] mod tests`);
  integration tests in `tests/`.

---

## 5. Pre-Commit Verification Workflow

**Run these checks BEFORE committing and pushing. CI will enforce them.**

```bash
# 1. Fix formatting and verify it's clean
cargo fmt --all
cargo fmt --all -- --check

# 2. Build the project
cargo build --all-features

# 3. Run clippy (strict: warnings as errors)
cargo clippy --all-targets --all-features -- --deny warnings

# 4. Run tests
cargo test --all-features
```

**All four checks must pass before pushing.** If `cargo fmt --all` makes changes, add those changes to your commit.

## 6. Quick checklist before opening a PR

- [ ] `cargo fmt --all -- --check` clean (run `cargo fmt --all` to fix)
- [ ] `cargo build --all-features` clean
- [ ] `cargo clippy --all-targets --all-features` clean
- [ ] `cargo test --all-features` green
- [ ] New source files carry the LGPL-3.0 header
- [ ] Error sites use `arlog_*!` + `return Err`
- [ ] No `println!` / `eprintln!` added in library code
- [ ] CHANGELOG.md **not** touched
- [ ] Branch is off current `dev`

## 7. Codebase clippy conventions (#180 lessons)

The strict clippy gate (`--all-targets --all-features -- -D warnings`)
is enforced in CI by the `kpm-build (ubuntu-latest)` job. The patterns
below are non-obvious — they were discovered during the #180 cleanup
series and aren't taught by clippy's lint messages alone.

### 7.1 Struct-init over `Default::default()` + field reassign

Clippy's `field_reassign_with_default` fires on the C-style pattern.

```rust
// ❌ Don't
let mut h = ARHandle::default();
h.xsize = w;
h.ysize = h;

// ✅ Do
let mut h = ARHandle {
    xsize: w,
    ysize: h,
    ..Default::default()
};
```

If subsequent code mutates other fields (e.g. `h.o2i[i] = ...`), keep
the binding `mut`. Method calls (`h.set_pixel_format(...)`) stay
after the initializer — they're not field reassigns.

Reference: PR #186, PR #188.

### 7.2 FFI shim cfg must match its caller's cfg

Dual-mode parity tests live under `#[cfg(all(test, feature = "dual-mode"))]`.
The `extern "C"` block declaring the `webarkit_cpp_*` shims it calls
must use the **same** cfg — not just `#[cfg(feature = "dual-mode")]` —
otherwise the lib target sees an extern with no caller and clippy
fires `dead_code` under `--all-features`.

```rust
// ❌ Don't (extern declared in non-test builds where it has no caller)
#[cfg(feature = "dual-mode")]
extern "C" {
    fn webarkit_cpp_foo(...) -> i32;
}

#[cfg(all(test, feature = "dual-mode"))]
mod dual_mode_tests {
    use super::*;
    #[test] fn foo_matches_cpp() { unsafe { webarkit_cpp_foo(...) }; }
}

// ✅ Do (extern and caller share the same gate)
#[cfg(all(test, feature = "dual-mode"))]
extern "C" {
    fn webarkit_cpp_foo(...) -> i32;
}
```

Reference: PR #185.

### 7.3 Items must come **above** `#[cfg(test)] mod tests`

Clippy's `items_after_test_module` fires when `pub fn ...` or any item
appears after the test module. When adding helpers to a file that
already has a `mod tests`, insert them above the `#[cfg(test)]` line.

Reference: PR #184 (`mat_mul_dff`, `dot_product*`).

### 7.4 SIMD runtime-dispatch variants: inline `#[allow]` + rationale

Functions like `get_similarity_sse41` / `get_similarity_avx2` are
runtime-dispatched alternatives to a scalar fallback. Their
signatures are **locked** to match each other and the scalar version.
Prefer inline `#[allow(clippy::too_many_arguments)]` with a
`// rationale:` line over raising the threshold in `clippy.toml` — the
allow scopes to exactly the sites that need it.

```rust
#[cfg(all(feature = "simd-x86-sse41", target_arch = "x86_64"))]
#[target_feature(enable = "sse4.1")]
// rationale: SIMD variant of get_similarity; signature locked to match
// the scalar fallback and sibling SIMD impl for runtime dispatch via
// is_x86_feature_detected!.
#[allow(clippy::too_many_arguments)]
unsafe fn get_similarity_sse41(...) -> Option<f32> { ... }
```

Reference: PR #187.

### 7.5 Auto-generated fixtures: file-scoped `#![allow]` + rationale

For test files containing constants captured verbatim from external
sources (e.g. C++ baseline output dumped by `kpm_dump_fixtures`),
preserve source-traceability with a file-scoped allow rather than
truncating the literals.

```rust
//! KPM regression tests — numerical validation against C++ baseline.

#![allow(dead_code)]
// rationale: SCREEN/WORLD/pose constants below were generated by the
// C++ kpm_dump_fixtures tool. Extra decimal digits document the
// captured C++ output verbatim; truncating to f32-exact would
// round-trip to the same bit pattern but lose traceability (#180).
#![allow(clippy::excessive_precision)]
```

Reference: PR #188 (`tests/kpm_regression.rs`).

### 7.6 Re-run strict clippy after the lib goes clean

`cargo clippy --all-targets` halts on lib errors before checking
examples/tests/benches. After clearing the last lib lint, **re-run**
the strict command — previously masked lints in non-lib targets will
surface. PR #188 cleaned up 26 such lints unmasked by PR #187.

## 📝 Documentation & Style
- All comments, docstrings, and commit messages must be in **English**.
- Use `Result<T, PureCvError>` for error handling instead of `panic!`.
- Create tests for every functions / type and benchmarks if possible.
- Add HEADER.txt to every new file you create. You found it in `.claude\HEADER.txt`.

> **CI will reject any PR that fails `cargo fmt -- --check` or `cargo clippy`.**

## 🐙 Github Instructions & Conventional Commits
- When creating a PR always start from the `dev` branch and point against `dev` branch.
- **MANDATORY:** You must use the [Conventional Commits](https://www.conventionalcommits.org/) specification for all commit messages. This is strictly required for our automated `git-cliff` changelog generation.
- **Format:** `<type>(<optional scope>): <description>`
- **Allowed Types:**
  - `feat`: A new feature or algorithm implementation.
  - `fix`: A bug fix.
  - `perf`: A code change that improves performance (e.g., optimizations).
  - `doc`: Documentation only changes.
  - `refactor`: A code change that neither fixes a bug nor adds a feature.
  - `test`: Adding missing tests or correcting existing ones.
  - `chore`: Changes to the build process, dependencies, or auxiliary tools.
- **Preferred Scopes for PureCV:** Use scopes to categorize the architectural work, such as `(simd)`, `(wasm)`, `(parallel)`, `(core)`, `(imgproc)`.
- **Examples of valid commits:**
  - `feat(simd): implement AVX2 support for matrix multiplication`
  - `perf(wasm): optimize memory allocation for Emscripten target`
  - `fix(core): resolve out-of-bounds error in Row-Major layout`
  - `doc: add usage examples for parallel processing`
