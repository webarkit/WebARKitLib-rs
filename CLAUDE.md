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

## 5. Quick checklist before opening a PR

- [ ] `cargo fmt --all -- --check` clean (run `cargo fmt --all` to fix)
- [ ] `cargo build --all-features` clean
- [ ] `cargo clippy --all-targets --all-features` clean
- [ ] `cargo test --all-features` green
- [ ] New source files carry the LGPL-3.0 header
- [ ] Error sites use `arlog_*!` + `return Err`
- [ ] No `println!` / `eprintln!` added in library code
- [ ] CHANGELOG.md **not** touched
- [ ] Branch is off current `dev`

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