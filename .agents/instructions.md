# Agent Instructions: Porting Core Data Structures (`include/AR/*.h`)

## Phase 1: Header Triage & Analysis
1. Analyze in the https://github.com/webarkit/WebARKitLib the `include/AR/` directory. Consider code from lib/src/AR, lib/SRC/AR2, lib/SRC/ARUtils
2. **IGNORE** the following headers: `video.h`, `gsub.h`, `gsub_lite.h`, `arMulti.h`, `arGL.h`, `arOSG.h`.
3. Focus strictly on: `ar.h`, `param.h`, `matrix.h`, `arFilterTransMat.h`.

## Phase 2: Struct Extraction & Translation
1. Identify ALL core data structures in `ar.h` and related math headers. This includes, but is not limited to:
   - Calibration: `ARParam`, `ARParamLT`
   - Tracking & State: `ARHandle`, `AR3DHandle`, `ARMarkerInfo`
   - Math: `ARMat`, `ARVec`
2. Create the corresponding Rust modules (e.g., `crates/core/src/types.rs`, `crates/core/src/math.rs`).
3. Translate the C `struct`s into Rust. Ensure correct memory alignment and types.
4. Add `#[derive(Debug, Default, Clone)]` where applicable.

## Phase 3: Validation
1. Write unit tests for every function and struct created to ensure their default instantiation behaves as expected and mathematical equivalence is maintained.
2. Ensure all generated documentation and inline comments are in English.

## Phase 4: Clippy hygiene (post-#180)

The strict clippy gate (`cargo clippy --workspace --all-targets --all-features -- -D warnings`)
is enforced in CI by the `kpm-build (ubuntu-latest)` job. When writing
or modifying any code in this repo, follow the conventions below to
avoid recreating lints cleared during the #180 series. Full reference
in [CLAUDE.md §7](../CLAUDE.md).

1. **Struct-init over `Default::default()` + field reassign.**
   `let x = T { f: ..., ..Default::default() };`, not
   `let mut x = T::default(); x.f = ...;`. Refs: PR #186, #188.

2. **FFI shim cfg must match its caller's cfg.** When declaring
   `extern "C" { fn webarkit_cpp_*(...) }` only for dual-mode parity
   tests, gate the block with `#[cfg(all(test, feature = "dual-mode"))]`
   — same as the caller module. A `#[cfg(feature = "dual-mode")]`
   alone leaks the extern into the non-test lib build, where it has
   no caller and trips `dead_code`. Ref: PR #185.

3. **Place new items above `#[cfg(test)] mod tests`.** Clippy's
   `items_after_test_module` fires otherwise. Ref: PR #184.

4. **SIMD runtime-dispatch variants get inline
   `#[allow(clippy::too_many_arguments)]` + `// rationale:`** — not
   a `clippy.toml` threshold raise. Their signatures are locked by
   the dispatcher contract. Ref: PR #187.

5. **Auto-generated test fixtures from external sources** (e.g.
   `tests/kpm_regression.rs` constants captured from C++) get
   file-scoped `#![allow(clippy::excessive_precision)]` + rationale
   to preserve source-traceability. Ref: PR #188.

6. **Re-run strict clippy after every clean.** Cargo halts on lib
   errors before checking `--all-targets`. Lints in
   examples/tests/benches stay masked until the lib is clean. Refs:
   PR #187 → #188 (26 lints surfaced this way).

When you must add a lint suppression, write it as `#[allow(...)]` with
a `// rationale:` comment explaining *why* — not `#[expect(...)]` or a
bare allow. The rationale is the contract for future contributors.