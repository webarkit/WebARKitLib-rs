# Design: Fix `ffi-backend` build from crates.io (issue #72)

- **Issue:** [webarkit/WebARKitLib-rs#72](https://github.com/webarkit/WebARKitLib-rs/issues/72)
- **Status:** Approved design, ready for implementation
- **Author:** @kalwalt (with brainstorming assistance)
- **Date:** 2026-04-22

---

## Understanding Summary

- **What:** Fix `ffi-backend` build failure from crates.io by making the required `WebARKitLib` C++ sources available to `crates/core/build.rs` without Python bootstrapping.
- **Why:** Today `build.rs` references `../../benchmarks/c_benchmark/src/WebARKitLib` which is gitignored and populated only by `python bootstrap.py` — so published crates ship without the C++ code and fail to build for every consumer.
- **Who:** Any user of `webarkitlib-rs` with the `ffi-backend` (or `dual-mode`) feature — from crates.io, git dependency, or docs.rs.
- **Approach:** Add upstream `webarkit/WebARKitLib` as a **git submodule** at `crates/core/third_party/WebARKitLib`, pinned by commit SHA, with `.gitmodules` tracking `master`. Point `build.rs` at the new path. Scope the published `.crate` with explicit `include = [...]` so only `lib/SRC/KPM/FreakMatcher/` + `include/` ship (~2.5 MB compressed).
- **Non-goals:**
  - No download-at-build-time logic.
  - No changes to what `ffi-backend` actually compiles or how it links.
  - No new platform support.
  - No change to the benchmark suite's `bootstrap.py` workflow — it keeps working at its existing location.
  - No CHANGELOG edits in this PR (project rule: CHANGELOG is updated at release time only).

## Assumptions

1. crates.io's 10 MiB `.crate` limit applies to the compressed tarball; measured ~2.5 MB fits comfortably.
2. Eigen (MPL-2.0) and any other upstream licenses inside `WebARKitLib` are compatible with LGPL-3.0 linking; vendored `COPYING.*` files will be preserved via `include = [...]`.
3. docs.rs will build `ffi-backend` successfully once C++ sources are present in the `.crate` (no network needed).
4. Maintainers will run `git submodule update --init --recursive` before `cargo publish`; CI will guard this.
5. The upstream `webarkit/WebARKitLib` repo URL is stable.

## Size measurements (recorded)

| Metric | Size |
|---|---|
| `lib/SRC/KPM/FreakMatcher/` + `include/`, uncompressed | ~11.5 MB |
| Same, compressed (zip Optimal ≈ gzip) | ~2.5 MB |
| crates.io `.crate` limit | 10 MiB |

Breakdown of FreakMatcher:
- `Eigen/` — 6.5 MB (header-only, transitively required)
- `unsupported/` — 4.1 MB (header-only Eigen modules)
- Detectors/matchers/framework/facade/etc. — ~550 KB
- Actually compiled: **12 `.cpp` files** (~200 KB)

---

## Decision Log

| # | Decision | Chosen | Alternatives | Rationale |
|---|----------|--------|--------------|-----------|
| 1 | How to deliver C++ sources on crates.io | Vendor via git submodule | Download-at-build; copy-vendored tree; upstream tarball per release | Works offline + on docs.rs; no duplicated bytes; reproducible |
| 2 | Submodule location | `crates/core/third_party/WebARKitLib` | `vendor/`, `cpp/` | `third_party/` unambiguous, no overlap with Cargo's vocabulary |
| 3 | Pinning policy | Commit SHA, `.gitmodules` tracks `master` | Always-latest, tag-based | Reproducible builds; easy `--remote` updates when desired |
| 4 | `libraries.json` schema | Add `branch` + optional `commit`; keep aligned with submodule SHA | Leave as-is | Prevents drift between C benchmark and Rust build |
| 5 | Publish payload | Explicit `include = [...]` limiting to `FreakMatcher/` + `include/` | Ship whole submodule | Smaller `.crate`; avoids shipping unused OSG/emscripten/test code |
| 6 | Old path in `build.rs` | Replace entirely | Keep as fallback | Single source of truth; avoids confusion |
| 7 | CI | Add job: init submodules + `cargo build --features ffi-backend` on 3 OSes + drift check | No CI change | Catches missing-submodule & drift regressions before release |
| 8 | README | Update clone instructions for `--recursive` + note Python-free Rust build | Skip | Contributors will hit this otherwise |

---

## Final Design

### 1. Repo layout

```
crates/core/
├── third_party/
│   └── WebARKitLib/         ← NEW submodule → webarkit/WebARKitLib @ <pinned-sha>
├── src/kpm/                 (unchanged)
├── build.rs                 (modified)
└── Cargo.toml               (modified: `include = [...]`)
.gitmodules                  ← NEW
```

### 2. `.gitmodules`

```ini
[submodule "crates/core/third_party/WebARKitLib"]
    path = crates/core/third_party/WebARKitLib
    url = https://github.com/webarkit/WebARKitLib.git
    branch = master
```

### 3. `build.rs` — minimal diff

Remove `workspace_root` computation; point `webarkitlib` at the new path; add a fail-fast check:

```rust
let webarkitlib = manifest_dir
    .join("third_party")
    .join("WebARKitLib");

let freak_matcher_root = webarkitlib.join("lib").join("SRC").join("KPM").join("FreakMatcher");
let include_root       = webarkitlib.join("include");

if !freak_matcher_root.exists() {
    panic!(
        "WebARKitLib C++ sources not found at {}. \
         Run `git submodule update --init --recursive` \
         (this is handled automatically for crates.io installs).",
        freak_matcher_root.display()
    );
}
```

File list, compiler flags, bindgen section, `rerun-if-changed` lines — unchanged.

### 4. `crates/core/Cargo.toml` — allowlist publish payload

Replace current `exclude` with:

```toml
include = [
    "src/**/*",
    "build.rs",
    "Cargo.toml",
    "README.md",
    "third_party/WebARKitLib/lib/SRC/KPM/FreakMatcher/**/*.cpp",
    "third_party/WebARKitLib/lib/SRC/KPM/FreakMatcher/**/*.h",
    "third_party/WebARKitLib/lib/SRC/KPM/FreakMatcher/**/*.hpp",
    "third_party/WebARKitLib/lib/SRC/KPM/FreakMatcher/**/*.inc",
    "third_party/WebARKitLib/include/**/*",
    "third_party/WebARKitLib/LICENSE*",
    "third_party/WebARKitLib/COPYING*",
    "third_party/WebARKitLib/lib/SRC/KPM/FreakMatcher/Eigen/COPYING*",
    "third_party/WebARKitLib/lib/SRC/KPM/FreakMatcher/unsupported/COPYING*",
]
```

Verify with `cargo package --list -p webarkitlib-rs --features ffi-backend` before merge.

### 5. `benchmarks/c_benchmark/libraries.json` — schema update

```json
{
  "name": "WebARKitLib",
  "source": {
    "type": "git",
    "url": "https://github.com/webarkit/WebARKitLib.git",
    "branch": "master",
    "commit": "<same-sha-as-submodule>"
  }
}
```

### 6. `benchmarks/bootstrap.py` — backwards-compatible field support

- If `commit` present → after clone, `git checkout <commit>`.
- If only `branch` present → clone with `--branch <branch>`.
- Neither → current default-branch behavior.

### 7. CI

New job (or extend existing) in `.github/workflows/`:

```yaml
ffi-backend-build:
  runs-on: ${{ matrix.os }}
  strategy:
    matrix:
      os: [ubuntu-latest, windows-latest, macos-latest]
  steps:
    - uses: actions/checkout@v4
      with:
        submodules: recursive
    - uses: dtolnay/rust-toolchain@stable
    - run: cargo build -p webarkitlib-rs --features ffi-backend
```

Linux-only drift check step:

```yaml
- name: Verify submodule SHA matches libraries.json commit
  run: |
    SUB=$(git -C crates/core/third_party/WebARKitLib rev-parse HEAD)
    JSON=$(jq -r '.[] | select(.name=="WebARKitLib") | .source.commit' \
           benchmarks/c_benchmark/libraries.json)
    test "$SUB" = "$JSON" || { echo "drift: submodule=$SUB json=$JSON"; exit 1; }
```

### 8. README updates

- Clone instructions mention `git clone --recursive` / `git submodule update --init --recursive`.
- Note under Bootstrap section: Python bootstrap only needed for the standalone C benchmark, not for the Rust `ffi-backend` build.

### 9. Error handling

| Scenario | Behavior |
|---|---|
| Missing submodule in local clone | `build.rs` panics with actionable message |
| Missing sources on crates.io install | Cannot happen (publish-time file presence + CI guard) |
| Upstream repo unreachable during `git submodule update` | Standard git error surfaces; not a `build.rs` concern |
| Submodule SHA drifts from `libraries.json` | CI drift-check job fails |

### 10. Testing strategy

**Pre-merge:**
1. `cargo clean && cargo build -p webarkitlib-rs --features ffi-backend` on Win/Linux/macOS.
2. `cargo package --list -p webarkitlib-rs --features ffi-backend` — inspect payload.
3. `cargo package` then unpack + build from the unpacked `.crate` — simulates crates.io consumer.
4. New CI jobs green on all three OSes.
5. Drift-check CI job green.

**Post-merge, pre-release:**
6. `cargo publish --dry-run`.
7. Real publish, consume from throwaway project for smoke test.

**Post-publish:**
8. Verify docs.rs build page green for `ffi-backend` on new version.

---

## Non-functional baseline (confirmed)

- Offline builds: **must work** — no network at build time.
- docs.rs: **must work** for `ffi-backend` feature.
- First-build time impact: negligible (no downloads; cc-rs compile time unchanged).
- Reproducibility: byte-identical output for a given submodule SHA.
- Target platforms: same as today (Win MSVC, Linux, macOS) — no new platform coverage intended.

## Rollout

Single PR, branched fresh from `dev` (per project rule: new branch per sub-issue). Includes:
- Submodule add (`.gitmodules` + pointer)
- `build.rs` repoint + fail-fast guard
- `Cargo.toml` `include = [...]`
- `libraries.json` schema change + `bootstrap.py` field support
- CI job + drift check
- README updates

**Not in this PR:** CHANGELOG edits (release-time only).
