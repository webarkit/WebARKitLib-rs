# Issue #103 — Fix Plan: Low CF from Template Matching

**Status**: PR-A diagnostic landed; root cause identified; PR-B scope revised (see §9)
**Tracking issue**: [webarkit/WebARKitLib-rs#103](https://github.com/webarkit/WebARKitLib-rs/issues/103)
**Scope**: issue #103 points 1 (pattern extraction sampling) and 2 (pixel-format vs. detection-mode mismatch)
**Out of scope**: issue #103 points 3 (`patt_ratio`) and 4 (`patt.hiro` / `ar_patt_load`)

---

## 1. Understanding Summary

- **Problem**: `ar_detect_marker` returns `cf ≈ 0.0965` on the standard hiro pattern in the simple example, well below the 0.5 cutoff, so the marker is rejected. Other ARToolKit5-based detectors (artoolkit5.js, AR.js) report `cf ≥ 0.7` on the same input.
- **Diagnosis (so far)**: the geometric pipeline is correct (`area`, `pos`, `dir` come out right). The divergence is isolated to the pattern-identity stage. The maintainer's audit on PR #102 flags `ar_patt_get_image` (~210 Rust lines vs. ~550 C lines at `arPattGetID.c:289-840`) and `pattern_match` (`pattern.rs:339-501`) as partial ports.
- **Audience**: contributors implementing the fix; reviewers of PR-A and PR-B.
- **Key constraint**: 1:1 with the C reference. No smoothing, no refactoring beyond the port. This mirrors the style of the recently-merged tracking-history port (#96 / PR #102).
- **Existing infrastructure**: [`benchmarks/c_benchmark/main.c`](../benchmarks/c_benchmark/main.c) builds via CMake against ARToolKit5, loads `patt.hiro` and a raw luma buffer, and runs `arDetectMarker`. Suitable as a ground-truth harness without modification.
- **Active code path in the simple example**: `AR_PIXEL_FORMAT_MONO` source × `AR_TEMPLATE_MATCHING_COLOR` detection mode. This is the same combination the C benchmark uses, so the example wiring matches the reference; the bug is in our code, not the test setup.

## 2. Assumptions

| # | Assumption |
|---:|---|
| A1 | The C benchmark can be built locally from the existing CMake project. New C code is acceptable to land in this repo. |
| A2 | Diagnostic dumps land at fixed paths under `benchmarks/data/`. No CI integration needed for PR-A — local-only. |
| A3 | Numerical tolerance for the patch is exact byte equality. cf parity to `1e-6` is a stretch goal, not a blocker. |
| A4 | Diagnostic code is gated (cargo example / separate C target) so it doesn't ship to library consumers. |
| A5 | The current Rust output (`cf=0.0965`, `id=-1`) is reproducible run-to-run on the same inputs. To be verified as the first diagnostic step. |
| A6 | The diagnostic stays in the repo permanently after PR-B lands — useful regression scaffolding. |

## 3. Acceptance Criteria

**PR-A** (diagnostic):
- C `dump_patt` and Rust `dump_patt` both produce a binary blob and a meta sidecar for the same input.
- `diff_patt.rs` reports byte-level divergence with first-N divergent indices.
- README documents the recipe end-to-end.

**PR-B** (fix):
- The simple example reports `cf ≥ 0.7` and `Matched ID: 0` on `benchmarks/data/img.jpg`.
- The Rust `ext_patt` buffer is **byte-for-byte identical** to the C reference `ext_patt` for the same vertices.
- A new integration test under `crates/core/tests/patt_extraction_parity.rs` locks the parity in (loads the C-dumped fixture, asserts byte-equal extraction).
- `cargo fmt`, `cargo clippy --lib -- -D warnings`, `cargo test --lib` all clean.
- cf parity to `1e-6` against the C reference: stretch goal, captured as a follow-up if not achieved.

## 4. Decision Log

| # | Decision | Alternatives considered | Rationale |
|---:|---|---|---|
| 1 | Use the C reference (`benchmarks/c_benchmark/`) as ground truth | artoolkit5.js / AR.js comparison; synthetic test only | 1:1-with-C constraint; bench already exists. Synthetic stays as PR-B regression scaffolding. |
| 2 | Medium instrumentation: dump the extracted patch + cf | Coarse cf-only; fine-grain (every internal signal) | Cheapest level that localizes extraction-vs-matching. Fine-grain only if patches match. |
| 3 | Two-PR shape (diagnostic-first, then fix) | One big PR; one PR per fix layer | Cheap PR-A review; PR-B can iterate freely; diagnostic stays as permanent tooling. |
| 4 | Acceptance bar B = patch byte-equal + functional cf ≥ 0.7 | A: functional only; C: cf parity 1e-6 | Patch is the meaningful interface; cf parity follows once arithmetic is identical. |
| 5 | Sibling C `dump_patt.c`, leave `main.c` untouched | Modify `main.c` | User preference; preserves the timing benchmark. |
| 6 | Sibling Rust `examples/dump_patt.rs`, leave `simple.rs` untouched | Feature-gate `simple.rs` | Symmetric with C choice; demo stays clean. |
| 7 | Raw bytes blob + plain-text meta sidecar | Header-prefixed blob; JSON | Trivial cross-language read; metadata stays human-inspectable. |
| 8 | "Example's path first" in PR-B; defer other format/mode permutations | Full pixel-format expansion up-front | Smaller, reviewable PR; unblocks the failing demo first. |
| 9 | `pattern_match` fix folded into PR-B vs split into PR-C: deferred | Pre-commit to PR-C; pre-commit to PR-B | Decision depends on diagnostic results — judge by size. |

## 5. Final Design — PR-A (Diagnostic)

### 5.1 Files added

| File | ~LoC | Purpose |
|---|---:|---|
| `benchmarks/c_benchmark/dump_patt.c` | ~120 | Single-shot detection + cf print + binary dump of `ext_patt` (sibling of `main.c`) |
| `benchmarks/c_benchmark/CMakeLists.txt` | +5 | Add second `add_executable(dump_patt …)` target |
| `crates/core/examples/dump_patt.rs` | ~110 | Rust counterpart: load image, run pipeline, dump `ext_patt` |
| `crates/core/examples/diff_patt.rs` | ~50 | Load both `.bin` files, byte-diff, print divergence summary |
| `benchmarks/data/README.md` | ~30 | One-page recipe: build C dumper, run both, run diff |

### 5.2 Binary blob format

`benchmarks/data/{c,rs}_ext_patt.bin` contains **only the raw `ext_patt` bytes** — no header, no metadata. Length is `patt_size² × channels` (16² × 3 = 768 in the default color-mode case).

Companion `benchmarks/data/{c,rs}_ext_patt_meta.txt`:

```
patt_size = 16
channels  = 3
mode      = AR_TEMPLATE_MATCHING_COLOR
pixfmt    = AR_PIXEL_FORMAT_MONO
xsize     = 429
ysize     = 317
vertex[0] = (X, Y)
vertex[1] = (X, Y)
vertex[2] = (X, Y)
vertex[3] = (X, Y)
cf        = 0.7531
id        = 0
dir       = 2
```

### 5.3 Vertex synchronisation

The C side detects vertices and prints them. The Rust side accepts those vertices via env var (`WEBARK_DUMP_VERTICES="x0,y0;x1,y1;x2,y2;x3,y3"`) or, if absent, runs its own detection and prints them. The user pastes the C vertices into the env var on the Rust run, eliminating geometric FP drift as a confounder. `diff_patt.rs` warns if the `*_meta.txt` files disagree on vertex coordinates.

### 5.4 Comparison output (`diff_patt.rs`)

```
patt_size: 16  channels: 3  expected: 768 bytes
c side:    benchmarks/data/c_ext_patt.bin   (768 bytes)
rs side:   benchmarks/data/rs_ext_patt.bin  (768 bytes)
identical: 0/768 bytes
divergent: 768 / 768  (100.00%)
first 8 divergences: [(0, c=0xfe, rs=0x12), …]
max abs delta: 244
```

If buffers match: `identical: 768/768 — patches are byte-equal.`

## 6. Final Design — PR-B (Fix)

### 6.1 Step 0 — Read the diagnostic

Classify divergence:

| Signal | Likely root | Action |
|---|---|---|
| Patches byte-equal, cf still differs | `pattern_match` arithmetic, or `ar_patt_load` (out of scope) | Re-scope; may need a separate brainstorm |
| Statistical drift | Wrong averaging arithmetic (luma weighting, total_div, integer truncation) | Fix `ar_patt_get_image` per-sample arithmetic |
| Structural drift | Sampling-grid bug (`xdiv`/`ydiv` doubling, output indexing, perspective) | Fix `ar_patt_get_image` sampling loop |
| Channel-permuted | BGR/RGB storage convention mismatch | One-line swap in color branch writes |
| Catastrophic | Severe index math or perspective bug | Likely a full re-port of `ar_patt_get_image` for the active path |

### 6.2 Step 1 — Fix order (active path: `MONO` source × `COLOR` detection)

1. **Sampling-grid math.** Port `arPattGetID.c:289-840` 1:1 for the active branch. Likely culprits in order: per-output-pixel division by `total_div = xdiv * ydiv` (wrong when `xdiv2 % patt_size ≠ 0`), the `xdiv2`/`ydiv2` doubling-loop bounds, the FieldImage rounding in `xc/yc`. The `(255 - data[i])` inversion lives in `pattern_match` — leave it.
2. **Channel-order verification.** Once samples land in the correct output cells, audit whether the BGR-write convention matches C. Cheap synthetic-input check.
3. **Pixel-format expansion** (deferred). BGR, BGRA, ABGR, ARGB to be filled in *after* the active path is byte-equal. RGB565, 2vuy, yuvs may stay `Err(...)` as documented stubs.
4. **`pattern_match`** — touch only if Step 0 surfaces persistent divergence after extraction is byte-equal. PR-C/fold-in decision is deferred per Decision #9.

### 6.3 Step 2 — Tests added in PR-B

- **`crates/core/tests/patt_extraction_parity.rs`** — loads `benchmarks/data/c_ext_patt.bin` (committed as a fixture) and asserts byte-equality with a fresh Rust `ar_patt_get_image` call on the same vertices.
- **Pixel-format coverage unit tests** in `pattern.rs` for any newly-added format. Synthetic inputs, just enough to catch typos.
- All existing marker / pattern / matrix tests continue to pass.

## 7. Risks

### PR-A

| ID | Risk | Mitigation |
|---:|---|---|
| R-A1 | C build deps unavailable on dev machine | If `main.c` builds today, `dump_patt.c` will too (no new external deps) |
| R-A2 | Vertex drift between C and Rust → false positive in patch diff | C exports vertices; Rust accepts them via env var; comparison isolates extraction |
| R-A3 | Patch trivially zero (out-of-image clip) → meaningless byte-equality | `diff_patt.rs` asserts non-zero variance before declaring parity |
| R-A4 | Dump captures wrong stage (post-inversion in `pattern_match`) | Explicit anchoring comment at `ar_patt_get_image` return; dump happens before `pattern_match` is called |

### PR-B

| ID | Risk | Mitigation |
|---:|---|---|
| R-B1 | 1:1 port adds ~340 lines | "Example's path first" scoping (Decision #8) — port only the active branch; defer other format/mode permutations |
| R-B2 | Bug actually in `ar_patt_load` (out of scope) | PR-A diagnostic surfaces this directly: byte-equal patches + cf divergence ⇒ escalate to a separate issue |
| R-B3 | Rust/C floating-point rounding diverges sub-LSB | `u8` integer division after extraction absorbs FP noise; cf parity to `1e-6` is stretch, not blocker |
| R-B4 | Hidden coupling with `pattern_match` (extraction fix exposes a dormant matcher bug) | Decision #9 already in place — judge by diagnostic + size, decide fold-in vs PR-C |

## 9. Findings — PR-A smoke run (2026-05-02)

The Rust-side `dump_patt` was run end-to-end against `benchmarks/data/img.jpg`
+ `patt.hiro`. **It produced `cf = 0.892460`, `id = 0`, `dir = 1`** — well above
the 0.7 acceptance bar — using the same library code path as `simple.rs`.

The only difference between `dump_patt.rs` and `simple.rs` is the
`AR2VideoBufferT` setup:

- `simple.rs` declares `pixfmt = MONO` but feeds an **RGBA** buffer into
  `AR2VideoBufferT.buff` (and a luma buffer into `buff_luma`).
- `dump_patt.rs` declares `pixfmt = MONO` and feeds the **luma** buffer into
  both `buff` and `buff_luma`.

Inside `ar_get_marker_info`, `ar_patt_get_image` indexes `buff` according to
the declared pixel format. With `pixfmt = MONO`, the MONO branch reads
`image[yc * xsize + xc]` (one byte per pixel). When `simple.rs` passes a
4-bytes-per-pixel RGBA buffer there, the sampler reads the wrong bytes and
produces nonsense correlation — hence the original `cf ≈ 0.0965`.

**Independent confirmation**: ARToolKit5's own
[`video2.c:682`](https://github.com/webarkit/artoolkit5/blob/66aa1cc12e1bdeb12ee6af5746dc4ff6f3ba34cb/lib/SRC/Video/video2.c#L682)
documents the contract verbatim:

```c
if (pixFormat == AR_PIXEL_FORMAT_MONO || /* ... */) {
    ret->buffLuma = ret->buff;   // when MONO, buff IS the luma data
} else {
    /* otherwise compute buffLuma from buff via arVideoLuma() */
}
```

So `buff` MUST hold data in the format declared by `arPixelFormat`. `simple.rs`
violates that contract.

### What this changes

- **R-B2 (the bug is in scope #4) is ruled out.** The library produces the
  expected cf when fed a buffer that matches its declared `pixfmt`.
- **PR-B's scope shrinks dramatically.** Instead of porting ~340 more lines of
  `arPattGetID.c`, the actual fix is small:
  1. Fix `simple.rs` to feed `buff` data matching the declared `pixfmt`
     (either pass luma when `pixfmt = MONO`, or set `pixfmt = RGBA` and pass
     RGBA).
  2. Add a defensive sanity check inside `ar_detect_marker` that compares
     `frame.buff.len()` to `xsize * ysize * pixel_size_for(pixfmt)` and
     `arlog_e!`s on mismatch — turns this kind of contract violation into a
     loud, early failure instead of a silent low-cf result.
  3. (Optional) Document the contract in the `ar_detect_marker` doc comment
     and link to `video2.c:682`.
- **The byte-equality-with-C check is left to the user** (the C bench source
  needs the bootstrap that lives on the main checkout, not in this worktree).
  Recommended path: from the main checkout, run
  `cargo run --release --example dump_patt --features log-helpers`, then
  `cmake --build benchmarks/c_benchmark/build --target dump_patt && ./build/dump_patt … ../data`,
  then `cargo run --release --example diff_patt --features log-helpers`.
  This closes R-A1 (no surprise behavioural divergence between Rust and C
  extraction) before PR-B lands.

### Updated Decision Log entries

| # | Decision | Why |
|---:|---|---|
| 10 | Pivot PR-B from "port `ar_patt_get_image` 1:1 with C" to "fix `simple.rs` frame contract + add early-failure check" | Smoke run + `video2.c:682` proves the library extraction is correct; the original bug surface in #103 is the example, not the port. |
| 11 | Defer C-side byte-equality verification to the user's main checkout (not the worktree) | Bootstrap deps not present in worktree; no need to block PR-A on it. The cf=0.89 result is strong enough evidence to proceed. |

## 10. Open Items / Follow-ups (out of this plan)

- Issue #103 point 3 — `patt_ratio` correctness vs. the value used by `mk_patt`. Separate brainstorm.
- Issue #103 point 4 — `patt.hiro` / `ar_patt_load` byte-order, header parsing, channel ordering. Separate brainstorm.
- Auto-thresholding (BRACKETING / ADAPTIVE / MEDIAN / OTSU) — pre-existing TODO at `crates/core/src/ar/marker.rs:124`. Unrelated.
- Eventual full pixel-format coverage in `ar_patt_get_image` (BGR, BGRA, ABGR, ARGB, RGB565, 2vuy, yuvs) — incremental, post-PR-B. Now lower priority since the active path is verified correct.
- Eventual CI parity test (Approach 2 from the brainstorm) — adds ARToolKit5 build deps to CI; defer until parity is locally trusted.
- PR-B (revised): fix `simple.rs` frame contract + add `ar_detect_marker` sanity check + doc-comment the contract referencing `video2.c:682`.
