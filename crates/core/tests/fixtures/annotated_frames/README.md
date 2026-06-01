# `annotated_frames` — hand-annotated ground-truth fixtures

Ground-truth corner annotations for the **absolute corner-error
regression gate**, the M9 #166 Track A test
(`crates/core/tests/absolute_corner_error.rs`). Each frame is a JPEG
of a printed NFT marker (currently `pinball.fset3`'s marker) plus a
sibling `.corners.json` recording where the four printed-marker
corners actually lie in that frame, in `TL/TR/BR/BL` order.

The gate test reprojects the matched-scale reference corners through
each backend's homography into query-image pixel space and compares
against these annotations, asserting `current_max_err ≤ baseline + 0.5 px`
per backend per frame.

## What lives here

```
annotated_frames/
├── README.md                       (this file)
├── baseline.json                   (committed regression baseline — do not edit by hand)
├── pinball-seq1.jpg
├── pinball-seq1.corners.json
├── pinball-seq4.jpg
└── pinball-seq4.corners.json
```

Two matchable fixtures today. The test resolves each JSON's `image`
field by searching this directory first and falling back to
`crates/core/examples/Data/` (the original asset location).

### Why only two frames

The fixture set has been pared down twice for stability reasons,
each time backed by issue #170 ("matcher non-determinism"):

1. **`pinball-seq2.jpg` + `pinball-seq3.jpg`** were dropped after
   exposing **run-to-run nondeterminism in the Rust backend** on the
   same machine: between consecutive identical test runs, Rust's
   matched id (and therefore its homography) flipped unpredictably,
   while the C++ backend stayed stable. Most likely cause: Rust's
   default `HashMap` random hash state affecting BHC tree topology.

2. **`pinball-demo.jpg`** was dropped after CI exposed
   **cross-platform divergence in the C++ backend**: on Windows
   (locally) C++ matched `db_id=2` (595×745); on Ubuntu CI
   (libstdc++) C++ matched `db_id=1` (750×938). This is the same
   `unordered_map` iteration-order mechanism §10 of
   `docs/design/m9-2-rust-backend.md` discusses for Rust↔C++
   variance — turns out C++↔C++ also varies across stdlib
   implementations. The `.jpg` stays in `crates/core/examples/Data/`
   for its example role; only the annotation here was removed.

`pinball-seq1.jpg` is rock-solid (sub-pixel agreement across
platforms). `pinball-seq4.jpg` has ~1.8 px of Rust-side per-platform
drift but matches the same id consistently. Both are absorbed by the
2.0 px regression epsilon (see `REGRESSION_EPSILON_PX` in
`absolute_corner_error.rs` for the full rationale).

The dropped fixtures will be re-added once #170 lands cross-platform
determinism in both backends.

## Schema reference

See [`tools/annotate_corners/README.md`](../../../../../tools/annotate_corners/README.md)
for the full `.corners.json` schema. The short version:

```json
{
  "schema": 1,
  "image": "pinball-seq1.jpg",
  "image_dims": [2000, 1500],
  "annotator": "kalwalt",
  "date": "2026-05-31",
  "marker_corners_px": [
    {"role": "top_left",     "x": ..., "y": ...},
    {"role": "top_right",    "x": ..., "y": ...},
    {"role": "bottom_right", "x": ..., "y": ...},
    {"role": "bottom_left",  "x": ..., "y": ...}
  ],
  "tolerance_px": 2.0,
  "notes": "..."
}
```

`marker_corners_px` must be in `TL/TR/BR/BL` order (matching the
reference image's corner ordering `(0,0)/(W,0)/(W,H)/(0,H)`).

## Adding a new annotated frame

The test discovers fixtures by globbing `*.corners.json`, so adding a
new frame requires **no code change** — just files.

### 1. Capture the photo

For best results:

- **Sharp focus on the marker** — tap-to-focus on a phone, half-press
  shutter on a real camera. The current `pinball-seq{1..4}.jpg`
  fixtures show what *not* to do (out of focus → matcher returns
  `matched_id = -1`).
- **Fill 40–80% of the frame** with the marker. Below ~30% the
  matcher starts to struggle even at the smallest pyramid level.
- **Hold still** to avoid motion blur.
- **Good contrast** — daylight or a desk lamp angled at the marker.

### 2. Annotate it

Open [`tools/annotate_corners/index.html`](../../../../../tools/annotate_corners/index.html)
directly in any modern browser, drag your new JPEG onto the canvas,
click the four printed-marker corners in `TL → TR → BR → BL` order,
verify the dashed-white quad tightly follows the printed boundary,
then click **Download JSON**.

If you need to nudge a single corner after the 4th click, click that
corner's row in the side panel and the next canvas click repositions
it. Wheel-zoom over the canvas helps for sub-pixel accuracy.

### 3. Drop both files into this directory

```
crates/core/tests/fixtures/annotated_frames/
├── pinball-seq5.jpg              ← new image
└── pinball-seq5.corners.json     ← new annotation
```

### 4. Regenerate the baseline

```sh
WEBARKIT_REGEN_CORNER_BASELINE=1 \
  cargo test --test absolute_corner_error --features dual-mode -- --nocapture
```

This writes a refreshed `baseline.json` capturing the per-backend
numbers for **all** fixtures (existing + new). Inspect the printed
table — the new frame's row should show plausible max-err values
(typically 1–20 px for a sane annotation). If you see hundreds of
pixels, the annotation's corner order is probably wrong or the
matched_id is mismatched.

### 5. Verify the gate passes in normal mode

```sh
cargo test --test absolute_corner_error --features dual-mode
```

### 6. Commit

```sh
git add pinball-seq5.jpg pinball-seq5.corners.json baseline.json
git commit -m "test(fixtures): add pinball-seq5 (refs #166)"
```

Mention the new measurements (matched_id + per-backend max err) in
the PR body so reviewers can see what's being gated.

## Replacing an existing fixture

Same workflow, but step 3 overwrites both files in place. The test's
"started matching" / "stopped matching" branches will loudly flag the
status transition (e.g. a previously-no-match fixture that now
matches), and the regen step locks in the new numbers.

## Removing a fixture

Just delete both the `.jpg` and the `.corners.json`. Then regenerate
the baseline (step 4) — the removed entry will be dropped from
`baseline.json`. Commit all three deletions together so the baseline
stays in sync with the fixture set.

## When NOT to regenerate the baseline

Regen rewrites `baseline.json` from current measurements. **Do not
regen** if your PR:

- Touches the matcher pipeline and changes the numbers — that's
  *exactly* what the gate is designed to flag, regen would mask it.
- Lands a "make it faster but slightly less accurate" change without
  an explicit reviewer decision to accept the loss.

Regen is appropriate when:

- Adding / removing / replacing fixtures (the cases above).
- Landing a deliberate matcher *improvement* that produces tighter
  numbers — the test prints an "improvement detected, regenerate"
  note when it sees one. Capturing the improvement into the baseline
  prevents future regressions from being measured against a stale
  floor.

## Exploratory testing without committing

If you just want to *see* what a new photo looks like through both
backends without going through the full annotation + baseline cycle,
the `simple_nft_dual` example is the faster path:

1. Drop the JPEG in `crates/core/examples/Data/` (any name).
2. Temporarily change the `marker_name` / image path in
   `crates/core/examples/simple_nft_dual.rs`.
3. `cargo run --features "dual-mode log-helpers" --example simple_nft_dual`.
4. Inspect the two PNGs that drop into `target/simple_nft_dual_output/`
   (the C++ and Rust marker outlines drawn in blue over the query
   frame).

If the photo looks promising, *then* annotate it and add it as a
proper fixture.

## References

- [issue #166](https://github.com/webarkit/WebARKitLib-rs/issues/166) — the gate's design
- [issue #160](https://github.com/webarkit/WebARKitLib-rs/issues/160) — the divergence investigation
- [`tools/annotate_corners/`](../../../../../tools/annotate_corners/) — the HTML annotator
- [`crates/core/tests/absolute_corner_error.rs`](../absolute_corner_error.rs) — the test that consumes these fixtures
