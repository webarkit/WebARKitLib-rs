# `annotate_corners` — marker-corner annotation tool

Standalone static HTML tool for producing the `.corners.json` ground-truth
fixtures consumed by the absolute corner-error test gate (issue
[#166](https://github.com/webarkit/WebARKitLib-rs/issues/166)).

## What it does

Given a query frame (JPEG / PNG) that contains a printed NFT marker, you
click the **four printed-marker rectangle corners** in
`top-left → top-right → bottom-right → bottom-left` order, and the tool
writes a `.corners.json` file with the four pixel coordinates plus
metadata.

The order matters: it must match the reference image's corner ordering
(`(0,0), (W,0), (W,H), (0,H)`) so the CI corner-by-corner comparison
works.

## How to use

1. Open `index.html` directly in any modern browser. No server, no build
   step, no dependencies — it's a single static file.
2. Drag a query frame onto the canvas area (or use the file picker).
3. Click each marker corner in the prompted order. The next-expected
   corner is highlighted in the status panel. A live hover crosshair
   helps you line up the click.
4. After the 4th click, the tool draws a dashed-white quadrilateral
   connecting the four corners — verify it tightly follows the printed
   marker boundary before exporting.
5. Fill in your GitHub handle (`Annotator`), adjust `Tolerance (px)` if
   the frame warrants it (default 2.0 px matches the M9 #152 envelope),
   and add any `Notes` (lighting, occlusion, motion blur, etc.).
6. Click **Download JSON** to save `<image-name>.corners.json` to your
   Downloads folder, or **Copy JSON to clipboard** for paste-into-editor
   workflows.
7. Move both the image and the JSON into
   `crates/core/tests/fixtures/annotated_frames/`.

## Keyboard shortcuts

- `Ctrl/Cmd + Z` — undo last click
- `Escape` — clear all clicks (start over for current image)

## JSON schema produced

See issue #166 for the canonical schema. The tool produces:

```json
{
  "schema": 1,
  "image": "pinball-demo.jpg",
  "image_dims": [2000, 1500],
  "annotator": "kalwalt",
  "date": "2026-05-24",
  "marker_corners_px": [
    {"role": "top_left",     "x": 145.00, "y": 88.00},
    {"role": "top_right",    "x": 612.50, "y": 92.30},
    {"role": "bottom_right", "x": 605.20, "y": 815.40},
    {"role": "bottom_left",  "x": 152.10, "y": 819.80}
  ],
  "tolerance_px": 2.0,
  "notes": "Corners are the four printed-marker rectangle corners, in the same order as reference-image corners (0,0)/(W,0)/(W,H)/(0,H)."
}
```

## Caveats

- **Annotation noise floor** ~1–2 px even with zoom; do not set
  `tolerance_px` tighter than that.
- The browser's native page-zoom (`Ctrl+`/`Ctrl-`) works correctly —
  click coordinates always resolve to native image pixels.
- For very large images, scroll the canvas area; the page is laid out
  with vertical and horizontal overflow on the canvas container.
- The exported JSON's `image` field uses the file name only (no path);
  the CI test discovers fixtures by scanning the directory, so file
  names need to be unique.

## When to use which mode

- **Browser tool (this one)**: any frame where you can drag the JPEG
  onto the page. Recommended.
- **Manual via GIMP/Photoshop pointer readout**: zero tooling fallback
  if the browser tool is unavailable for any reason. Tedious; transcribe
  carefully and double-check ordering.

## References

- Issue [#166](https://github.com/webarkit/WebARKitLib-rs/issues/166) —
  the absolute corner-error gate this tool feeds.
- Issue [#160](https://github.com/webarkit/WebARKitLib-rs/issues/160) —
  the divergence investigation that motivated the new gate.
- `docs/design/m9-2-rust-backend.md` §10 — the §10 measurement that
  established the cross-backend gap.
