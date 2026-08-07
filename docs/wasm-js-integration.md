# WebAssembly (JS/TS) integration guide

How to use [`@webarkit/webarkitlib-wasm`](https://www.npmjs.com/package/@webarkit/webarkitlib-wasm)
— the WebAssembly build of WebARKitLib-rs — from a browser app. The whole
pipeline runs the **pure-Rust** matcher; there is no C++ or Emscripten
dependency at runtime.

- [Install](#install)
- [Load and initialize the module](#load-and-initialize-the-module)
- [Standard vs SIMD build](#standard-vs-simd-build)
- [The three handles](#the-three-handles)
- [NFT: KPM detection → AR2 tracking](#nft-kpm-detection--ar2-tracking)
  - [1. Fetch the assets](#1-fetch-the-assets)
  - [2. Detect the initial pose (KPM)](#2-detect-the-initial-pose-kpm)
  - [3. Refine per frame (AR2 tracking)](#3-refine-per-frame-ar2-tracking)
  - [4. Full detect-then-track loop](#4-full-detect-then-track-loop)
- [Square marker detection](#square-marker-detection)
- [The pose matrix](#the-pose-matrix)
- [TypeScript](#typescript)
- [Memory management](#memory-management)
- [Common pitfalls](#common-pitfalls)

## Install

```bash
npm install @webarkit/webarkitlib-wasm
```

The package ships two pre-built engines and no build step is required. It is
an ES module (`"type": "module"`).

## Load and initialize the module

wasm-bindgen produces a default-export `init` function that fetches and
instantiates the `.wasm`. Call it once, then call `init_wasm()` to install the
console logger, before constructing any handle:

```js
import init, { init_wasm, get_version } from '@webarkit/webarkitlib-wasm';

await init();          // instantiate the wasm module
init_wasm();           // wire up console logging (optional but recommended)
console.log(get_version());
```

If you want Rust panics to surface as readable console errors during
development, also call `init_panic_hook()` after `init()`.

## Standard vs SIMD build

The package exposes two engines through its `exports` map:

| Import | Engine | When |
|--------|--------|------|
| `@webarkit/webarkitlib-wasm` | Standard (portable) | Default — works everywhere |
| `@webarkit/webarkitlib-wasm/simd` | WASM SIMD | Faster; needs a browser with WASM SIMD |

```js
// Standard
import init, { WasmKpmHandle } from '@webarkit/webarkitlib-wasm';
// SIMD (identical API)
import init, { WasmKpmHandle } from '@webarkit/webarkitlib-wasm/simd';
```

The two engines expose the **same API**, so you can feature-detect and pick
one at runtime:

```js
// Minimal WASM-SIMD probe (the canonical 8-byte "has SIMD" module).
const simdOk = WebAssembly.validate(new Uint8Array([
  0,97,115,109,1,0,0,0,1,5,1,96,0,1,123,3,2,1,0,10,10,1,8,0,65,0,253,15,253,98,11,
]));
const mod = simdOk
  ? await import('@webarkit/webarkitlib-wasm/simd')
  : await import('@webarkit/webarkitlib-wasm');
await mod.default();
mod.init_wasm();
```

## The three handles

| Handle | Purpose | Input marker data |
|--------|---------|-------------------|
| `WasmARHandle` | Square **pattern / barcode** marker detection + pose | `.patt` file, or matrix-code type |
| `WasmKpmHandle` | **NFT detection** — finds a natural-feature marker and returns its initial pose | `.fset3` |
| `WasmNFTHandle` | **NFT tracking** — refines a pose frame-to-frame via AR2 template matching | `.iset` + `.fset` |

For NFT you use `WasmKpmHandle` and `WasmNFTHandle` together: KPM finds the
marker cold, AR2 tracking keeps it locked cheaply once found. This mirrors the
native `simple_nft` example.

## NFT: KPM detection → AR2 tracking

### 1. Fetch the assets

You need the camera parameters and the marker set produced by
`nft_marker_gen` (`.fset3` + `.iset` + `.fset`). Load each as a `Uint8Array`:

```js
async function bytes(url) {
  const res = await fetch(url);
  return new Uint8Array(await res.arrayBuffer());
}

const cameraParam = await bytes('assets/camera_para.dat');
const fset3 = await bytes('assets/pinball.fset3');
const iset  = await bytes('assets/pinball.iset');
const fset  = await bytes('assets/pinball.fset');
```

### 2. Detect the initial pose (KPM)

Construct with the camera parameters and the **frame size** in pixels. The
constructor scales the camera intrinsics to that size for you (the
`arParamChangeSize` equivalent), so pass the dimensions of the frames you will
feed to `detect()`.

```js
import { WasmKpmHandle } from '@webarkit/webarkitlib-wasm';

const width = 640, height = 480;
const kpm = new WasmKpmHandle(cameraParam, width, height);
kpm.load_ref_data(fset3);        // KPM reference keypoints

// `rgba` is a width*height*4 buffer, e.g. from a canvas:
//   ctx.getImageData(0, 0, width, height).data
const result = kpm.detect(rgba);
if (result) {
  // result: { pose: number[12], page: number, error: number }
  console.log('found page', result.page, 'error', result.error);
}
```

`detect()` returns `null` when no marker is found, or
`{ pose, page, error }` where `pose` is the 3×4 camera pose flattened to 12
row-major floats. See [The pose matrix](#the-pose-matrix).

### 3. Refine per frame (AR2 tracking)

Feed the KPM pose into a `WasmNFTHandle` as the initial pose, then call
`track()` each frame:

```js
import { WasmNFTHandle } from '@webarkit/webarkitlib-wasm';

const nft = new WasmNFTHandle(cameraParam, width, height);
nft.load_nft_marker(iset, fset);
nft.set_initial_pose(result.pose);      // Float32Array or number[12] from KPM

const tracked = nft.track(rgba, width, height);
if (tracked.found) {
  // tracked: { found, matrix: number[12], error, cont_num }
  render(tracked.matrix);
}
```

### 4. Full detect-then-track loop

The usual pattern: run cheap AR2 tracking while it holds, and fall back to KPM
detection when tracking is lost (`found === false`, or `cont_num` resets to 0).

```js
let locked = false;

function onFrame(rgba) {
  if (locked) {
    const t = nft.track(rgba, width, height);
    if (t.found) { render(t.matrix); return; }
    locked = false;               // lost it — re-detect next frame
    nft.reset_tracking();
  }

  const d = kpm.detect(rgba);     // cold detect
  if (d) {
    nft.set_initial_pose(d.pose);
    locked = true;
    render(d.pose);
  }
}
```

## Square marker detection

For classic ARToolKit square markers (Hiro pattern, barcodes) use
`WasmARHandle` instead:

```js
import { WasmARHandle } from '@webarkit/webarkitlib-wasm';

const ar = new WasmARHandle(cameraParam);      // frame size taken from param
const pattId = ar.load_pattern(await (await fetch('assets/patt.hiro')).text());
ar.set_threshold(100);

const markers = ar.detect_markers(rgba, width, height, /* ...pixel format */);
// then ar.get_trans_mat(markerIdx, markerWidthMm) for a pose
```

`WasmARHandle` also supports matrix-code (barcode) detection via
`set_matrix_code_type(...)` — see the `simple_video_marker_example.html` demo.

## The pose matrix

Both `detect()` and `track()` return the pose as **12 floats = a 3×4
row-major camera pose matrix** `[R | t]`:

```text
[ m0  m1  m2  m3 ]      indices:  [ 0  1  2  3 ]
[ m4  m5  m6  m7 ]                [ 4  5  6  7 ]
[ m8  m9  m10 m11]                [ 8  9  10 11]
```

The upper-left 3×3 is rotation; the last column is translation in
millimetres. To drive a WebGL/Three.js/Babylon renderer, expand it to a 4×4
model-view matrix (append `[0,0,0,1]`) and apply your library's coordinate
convention. This is the same matrix layout the native API and jsartoolkitNFT
use, so existing conversion helpers apply.

## TypeScript

Type definitions ship with the package
(`dist-std/webarkitlib_wasm.d.ts`, referenced by `"types"` in
`package.json`), so imports are typed out of the box. The methods that return
serialized objects (`detect`, `track`, `get_trans_mat`) are typed as `any`
because they cross the wasm boundary as `JsValue`. Declare the shapes yourself
for safety:

```ts
interface KpmDetect { pose: number[]; page: number; error: number }
interface NftTrack  { found: boolean; matrix: number[]; error: number; cont_num: number }

const d = kpm.detect(rgba) as KpmDetect | null;
const t = nft.track(rgba, width, height) as NftTrack;
```

## Memory management

Each handle owns WebAssembly-side memory. wasm-bindgen generates a `free()`
method on every exported class — call it when you are done with a handle
(e.g. when unmounting a component or switching markers) so the memory is
released deterministically instead of waiting for GC:

```js
kpm.free();
nft.free();
```

Internal C-side resources (camera-parameter lookup tables, ICP handles) are
released by the Rust `Drop` implementations when the handle is freed.

## Common pitfalls

- **RGBA length must match exactly.** `detect()` and `track()` expect
  `width * height * 4` bytes and reject anything else. Feed canvas
  `ImageData.data` at the same dimensions you constructed the handle with.
- **Load before detect/track.** `detect()` errors if `load_ref_data()` hasn't
  run; `track()` errors if `load_nft_marker()` hasn't run. Check
  `is_loaded()` if unsure.
- **Single marker per handle.** `load_ref_data()` remaps all pages to page 0,
  so one `WasmKpmHandle` tracks one marker. Use multiple handles for multiple
  markers.
- **`track()` needs a prior pose.** Call `set_initial_pose()` (from KPM, or a
  previous frame) before the first `track()`; without an initial pose it
  returns `{ found: false }`.
- **Frame size is fixed per handle.** The constructor scales camera params to
  the size you pass. If your video resolution changes, build a new handle.

## See also

- [`simple_nft`](../crates/core/examples/simple_nft.rs) — the native
  equivalent of this detect-then-track flow.
- The `crates/wasm/www/` demos (`simple.html`,
  `simple_nft_example.html`, `simple_video_marker_example.html`) — runnable
  browser examples.
- [jsartoolkitNFT](https://github.com/webarkit/jsartoolkitNFT) — the
  Emscripten/C++ stack this pure-Rust port matches for pose output.
