# Issue #215 Design Doc — WASM WebWorker Live Video NFT Pipeline & Camera Intrinsics Alignment

**Status**: Implemented & Approved  
**Branch**: `feat/wasm-video-nft-215`  
**Parent Issue**: [#215](https://github.com/webarkit/WebARKitLib-rs/issues/215)  
**Author**: Walter Perdan ([@kalwalt](https://github.com/kalwalt))  
**Date**: 2026-08-23  

---

## 1. Problem Statement & Objectives

Goal 5 of Issue [#215](https://github.com/webarkit/WebARKitLib-rs/issues/215) targets creating a full real-time Natural Feature Tracking (NFT) WebAssembly demonstration (`simple_video_nft_example.html`) that tracks target markers in live camera video streams.

### Key Objectives & User Feedback Addressed:
1. **Performance**: Heavy WASM keypoint feature matching (KPM FREAK) and AR2 template tracking must run on a background thread to prevent UI freezing and maintain **60+ FPS main-thread UI rendering**.
2. **2D Bounding Quad & Pose Alignment**: Fix target corner ordering and 3D coordinate space origin projection ($p0 = (0,0,0)$ at Bottom-Left, $p1 = (W_{\text{mm}},0,0)$ at Bottom-Right).
3. **Camera Intrinsics API**: Expose native WASM camera intrinsics (`[fx, fy, cx, cy]`) and $4 \times 4$ OpenGL projection matrix matching `jsartoolkitNFT`'s `threejs_worker.js`.
4. **Static Image vs. Live Webcam FOV Lens Compensation**: Account for field-of-view differences between static calibration files (`camera_para.dat` narrow ~55° FOV) and physical live webcams (~61°–72° FOV).

---

## 2. Architecture & Implementation

### 2.1 Dedicated WebWorker Pipeline (`crates/wasm/www/nft.worker.js`)
- Dynamically imports `webarkitlib_wasm.js` inside a background ES module WebWorker.
- Manages state machine: `SEARCHING` (throttled KPM detection) $\leftrightarrow$ `TRACKING` (per-frame AR2 template tracking).
- Transfers RGBA pixel buffers using zero-copy `ArrayBuffer` transferables (`postMessage(msg, [buffer])`), eliminating main-thread serialization overhead.

### 2.2 WASM Projection Matrix & Camera Intrinsics API (`crates/wasm/src/lib.rs`)
Added `#[wasm_bindgen]` export methods to `WasmNFTHandle` and `WasmKpmHandle`:
- `get_camera_intrinsics() -> Result<Box<[f32]>, JsValue>`: Returns `[fx, fy, cx, cy]` directly from `camera_para.dat`.
- `get_projection_matrix(near: f32, far: f32) -> Result<Box<[f32]>, JsValue>`: Computes the column-major $4 \times 4$ OpenGL camera projection matrix (16 floats) matching `arglCameraFrustumRH` (`lib/SRC/AR/paramGL.c`).

Both are thin wrappers over the free functions `intrinsics_from` / `projection_from`, so the
math lives in exactly one place and is shared by `WasmNFTHandle` and `WasmKpmHandle`. Both
return `Err` rather than a zero-filled buffer when no camera parameters are loaded, and
`get_projection_matrix` additionally rejects any frustum that is not `0 < near < far` — the
old form emitted `inf`/`NaN` straight into the render path when `far == near`.

```rust
fn projection_from(param: &ARParam, near: f32, far: f32) -> Result<Box<[f32]>, String> {
    // ... validation of near/far and frame size ...
    let w = (param.xsize - 1) as f32;
    let h = (param.ysize - 1) as f32;
    let mut proj = vec![0.0f32; 16];
    proj[0] = 2.0 * fx / w;
    proj[4] = 2.0 * skew / w;
    proj[5] = 2.0 * fy / h;
    proj[8] = 1.0 - (2.0 * cx / w);   // negated w.r.t. the naive OpenCV -> GL form
    proj[9] = (2.0 * cy / h) - 1.0;
    proj[10] = -(far + near) / (far - near);
    proj[11] = -1.0;
    proj[14] = -(2.0 * far * near) / (far - near);
    Ok(proj.into_boxed_slice())
}
```

**Sign conventions** follow `arglCameraFrustumRH` exactly:

- The C code flips the image y-axis (`icpara[1][i] = (h-1)*icpara[2][i] - icpara[1][i]`) and
  then negates the row. For the focal term the two negations cancel, so `m[5]` is
  $+2 f_y / (h-1)$; for the centre term they cancel too, giving $m[9] = 2 c_y / (h-1) - 1$.
- `m[8]` is $1 - 2 c_x / (w-1)$, i.e. **negated** relative to the naive form. This is invisible
  for a centred principal point and only shows up once $c_x$ is off-centre — see the
  `projection_x_shift_is_negated_relative_to_y` unit test, which is the only configuration that
  distinguishes the two conventions.
- Denominators are $w-1$ / $h-1$, not $w$ / $h$, matching the C.
- The `arParamDecompMat` step is skipped: `ARParam::load` yields a matrix whose extrinsic part
  is the identity, so `icpara == mat` and the trailing `q * trans` multiply reduces to `q`.

### 2.2.1 Isotropic Camera Parameter Scaling (`scale_param_isotropic`)

`arParamChangeSize` scales rows 0 and 1 anamorphically by $(s_x, s_y)$. That breaks
$f_x = f_y$ the moment the requested frame aspect differs from the calibration aspect
(a 16:9 webcam stream against a 4:3 `camera_para.dat`), so the focal lengths are instead
scaled **isotropically** by the height ratio.

This assumes the wider frame is a horizontal **field-of-view extension** of the calibration,
not a vertical crop — the extra width adds scene rather than stretching it. The principal
point is therefore carried across proportionally in each axis independently.

Column 3 (the translation terms) is scaled along with the rest of its row, as
`arParamChangeSize` does for `col in 0..4`: those terms are non-zero whenever the calibration
encodes a camera offset, and silently dropping them skews every pose.

### 2.3 Camera Field-of-View (FOV) Lens Calculation (`simple_video_nft_example.html`)
To bridge the gap between static test images (captured on narrow calibrated lenses) and live webcams:
- **Pinhole Camera FOV Formula**:
  $$f_x = \frac{W}{2 \cdot \tan\left(\frac{\theta_{\text{FOV}}}{2}\right)}$$
- **Camera Lens Preset Selector**:
  - `Auto / Standard Webcam (~61° FOV)`: $f_x = \text{canvasWidth} / (2 \cdot \tan(30.5^\circ)) \approx 543 \text{ px}$ (Matches physical USB / laptop webcams).
  - `Laptop Wide Angle (~72° FOV)`: $f_x = \text{canvasWidth} / (2 \cdot \tan(36^\circ)) \approx 440 \text{ px}$.
  - `Calibrated File (camera_para.dat ~55° FOV)`: Uses exact $f_x = 609.4$ from `camera_para.dat` (Matches `pinball-demo.jpg` static test image 100%).

### 2.4 Webcam Resolution Presets & Soft Constraints
To support modern 16:9 webcam streams with smooth cross-device fallback:
- **Resolution Selector**:
  - `1280x720` — **1280×720 (HD 16:9, Default)**: Modern high-definition standard.
  - `1920x1080` — **1920×1080 (FHD 16:9)**: Full HD widescreen.
  - `640x480` — **640×480 (VGA 4:3)**: Classic legacy aspect ratio.
  - `640x360` — **640×360 (SD 16:9)**: Low-bandwidth mobile widescreen.
  - `auto` — **Auto / Native Default**: Browser-negotiated default resolution.
- **`getUserMedia` Soft Constraints**: Employs `{ width: { ideal: W }, height: { ideal: H } }` so the browser gracefully falls back to the nearest supported mode on constrained hardware without throwing exceptions.
- **Dynamic Form Management**: Automatically disables resolution settings when "Static Test Image" is active or while pipeline tracking is in progress.

## 3. The "Right Edge Overshoot" Root Cause, Mathematics & Verification

During testing of the live video NFT example, we investigated a subtle visual divergence: while the 3D bounding box superimposed with 100% precision in "Static Test Image" mode (`pinball-demo.jpg`), in "Live Camera" mode tracking a printed sheet of paper, the green bounding quad's right edge overshot past the right boundary of the physical paper.

### 3.1 Digital Marker Dimensions & DPI Mathematics
When an NFT dataset (`.iset`, `.fset`, `.fset3`) is generated from a source image, ARToolKit stores the base scale resolution and the target DPI:
- **Base Digital Resolution**: $893 \times 1117\text{ px}$
- **Target DPI**: $120\text{ DPI}$

From these values, the physical world metric dimensions in 3D space ($Z = 0$ marker plane) are derived:
$$W_{\text{mm}} = \frac{893\text{ px}}{120\text{ DPI}} \times 25.4\text{ mm/inch} = \mathbf{189.02\text{ mm}}$$
$$H_{\text{mm}} = \frac{1117\text{ px}}{120\text{ DPI}} \times 25.4\text{ mm/inch} = \mathbf{236.43\text{ mm}}$$

The inherent digital aspect ratio is:
$$\text{Aspect Ratio}_{\text{digital}} = \frac{893}{1117} = \mathbf{0.7995} \approx 0.80\text{ (4:5 ratio)}$$

### 3.2 The Paper Printout Distortion (A4 Aspect Ratio Trap)
Standard ISO 216 **A4 Paper** ($210\text{ mm} \times 297\text{ mm}$) has a geometric aspect ratio of:
$$\text{Aspect Ratio}_{\text{A4}} = \frac{210}{297} = \frac{1}{\sqrt{2}} \approx \mathbf{0.7071}$$

| Medium | Aspect Ratio ($W : H$) | Geometric Characteristics |
| :--- | :--- | :--- |
| **Digital Marker (`pinball`)** | **$0.7995$** ($\sim 4:5$) | Wider proportion |
| **Physical A4 Paper** | **$0.7071$** ($\sim 1:\sqrt{2}$) | Taller, narrower proportion |

When printing an image using standard operating system print dialogs (Windows Photo Viewer, macOS Preview, mobile print managers):
- The default option **"Fit picture to frame" / "Fit to Page"** scales the image to match the full height of the printable page area ($297\text{ mm}$).
- To accommodate margins without manual clipping, the printer driver compresses (squishes) the horizontal axis non-uniformly by $\approx 12\% - 15\%$.
- The physical printed image on the paper ends up measuring only $\sim 165\text{ mm}$ wide instead of the true $189.02\text{ mm}$ expected by the digital dataset.

### 3.3 Computer Vision Pipeline Reaction
1. **Feature Matching**: ARToolKit's KPM (FREAK descriptors) and AR2 template trackers match local gradient patterns (bumpers, flippers, letters) invariant to scale and slight affine deformation.
2. **Pose Estimation**: The PnP pose estimator aligns the coordinate origin $(0,0,0)$ with the bottom-left visual features of the marker ($X=0, Y=0$).
3. **Corner Projection**: The 3D bounding box projects the four physical corners:
   $$P_0 = (0, 0, 0), \quad P_1 = (189.02, 0, 0), \quad P_2 = (189.02, 236.43, 0), \quad P_3 = (0, 236.43, 0)$$
4. **The Overshoot**: Because the physical paper on the desk was compressed to $\sim 165\text{ mm}$, the projected right corners ($P_1, P_2$) land at the mathematically correct digital coordinate ($189.02\text{ mm}$), visually extending $\approx 24\text{ mm}$ past the right edge of the physical paper.

### 3.4 Empirical Proof: Smartphone Display Test
To definitively verify the hypothesis:
- The user displayed `pinball.jpg` directly on a smartphone screen (which displays pixels with a strict $1:1$ square aspect ratio and zero printer driver compression).
- When tracked by the live webcam, the green 3D bounding box aligned **with 100% pixel-perfect superimposition across all four borders**.
- This confirmed that the 3D-to-2D projection math, camera intrinsics ($[f_x, f_y, c_x, c_y]$), and WASM pose matrix $[R \mid t]$ are completely accurate.

### 3.5 Why JSARToolKitNFT Never Exposed This
In the upstream `jsartoolkitNFT` reference implementation (`examples/threejs_worker.js`):
- The visualization renders only a **single 3D sphere placed at the center coordinate**:
  $$X_{\text{sphere}} = \frac{W_{\text{mm}}}{2}, \quad Y_{\text{sphere}} = \frac{H_{\text{mm}}}{2}$$
- When a marker is squished horizontally on paper, the physical center remains at the geometric midpoint, completely masking the horizontal compression.
- `WebARKitLib.rs` is the first implementation to project the full outer perimeter bounding quad ($P_0 \to P_1 \to P_2 \to P_3$), exposing the physical printout distortion.

### 3.6 Production Architecture ($320 \times 240$ Downscaled Pipeline)
- **High-Performance 4:3 Padded Downscaling**: Downscaling video frames to max 320px (`pscale = 320 / max(vw, vh*4/3)`) and padding to 4:3 ensures WASM KPM keypoint detection and AR2 template tracking run in ~3.8 ms per frame (~22–30+ Worker FPS) while the UI maintains **134+ FPS**.
- **Coordinate Projection Inversion**: Reverse transformation `(uProc - ox) / pscale` accurately maps 3D projections from the padded processing buffer back to native canvas coordinates.
- **Clean API**: No manual slider hacks; physical marker dimensions ($W_{\text{mm}}$, $H_{\text{mm}}$) are used directly.
- **Visual Diagnostics**: Retained the magenta center tracking dot and 3D coordinate axes at $(0,0,0)$ to provide visual parity with `jsartoolkitNFT`.

---

## 4. Verification & Results

- **Unit Tests**: All **432 unit tests** in `webarkitlib-rs` passed cleanly.
- **WASM Builds**: Both `dist-std` (Scalar) and `dist-simd` (SIMD) packages built with `wasm-pack`.
- **UI Performance**: Main thread UI rendering runs at **135+ FPS**, while WASM tracking executes in the WebWorker background thread.
- **Git Commit**: Committed on branch `feat/wasm-video-nft-215` as commit `6249269`.
