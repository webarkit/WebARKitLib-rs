/*
 *  lib.rs
 *  WebARKitLib-rs
 *
 *  This file is part of WebARKitLib-rs - WebARKit.
 *
 *  WebARKitLib-rs is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  WebARKitLib-rs is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with WebARKitLib-rs.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  As a special exception, the copyright holders of this library give you
 *  permission to link this library with independent modules to produce an
 *  executable, regardless of the license terms of these independent modules, and to
 *  copy and distribute the resulting executable under terms of your choice,
 *  provided that you also meet, for each linked independent module, the terms and
 *  conditions of the license of that module. An independent module is a module
 *  which is neither derived from nor based on this library. If you modify this
 *  library, you may extend this exception to your version of the library, but you
 *  are not obligated to do so. If you do not wish to do so, delete this exception
 *  statement from your version.
 *
 *  Copyright 2026 WebARKit.
 *
 *  Author(s): Walter Perdan @kalwalt https://github.com/kalwalt
 *
 */

//! # WebARKitLib-rs Core
//!
//! `webarkitlib-rs` is the foundational library for the Rust port of WebARKitLib.
//! It provides core computer vision algorithms for Augmented Reality, including:
//! - Image processing and filtering
//! - Thresholding and labeling
//! - Marker detection and identification
//! - Pose estimation and matrix calculations
//! - NFT (Natural Feature Tracking) marker generation pipeline
//!
//! ## NFT Marker Generation
//!
//! The [`ar2`] module provides a complete pipeline for generating NFT marker files
//! compatible with ARnft and the C++ NFT-Marker-Creator-App:
//!
//! - [`ar2::ar2_gen_image_set`] — builds a multi-resolution image pyramid (`.iset`)
//! - [`ar2::ar2_gen_feature_map`] — extracts gradient-based features at each pyramid level (`.fset`)
//! - [`ar2::AR2ImageSetT::save`] — writes the image pyramid as a JPEG-compressed `.iset` file
//!   matching `ar2WriteImageSet()` format (~300 KB vs ~10 MB raw)
//! - [`kpm::types::KpmRefDataSet::generate`] — extracts FREAK descriptors for KPM recognition (`.fset3`)
//!
//! See the `nft_marker_gen` example for a complete end-to-end usage.
//!
//! ## Performance and SIMD
//!
//! This crate includes high-performance SIMD optimizations for critical image processing
//! paths (grayscale conversion, filtering, pattern matching, and NFT feature correlation).
//!
//! ### SIMD feature flags
//!
//! | Feature | What it accelerates |
//! |---|---|
//! | `simd-x86-sse41` | `get_similarity` correlation kernel (SSE4.1 intrinsics) |
//! | `simd-x86-avx2` | `get_similarity` correlation kernel (AVX2+FMA intrinsics, fastest) |
//! | `simd-image` | Grayscale conversion and image processing |
//! | `simd-pattern` | Pattern matching correlation |
//! | `simd-wasm32` | WASM SIMD for web targets |
//! | `simd` | Umbrella — enables all of the above |
//!
//! SIMD dispatch is automatic at runtime: AVX2+FMA → SSE4.1 → scalar fallback.
//!
//! ### Rayon parallelism
//!
//! Rayon is always enabled (no feature flag). The NFT pipeline parallelizes:
//! - **Image pyramid generation** — each pyramid level is built in parallel
//! - **Stage-3 feature scoring** — row-level parallelism via `par_chunks_mut`
//!
//! ### Recommended build for NFT marker generation (x86_64)
//!
//! ```bash
//! cargo run --release --features "log-helpers,simd-x86-sse41,simd-x86-avx2" \
//!     --example nft_marker_gen -- --input marker.jpg --output marker --dpi 220
//! ```
//!
//! > **Always use `--release`** — debug builds are 5–10× slower.
//!
//! No C++ toolchain is required: since #179 `nft_marker_gen` produces the
//! complete marker set (`.iset` + `.fset` + `.fset3`) through the pure-Rust
//! [`RustFreakMatcher`](kpm::RustFreakMatcher). The `ffi-backend` feature is
//! **not** needed for marker generation — it exists only for development
//! cross-validation against the C++ backend.
//!
//! Detailed benchmark results can be found in `crates/core/benches/BENCHMARKS.md`.
//!
//!
//! ## Example
//!
//! ```rust,no_run
//! // Preferred: use the canonical module path
//! use webarkitlib_rs::ar::marker::ar_detect_marker;
//!
//! // Also available via the top-level re-export (backward compatibility):
//! use webarkitlib_rs::marker::ar_detect_marker as ar_detect_marker_compat;
//! ```

pub mod ar;
pub mod ar2;
pub mod arlog;
pub mod icp;
pub mod kpm;
pub mod types;
pub mod version;

// Re-exports — keep the flat crate-level paths that existed before the `ar/`
// restructure (issue #82) so downstream code compiles without path changes.
pub use ar::bch;
pub use ar::filter;
pub use ar::image_proc;
pub use ar::labeling;
pub use ar::marker;
pub use ar::math;
pub use ar::matrix;
pub use ar::param;
pub use ar::param_gl;
pub use ar::pattern;
pub use ar::pose;
