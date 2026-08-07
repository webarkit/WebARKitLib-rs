/*
 *  kpm_bench.rs
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

//! End-to-end KPM (Keypoint Matching) detection benchmark.
//!
//! Measures a single `KpmHandle::kpm_matching` query — the per-frame NFT
//! detection path — using the pure-Rust `RustFreakMatcher` backend against
//! the `pinball` reference marker on the `pinball-demo.jpg` query image.
//! `marker_bench` only covers barcode/template marker detection, so this is
//! the regression signal for the FreakMatcher pipeline (deferred from #142,
//! tracked in #225).
//!
//! Fixtures live in `crates/core/examples/Data/` (the same assets the
//! `simple_nft` example and the KPM regression tests use). Setup — loading
//! the reference data set and building the handle — is done once outside the
//! measured loop; only `kpm_matching` is timed.
//!
//! To also compare against the C++ FreakMatcher, build with
//! `--features ffi-backend` and swap in `CppFreakMatcher` (see #225 goal:
//! pure-Rust should stay within ~20% of C++).

use criterion::{criterion_group, criterion_main, Criterion};
use std::hint::black_box;
use std::io::Cursor;
use std::path::Path;
use std::sync::Arc;
use std::time::Duration;

use webarkitlib_rs::kpm::ref_data_set::KPM_CHANGE_PAGE_NO_ALL_PAGES;
use webarkitlib_rs::kpm::types::KpmRefDataSet;
use webarkitlib_rs::kpm::{KpmHandle, RustFreakMatcher};
use webarkitlib_rs::types::{ARParam, ARParamLT};

fn kpm_matching_benchmark(c: &mut Criterion) {
    let data_dir = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("examples")
        .join("Data");

    // Query image (pinball-demo) → grayscale. Exact luma values are
    // irrelevant for timing, so we use the `image` crate directly.
    let img = image::open(data_dir.join("pinball-demo.jpg"))
        .expect("failed to open pinball-demo.jpg")
        .to_luma8();
    let width = img.width() as i32;
    let height = img.height() as i32;
    let luma: Vec<u8> = img.into_raw();

    // Camera parameters, scaled to the image size (mirrors simple_nft).
    let param_bytes =
        std::fs::read(data_dir.join("camera_para.dat")).expect("failed to read camera_para.dat");
    let mut param =
        ARParam::load(Cursor::new(&param_bytes)).expect("failed to parse camera params");
    let sx = width as f64 / param.xsize as f64;
    let sy = height as f64 / param.ysize as f64;
    for col in 0..4 {
        param.mat[0][col] *= sx;
        param.mat[1][col] *= sy;
    }
    param.xsize = width;
    param.ysize = height;

    // Reference data set (.fset3), assigned to page 0.
    let mut ref_data =
        KpmRefDataSet::load(&data_dir.join("pinball.fset3")).expect("failed to load pinball.fset3");
    ref_data.change_page_no(KPM_CHANGE_PAGE_NO_ALL_PAGES, 0);

    // KpmHandle over the pure-Rust FreakMatcher backend.
    let param_lt = Arc::new(ARParamLT::new_basic(param));
    let backend = RustFreakMatcher::new(width, height).expect("failed to create RustFreakMatcher");
    let mut kpm_handle = KpmHandle::new(width, height, Some(param_lt), Box::new(backend));
    kpm_handle
        .set_ref_data_set(ref_data)
        .expect("failed to set ref data set");

    // A full KPM query is heavy; keep the sample count and measurement time
    // modest so the bench stays runnable in CI without dominating the suite.
    let mut group = c.benchmark_group("kpm");
    group.sample_size(10);
    group.measurement_time(Duration::from_secs(15));
    group.bench_function(format!("kpm_matching_pinball_{width}x{height}"), |b| {
        b.iter(|| {
            kpm_handle
                .kpm_matching(black_box(&luma))
                .expect("kpm_matching failed");
        })
    });
    group.finish();
}

criterion_group!(benches, kpm_matching_benchmark);
criterion_main!(benches);
