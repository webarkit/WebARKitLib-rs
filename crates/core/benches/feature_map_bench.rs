/*
 *  feature_map_bench.rs
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

//! Benchmark for the AR2 feature-map pipeline.
//!
//! Exercises `ar2_gen_feature_map` end-to-end on the `pinball.jpg` fixture so
//! Rayon + SIMD changes to Stage 3 can be measured against a stable baseline.
//!
//! The input image is downscaled for bench speed — the full 1637×2048 input
//! would make each iteration multi-minute. We target a realistic-but-bounded
//! working size that still stresses the scoring loop.

use criterion::{criterion_group, criterion_main, Criterion};
use std::hint::black_box;
use std::path::PathBuf;
use webarkitlib_rs::ar2::{ar2_gen_feature_map, ar2_gen_image_set};
use webarkitlib_rs::arlog_e;

fn bench_ar2_gen_feature_map(c: &mut Criterion) {
    webarkitlib_rs::arlog::ar_log_init_default();

    // Locate the pinball fixture relative to the crate root.
    let mut path = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    path.push("examples/Data/pinball.jpg");
    if !path.exists() {
        arlog_e!("Skipping bench: missing {}", path.display());
        return;
    }

    let img = image::open(&path).expect("failed to open pinball.jpg");
    // Downscale to ~400px wide for a tractable bench iteration.
    let target_w = 400u32;
    let scale = target_w as f32 / img.width() as f32;
    let target_h = (img.height() as f32 * scale) as u32;
    let small = img.resize_exact(target_w, target_h, image::imageops::FilterType::Triangle);
    let gray = small.to_luma8();
    let w = gray.width() as i32;
    let h = gray.height() as i32;
    let data = gray.into_raw();

    let image_set = ar2_gen_image_set(&data, w, h, 1, 72.0).expect("ar2_gen_image_set failed");

    let mut group = c.benchmark_group("ar2_gen_feature_map");
    group.sample_size(10);
    group.bench_function("pinball_400w", |b| {
        b.iter(|| {
            let fs = ar2_gen_feature_map(black_box(&image_set), 10, 100, 2)
                .expect("ar2_gen_feature_map failed");
            black_box(fs);
        })
    });
    group.finish();
}

criterion_group!(benches, bench_ar2_gen_feature_map);
criterion_main!(benches);
