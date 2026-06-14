/*
 *  pyramid_bench.rs
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

//! Criterion baseline for `kpm::freak::pyramid` downsampling (#131).
//!
//! Establishes the scalar baseline so the SIMD path (#132) and any
//! chunked-layout work (#133) can prove their speedup against stable
//! numbers. Two groups are measured at typical AR input resolutions
//! (640×480, 1280×720, 1920×1080):
//!
//! - `downsample`: one 2×2 box-filter decimation step (the hot inner loop).
//! - `pyramid_build`: end-to-end `Pyramid::build` with `num_levels = 4`.
//!
//! Inputs are filled deterministically with a fixed-seed PRNG so runs are
//! comparable across machines and across PRs in the trilogy.

use criterion::{criterion_group, criterion_main, BenchmarkId, Criterion, Throughput};
use purecv::core::Matrix;
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};
use std::hint::black_box;
use webarkitlib_rs::kpm::freak::pyramid::{downsample_scalar, Pyramid};

/// Typical AR camera input resolutions as `(width, height)`.
const SIZES: &[(usize, usize)] = &[(640, 480), (1280, 720), (1920, 1080)];

/// Build a deterministic grayscale `Matrix<u8>` of `rows × cols` using a
/// fixed-seed PRNG, so every benchmark run sees identical pixel data.
fn random_gray(rows: usize, cols: usize) -> Matrix<u8> {
    let mut rng = StdRng::seed_from_u64(0xA12B_C3D4);
    let data: Vec<u8> = (0..rows * cols).map(|_| rng.random::<u8>()).collect();
    Matrix::<u8>::from_vec(rows, cols, 1, data)
}

fn bench_downsample(c: &mut Criterion) {
    let mut group = c.benchmark_group("downsample");
    for &(w, h) in SIZES {
        let src = random_gray(h, w);
        // One output pixel per 2×2 source block; throughput is measured in
        // source pixels touched.
        group.throughput(Throughput::Elements((w * h) as u64));
        group.bench_with_input(
            BenchmarkId::new("scalar", format!("{w}x{h}")),
            &src,
            |bencher, src| bencher.iter(|| downsample_scalar(black_box(src))),
        );
    }
    group.finish();
}

fn bench_pyramid_build(c: &mut Criterion) {
    let mut group = c.benchmark_group("pyramid_build");
    for &(w, h) in SIZES {
        let src = random_gray(h, w);
        group.throughput(Throughput::Elements((w * h) as u64));
        group.bench_with_input(
            BenchmarkId::new("levels4", format!("{w}x{h}")),
            &src,
            |bencher, src| {
                bencher.iter(|| {
                    let mut p = Pyramid::new(4, 2.0);
                    p.build(black_box(src)).expect("build should succeed");
                    p
                })
            },
        );
    }
    group.finish();
}

criterion_group!(benches, bench_downsample, bench_pyramid_build);
criterion_main!(benches);
