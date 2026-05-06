/*
 *  simd_bench.rs
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

use criterion::{black_box, criterion_group, criterion_main, Criterion};
use webarkitlib_rs::image_proc::{box_filter_h_scalar, box_filter_v_scalar, rgba_to_gray_scalar};
use webarkitlib_rs::pattern::dot_product_scalar;

#[cfg(all(target_arch = "x86_64", target_feature = "sse4.1"))]
use webarkitlib_rs::image_proc::{box_filter_h_simd_x86, box_filter_v_simd_x86, rgba_to_gray_simd_x86};

#[cfg(all(target_arch = "x86_64", target_feature = "sse4.1"))]
use webarkitlib_rs::pattern::dot_product_simd_x86;

fn bench_rgba_to_gray(c: &mut Criterion) {
    let size = 640 * 480 * 4;
    let data = vec![128u8; size];

    let mut group = c.benchmark_group("rgba_to_gray");

    group.bench_function("scalar", |bencher| {
        bencher.iter(|| rgba_to_gray_scalar(black_box(&data)))
    });

    #[cfg(all(target_arch = "x86_64", target_feature = "sse4.1"))]
    group.bench_function("simd_x86", |bencher| {
        bencher.iter(|| unsafe { rgba_to_gray_simd_x86(black_box(&data)) })
    });

    group.finish();
}

fn bench_dot_product(c: &mut Criterion) {
    let size = 1024;
    let a = vec![123i16; size];
    let b = vec![456i16; size];

    let mut group = c.benchmark_group("dot_product");

    group.bench_function("scalar", |bencher| {
        bencher.iter(|| dot_product_scalar(black_box(&a), black_box(&b)))
    });

    #[cfg(all(target_arch = "x86_64", target_feature = "sse4.1"))]
    group.bench_function("simd_x86", |bencher| {
        bencher.iter(|| unsafe { dot_product_simd_x86(black_box(&a), black_box(&b)) })
    });

    group.finish();
}

fn bench_box_filter(c: &mut Criterion) {
    let width = 640;
    let height = 480;
    let size = (width * height) as usize;
    let data = vec![128u8; size];
    let mut temp_u16 = vec![0u16; size];
    let mut out = vec![0u8; size];
    let half = 2;
    let bias = 0;

    let mut group = c.benchmark_group("box_filter");

    group.bench_function("h_scalar", |bencher| {
        bencher.iter(|| {
            box_filter_h_scalar(
                black_box(&data),
                black_box(&mut temp_u16),
                width,
                height,
                half,
            )
        })
    });

    #[cfg(all(target_arch = "x86_64", target_feature = "sse4.1"))]
    group.bench_function("h_simd_x86", |bencher| {
        bencher.iter(|| unsafe {
            box_filter_h_simd_x86(
                black_box(&data),
                black_box(&mut temp_u16),
                width,
                height,
                half,
            )
        })
    });

    group.bench_function("v_scalar", |bencher| {
        bencher.iter(|| {
            box_filter_v_scalar(
                black_box(&temp_u16),
                black_box(&mut out),
                width,
                height,
                half,
                bias,
            )
        })
    });

    #[cfg(all(target_arch = "x86_64", target_feature = "sse4.1"))]
    group.bench_function("v_simd_x86", |bencher| {
        bencher.iter(|| unsafe {
            box_filter_v_simd_x86(
                black_box(&temp_u16),
                black_box(&mut out),
                width,
                height,
                half,
                bias,
            )
        })
    });

    group.finish();
}

criterion_group!(
    benches,
    bench_rgba_to_gray,
    bench_dot_product,
    bench_box_filter
);
criterion_main!(benches);
