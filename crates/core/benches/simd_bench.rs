use criterion::{black_box, criterion_group, criterion_main, Criterion};
use core::pattern::{dot_product_scalar, dot_product_simd_x86};
use core::image_proc::{rgba_to_gray_scalar, rgba_to_gray_simd_x86, box_filter_h_scalar, box_filter_v_scalar, box_filter_h_simd_x86, box_filter_v_simd_x86};

fn bench_dot_product(c: &mut Criterion) {
    let size = 16 * 16 * 3;
    let a = vec![10i32; size];
    let b = vec![20i32; size];

    let mut group = c.benchmark_group("dot_product");
    
    group.bench_function("scalar", |bencher| {
        bencher.iter(|| dot_product_scalar(black_box(&a), black_box(&b)))
    });

    #[cfg(all(feature = "simd-pattern", target_arch = "x86_64", target_feature = "sse4.1"))]
    {
        group.bench_function("simd_x86", |bencher| {
            bencher.iter(|| unsafe { dot_product_simd_x86(black_box(&a), black_box(&b)) })
        });
    }
    
    group.finish();
}

fn bench_rgba_to_gray(c: &mut Criterion) {
    let width = 640;
    let height = 480;
    let rgba = vec![128u8; (width * height * 4) as usize];

    let mut group = c.benchmark_group("rgba_to_gray");
    
    group.bench_function("scalar", |bencher| {
        bencher.iter(|| rgba_to_gray_scalar(black_box(&rgba)))
    });

    #[cfg(all(feature = "simd-image", target_arch = "x86_64", target_feature = "sse4.1"))]
    {
        group.bench_function("simd_x86", |bencher| {
            bencher.iter(|| unsafe { rgba_to_gray_simd_x86(black_box(&rgba)) })
        });
    }
    
    group.finish();
}

fn bench_box_filter(c: &mut Criterion) {
    let width = 640;
    let height = 480;
    let data = vec![128u8; (width * height) as usize];
    let mut temp = vec![0u16; (width * height) as usize];
    let mut out = vec![0u8; (width * height) as usize];
    let box_size = 5;
    let half = box_size / 2;
    let bias = 0;

    let mut group = c.benchmark_group("box_filter");
    
    group.bench_function("scalar", |bencher| {
        bencher.iter(|| {
            box_filter_h_scalar(black_box(&data), black_box(&mut temp), width, height, half);
            box_filter_v_scalar(black_box(&temp), black_box(&mut out), width, height, half, bias);
        })
    });

    #[cfg(all(feature = "simd-image", target_arch = "x86_64", target_feature = "sse4.1"))]
    {
        group.bench_function("simd_x86", |bencher| {
            bencher.iter(|| unsafe {
                box_filter_h_simd_x86(black_box(&data), black_box(&mut temp), width, height, half);
                box_filter_v_simd_x86(black_box(&temp), black_box(&mut out), width, height, half, bias);
            })
        });
    }
    
    group.finish();
}

criterion_group!(benches, bench_dot_product, bench_rgba_to_gray, bench_box_filter);
criterion_main!(benches);
