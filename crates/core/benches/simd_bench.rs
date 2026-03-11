use criterion::{black_box, criterion_group, criterion_main, Criterion};
use webarkitlib_rs::image_proc::{
    rgba_to_gray_scalar, rgba_to_gray_simd_x86,
    box_filter_h_scalar, box_filter_h_simd_x86,
    box_filter_v_scalar, box_filter_v_simd_x86
};
use webarkitlib_rs::pattern::{dot_product_scalar, dot_product_simd_x86};

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
        bencher.iter(|| box_filter_h_scalar(black_box(&data), black_box(&mut temp_u16), width, height, half))
    });

    #[cfg(all(target_arch = "x86_64", target_feature = "sse4.1"))]
    group.bench_function("h_simd_x86", |bencher| {
        bencher.iter(|| unsafe { box_filter_h_simd_x86(black_box(&data), black_box(&mut temp_u16), width, height, half) })
    });

    group.bench_function("v_scalar", |bencher| {
        bencher.iter(|| box_filter_v_scalar(black_box(&temp_u16), black_box(&mut out), width, height, half, bias))
    });

    #[cfg(all(target_arch = "x86_64", target_feature = "sse4.1"))]
    group.bench_function("v_simd_x86", |bencher| {
        bencher.iter(|| unsafe { box_filter_v_simd_x86(black_box(&temp_u16), black_box(&mut out), width, height, half, bias) })
    });
    
    group.finish();
}

criterion_group!(benches, bench_rgba_to_gray, bench_dot_product, bench_box_filter);
criterion_main!(benches);
