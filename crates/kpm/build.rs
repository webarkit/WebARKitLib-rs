use std::env;
use std::path::PathBuf;

fn main() {
    if cfg!(feature = "ffi-backend") {
        build_freak_matcher();
        generate_bindings();
    }
}

fn build_freak_matcher() {
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());

    // Workspace root is two levels up from crates/kpm/
    let workspace_root = manifest_dir.join("..").join("..");
    let webarkitlib = workspace_root
        .join("benchmarks")
        .join("c_benchmark")
        .join("src")
        .join("WebARKitLib");

    let freak_matcher_root = webarkitlib
        .join("lib")
        .join("SRC")
        .join("KPM")
        .join("FreakMatcher");

    let include_root = webarkitlib.join("include");

    let src_dir = manifest_dir.join("src");

    // FreakMatcher C++ source files
    let cpp_sources: Vec<PathBuf> = vec![
        freak_matcher_root.join("facade").join("visual_database_facade.cpp"),
        freak_matcher_root.join("detectors").join("DoG_scale_invariant_detector.cpp"),
        freak_matcher_root.join("detectors").join("gaussian_scale_space_pyramid.cpp"),
        freak_matcher_root.join("detectors").join("pyramid.cpp"),
        freak_matcher_root.join("matchers").join("hough_similarity_voting.cpp"),
        // Additional sources required by transitive dependencies
        freak_matcher_root.join("detectors").join("gradients.cpp"),
        freak_matcher_root.join("detectors").join("orientation_assignment.cpp"),
        freak_matcher_root.join("matchers").join("freak.cpp"),
        freak_matcher_root.join("framework").join("image.cpp"),
        freak_matcher_root.join("framework").join("logger.cpp"),
        freak_matcher_root.join("framework").join("timers.cpp"),
        freak_matcher_root.join("framework").join("date_time.cpp"),
        // Our C API wrapper
        src_dir.join("kpm_c_api.cpp"),
    ];

    let mut build = cc::Build::new();
    build
        .cpp(true)
        .std("c++17")
        .define("BINARY_FEATURE", "1")
        .include(&freak_matcher_root)
        .include(&include_root)
        .include(&src_dir);

    // Platform-specific flags
    let target = env::var("TARGET").unwrap_or_default();
    if !target.contains("msvc") {
        build.flag("-Wno-unused-parameter");
        build.flag("-Wno-sign-compare");
    }

    for src in &cpp_sources {
        build.file(src);
    }

    build.compile("freakMatcher");

    // Link C++ standard library on non-MSVC platforms
    if !target.contains("msvc") {
        println!("cargo:rustc-link-lib=stdc++");
    }

    // Rerun if sources change
    for src in &cpp_sources {
        println!("cargo:rerun-if-changed={}", src.display());
    }
    println!("cargo:rerun-if-changed=src/kpm_c_api.h");
}

fn generate_bindings() {
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let header = manifest_dir.join("src").join("kpm_c_api.h");
    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());

    let bindings = bindgen::Builder::default()
        .header(header.to_str().unwrap())
        .parse_callbacks(Box::new(bindgen::CargoCallbacks::new()))
        .generate()
        .expect("Unable to generate bindings for kpm_c_api.h");

    bindings
        .write_to_file(out_dir.join("kpm_bindings.rs"))
        .expect("Couldn't write kpm_bindings.rs");
}
