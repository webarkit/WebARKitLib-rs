/*
 *  build.rs
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

    // WebARKitLib C++ sources ship with the crate via the git submodule at
    // `crates/core/third_party/WebARKitLib`. See docs/design/issue-72-ffi-backend-vendoring.md.
    let webarkitlib = manifest_dir
        .join("third_party")
        .join("WebARKitLib");

    let freak_matcher_root = webarkitlib
        .join("lib")
        .join("SRC")
        .join("KPM")
        .join("FreakMatcher");

    let include_root = webarkitlib.join("include");

    if !freak_matcher_root.exists() {
        panic!(
            "WebARKitLib C++ sources not found at {}. \
             If you are building from a git clone, run \
             `git submodule update --init --recursive`. \
             (Installs from crates.io ship the sources inside the .crate tarball.)",
            freak_matcher_root.display()
        );
    }

    // C++ wrapper files now live under src/kpm/
    let src_dir = manifest_dir.join("src").join("kpm");

    // FreakMatcher C++ source files
    let cpp_sources: Vec<PathBuf> = vec![
        freak_matcher_root
            .join("facade")
            .join("visual_database_facade.cpp"),
        freak_matcher_root
            .join("detectors")
            .join("DoG_scale_invariant_detector.cpp"),
        freak_matcher_root
            .join("detectors")
            .join("gaussian_scale_space_pyramid.cpp"),
        freak_matcher_root.join("detectors").join("pyramid.cpp"),
        freak_matcher_root
            .join("matchers")
            .join("hough_similarity_voting.cpp"),
        // Additional sources required by transitive dependencies
        freak_matcher_root.join("detectors").join("gradients.cpp"),
        freak_matcher_root
            .join("detectors")
            .join("orientation_assignment.cpp"),
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
    println!("cargo:rerun-if-changed=src/kpm/kpm_c_api.h");
}

fn generate_bindings() {
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let header = manifest_dir.join("src").join("kpm").join("kpm_c_api.h");
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
