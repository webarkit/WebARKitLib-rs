# WebARKitLib-rs 🦀

**WebARKitLib-rs** is a high-performance, memory-safe Rust port of the [WebARKitLib](https://github.com/webarkit/WebARKitLib) (originally C/C++). 

This project aims to provide a pure-Rust implementation of the core ARToolKit algorithms, targeting both **native** systems and **WebAssembly (WASM)** for high-performance augmented reality in the browser.

> [!NOTE]
> This project is currently a **Work in Progress**. While the core marker detection and pose estimation are functional, we are continuously optimizing and expanding the feature set.

## 🌟 Key Features

- **Pure Rust**: Built for safety, speed, and modern concurrency.
- **WASM Ready**: Designed from the ground up to be compiled to WebAssembly for seamless web integration.
- **Side-Effect Free**: The core library is designed to be pure, making it easy to test and integrate into different environments.
- **CI-Integrated Benchmarking**: Automated performance comparisons against the original C implementation.

## 🚀 Getting Started

### Native Rust Example

To see the marker detection in action on your local machine, run the provided simple example:

```bash
cargo run --example simple
```
This example loads a camera parameter file, a hiro marker pattern, and a sample image, performing detection and outputting the 3D pose extrinsics.

### WebAssembly (WASM) Demo

The WASM port allows you to run the AR engine directly in most modern browsers.

1. **Build the WASM module**:
   ```bash
   cd crates/wasm
   wasm-pack build --target web
   ```

2. **Run the demo**:
   You can serve the `www` folder using any local HTTP server (e.g., `python -m http.server 8080`).

## 📊 Benchmarking

We maintain a strict performance comparison with the original C library to ensure our Rust port remains competitive.

### Running the Comparison

1.  **Bootstrap the C library**:
    ```bash
    cd benchmarks/c_benchmark
    python ../bootstrap.py --bootstrap-file libraries.json
    ```

2.  **Execute the Suite**:
    ```bash
    # Rust Benchmark
    cargo bench -p core --bench marker_bench

    # C Benchmark (Manual build required once)
    cd benchmarks/c_benchmark/build
    cmake --build . --config Release
    ../c_benchmark ../../data/camera_para.dat ../../data/patt.hiro ../../data/hiro.raw 429 317
    ```

### Tracking Performance

We use `criterion` to track performance over time. You can save specific snapshots as "baselines":

```bash
# Save a milestone baseline
cargo bench -- --save-baseline milestone-20240304
```

## 🏗️ Project Structure

- `crates/core`: The core AR engine (pure Rust).
- `crates/wasm`: WASM bindings and JavaScript glue code.
- `benchmarks`: C vs Rust performance comparison suite.
- `examples`: Usage demonstrations.

## 🗺️ Roadmap

### 🎯 Near-term Goals (v1.0.0+)
- **Dual WASM Builds**: Provide automated builds for both SIMD-enabled and non-SIMD environments to maximize compatibility.
- **Barcode Marker Support**: Implement support for matrix-based barcode markers.
- **Enhanced Documentation**: Expand API documentation, integration guides, and advanced examples.

### 🔭 Long-term Vision
- **Multi-Marker Tracking**: Support for simultaneous tracking of multiple independent markers.
- **NFT (Natural Feature Tracking)**: Support for KPM/NFT algorithms to track arbitrary images.
- **Integrated Video Pipeline**: Develop advanced video processing handlers (inspired by ARToolkitX) to provide a unified AR experience across platforms.

## 📜 Credits

This project is a port of the excellent [WebARKitLib](https://github.com/webarkit/WebARKitLib) project. Special thanks to the original ARToolKit contributors.

## 📄 License

This project is licensed under the LGPLv3 License - see the [LICENSE](LICENSE) file for details.
