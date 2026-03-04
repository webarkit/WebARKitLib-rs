# WebARKitLib.rs
Rust version of the WebARKitLib. This is a Work in Progress!

## Benchmarking

We provide a comprehensive benchmarking suite to compare the performance of the Rust port against the original C implementation.

### 1. Prerequisites (C Benchmark)
The C benchmark requires the original WebARKitLib source code. We use a bootstrapping script to automate this:

```bash
cd benchmarks/c_benchmark
python ../bootstrap.py --bootstrap-file libraries.json
```
This will download the C source into `benchmarks/c_benchmark/src/WebARKitLib`.

### 2. Running the C Benchmark
The C benchmark is built using CMake:

```bash
cd benchmarks/c_benchmark
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . --config Release
cd ..
./build/Release/c_benchmark.exe ../data/camera_para.dat ../data/patt.hiro ../data/hiro.raw 429 317
```

### 3. Running the Rust Benchmark
The Rust implementation uses the `criterion` crate for high-precision benchmarking:

```bash
cargo bench -p core --bench marker_bench
```

### 4. How Criterion Works
The `criterion` crate stores historical benchmark data in `target/criterion/`.
- **Storage**: Results are saved in JSON/CSV format for each benchmark function.
- **Comparisons**: Every time you run `cargo bench`, it compares the current results with the previous run found in `target/criterion`.
- **Reporting**: It generates beautiful HTML reports at `target/criterion/report/index.html`.

### 5. Persistent Baselines (Tracking Progress)
Since `target/` is git-ignored, you can save specific snapshots as "baselines" to track progress over weeks or months.

#### Saving a Timestamped Baseline
To save a snapshot with a date (e.g., for a new release or a major optimization), use the following command:

**PowerShell:**
```powershell
$TS = Get-Date -Format "yyyyMMdd-HHmm"
cargo bench -- --save-baseline "milestone-$TS"
```

**Bash:**
```bash
TS=$(date +%Y%m%d-%H%M)
cargo bench -- --save-baseline "milestone-$TS"
```

#### Comparing against a Baseline
Later, you can compare your current code against that specific snapshot:
```bash
cargo bench -- --baseline milestone-20231027-1430
```

### 6. Continuous Integration (GitHub Actions)
Every push to `main` automatically runs the benchmarks. You can download the full HTML reports:
1. Go to the **Actions** tab on GitHub.
2. Select the latest **Rust CI** run.
3. Scroll down to **Artifacts** and download `benchmark-reports`.
4. Open `report/index.html` locally to see trends and regression analysis.
