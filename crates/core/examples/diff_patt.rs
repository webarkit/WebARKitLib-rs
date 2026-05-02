/*
 *  diff_patt.rs
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
 *  Author(s): Walter Perdan @kalwalt https://github.com/kalwalt https://github.com/kalwalt
 *
 */

//! Byte-level diff between the C-side and Rust-side `ext_patt` dumps produced
//! by `dump_patt` (issue #103 diagnostic).
//!
//! Usage:
//!   cargo run --example diff_patt --features log-helpers -- \
//!     [c_bin] [rs_bin]
//!
//! Defaults: `benchmarks/data/c_ext_patt.bin` and `benchmarks/data/rs_ext_patt.bin`.
//!
//! Reports:
//!   - byte-equal status
//!   - count and percentage of divergent bytes
//!   - first 8 divergent indices with (c, rs) values
//!   - max absolute delta
//!   - per-side variance (warns if either is zero — meaningless comparison)

use std::fs;
use std::path::PathBuf;
use webarkitlib_rs::{arlog_e, arlog_i, version};

fn main() {
    webarkitlib_rs::arlog::ar_log_init_default();
    arlog_i!("WebARKitLib-rs v{}", version::get_version());
    arlog_i!("diff_patt: byte-comparing C and Rust ext_patt dumps");

    let args: Vec<String> = std::env::args().collect();
    let c_path = PathBuf::from(arg_or(&args, 1, "benchmarks/data/c_ext_patt.bin"));
    let rs_path = PathBuf::from(arg_or(&args, 2, "benchmarks/data/rs_ext_patt.bin"));

    let c_bytes = match fs::read(&c_path) {
        Ok(b) => b,
        Err(e) => {
            arlog_e!("Failed to read {}: {}", c_path.display(), e);
            std::process::exit(2);
        }
    };
    let rs_bytes = match fs::read(&rs_path) {
        Ok(b) => b,
        Err(e) => {
            arlog_e!("Failed to read {}: {}", rs_path.display(), e);
            std::process::exit(2);
        }
    };

    arlog_i!("c side:  {} ({} bytes)", c_path.display(), c_bytes.len());
    arlog_i!("rs side: {} ({} bytes)", rs_path.display(), rs_bytes.len());

    if c_bytes.len() != rs_bytes.len() {
        arlog_e!(
            "LENGTH MISMATCH: c={} rs={} — cannot compare byte-by-byte",
            c_bytes.len(),
            rs_bytes.len()
        );
        std::process::exit(1);
    }
    let n = c_bytes.len();
    if n == 0 {
        arlog_e!("Both buffers are empty — no comparison performed");
        std::process::exit(1);
    }

    // Sanity: warn if either side is uniformly zero (likely a bug — meaningless compare).
    let c_var = variance(&c_bytes);
    let rs_var = variance(&rs_bytes);
    if c_var == 0.0 {
        arlog_e!("WARN: C buffer has zero variance (all bytes identical) — comparison may be misleading");
    }
    if rs_var == 0.0 {
        arlog_e!("WARN: Rust buffer has zero variance (all bytes identical) — comparison may be misleading");
    }

    let mut divergences: Vec<(usize, u8, u8)> = Vec::new();
    let mut max_abs_delta: u32 = 0;
    for i in 0..n {
        if c_bytes[i] != rs_bytes[i] {
            divergences.push((i, c_bytes[i], rs_bytes[i]));
            let d = (c_bytes[i] as i32 - rs_bytes[i] as i32).unsigned_abs();
            if d > max_abs_delta {
                max_abs_delta = d;
            }
        }
    }

    let identical = n - divergences.len();
    let pct = (divergences.len() as f64 / n as f64) * 100.0;

    arlog_i!("identical: {}/{} bytes", identical, n);
    arlog_i!("divergent: {} / {}  ({:.2}%)", divergences.len(), n, pct);

    if divergences.is_empty() {
        arlog_i!("PATCHES ARE BYTE-EQUAL ✓");
        std::process::exit(0);
    } else {
        let head: Vec<String> = divergences
            .iter()
            .take(8)
            .map(|(i, c, r)| format!("({}, c=0x{:02x}, rs=0x{:02x})", i, c, r))
            .collect();
        arlog_i!("first 8 divergences: [{}]", head.join(", "));
        arlog_i!("max abs delta: {}", max_abs_delta);
        std::process::exit(1);
    }
}

fn arg_or(args: &[String], idx: usize, default: &str) -> String {
    args.get(idx).cloned().unwrap_or_else(|| default.to_string())
}

fn variance(buf: &[u8]) -> f64 {
    if buf.is_empty() {
        return 0.0;
    }
    let mean: f64 = buf.iter().map(|&b| b as f64).sum::<f64>() / buf.len() as f64;
    let var: f64 = buf
        .iter()
        .map(|&b| {
            let d = b as f64 - mean;
            d * d
        })
        .sum::<f64>()
        / buf.len() as f64;
    var
}
