/*
 *  bch.rs
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
//! # BCH Code Decoding - used for error correction in AR marker detection.
use crate::types::ARMatrixCodeType;
use crate::{arlog_d, arlog_e};

// =====================================================================
// GF(2^m) lookup tables — ported verbatim from arPattGetID.c lines
// 1826-1831 in the upstream WebARKitLib C code.
// =====================================================================

/// GF(2^4) `alpha_to` table (n = 15) — used by BCH(15, k, t).
const BCH_15_ALPHA_TO: [i32; 15] = [1, 2, 4, 8, 3, 6, 12, 11, 5, 10, 7, 14, 15, 13, 9];

/// GF(2^4) `index_of` (inverse) table — `BCH_15_INDEX_OF[BCH_15_ALPHA_TO[i]] = i`.
const BCH_15_INDEX_OF: [i32; 16] = [-1, 0, 1, 4, 2, 8, 5, 10, 3, 14, 9, 7, 6, 13, 11, 12];

/// GF(2^5) `alpha_to` table (n = 31) — used by BCH(31, k, t).
const BCH_31_ALPHA_TO: [i32; 31] = [
    1, 2, 4, 8, 16, 5, 10, 20, 13, 26, 17, 7, 14, 28, 29, 31, 27, 19, 3, 6, 12, 24, 21, 15, 30, 25,
    23, 11, 22, 9, 18,
];

/// GF(2^5) `index_of` (inverse) table — `BCH_31_INDEX_OF[BCH_31_ALPHA_TO[i]] = i`.
const BCH_31_INDEX_OF: [i32; 32] = [
    -1, 0, 1, 18, 2, 5, 19, 11, 3, 29, 6, 27, 20, 8, 12, 23, 4, 10, 30, 17, 7, 22, 28, 26, 21, 25,
    9, 16, 13, 14, 24, 15,
];

/// GF(2^7) `alpha_to` table (n = 127) — used by BCH(127, 64, 22) for AR_MATRIX_CODE_GLOBAL_ID.
const BCH_127_ALPHA_TO: [i32; 127] = [
    1, 2, 4, 8, 16, 32, 64, 3, 6, 12, 24, 48, 96, 67, 5, 10, 20, 40, 80, 35, 70, 15, 30, 60, 120,
    115, 101, 73, 17, 34, 68, 11, 22, 44, 88, 51, 102, 79, 29, 58, 116, 107, 85, 41, 82, 39, 78,
    31, 62, 124, 123, 117, 105, 81, 33, 66, 7, 14, 28, 56, 112, 99, 69, 9, 18, 36, 72, 19, 38, 76,
    27, 54, 108, 91, 53, 106, 87, 45, 90, 55, 110, 95, 61, 122, 119, 109, 89, 49, 98, 71, 13, 26,
    52, 104, 83, 37, 74, 23, 46, 92, 59, 118, 111, 93, 57, 114, 103, 77, 25, 50, 100, 75, 21, 42,
    84, 43, 86, 47, 94, 63, 126, 127, 125, 121, 113, 97, 65,
];

/// GF(2^7) `index_of` (inverse) table — `BCH_127_INDEX_OF[BCH_127_ALPHA_TO[i]] = i`.
const BCH_127_INDEX_OF: [i32; 128] = [
    -1, 0, 1, 7, 2, 14, 8, 56, 3, 63, 15, 31, 9, 90, 57, 21, 4, 28, 64, 67, 16, 112, 32, 97, 10,
    108, 91, 70, 58, 38, 22, 47, 5, 54, 29, 19, 65, 95, 68, 45, 17, 43, 113, 115, 33, 77, 98, 117,
    11, 87, 109, 35, 92, 74, 71, 79, 59, 104, 39, 100, 23, 82, 48, 119, 6, 126, 55, 13, 30, 62, 20,
    89, 66, 27, 96, 111, 69, 107, 46, 37, 18, 53, 44, 94, 114, 42, 116, 76, 34, 86, 78, 73, 99,
    103, 118, 81, 12, 125, 88, 61, 110, 26, 36, 106, 93, 52, 75, 41, 72, 85, 80, 102, 60, 124, 105,
    25, 40, 51, 101, 84, 24, 123, 83, 50, 49, 122, 120, 121,
];

// =====================================================================
// Public decoders for short codes (parity-65, hamming-63).
// =====================================================================

pub fn decode_parity65(code_raw: u64) -> Result<u64, &'static str> {
    const PARITY65_DECODER_TABLE: [i8; 64] = [
        0, -1, -1, 3, -1, 5, 6, -1, -1, 9, 10, -1, 12, -1, -1, 15, -1, 17, 18, -1, 20, -1, -1, 23,
        24, -1, -1, 27, -1, 29, 30, -1, -1, 1, 2, -1, 4, -1, -1, 7, 8, -1, -1, 11, -1, 13, 14, -1,
        16, -1, -1, 19, -1, 21, 22, -1, -1, 25, 26, -1, 28, -1, -1, 31,
    ];
    if code_raw >= 64 {
        arlog_d!("decode_parity65: EDC fail (code_raw={:#x} >= 64)", code_raw);
        return Err("EDC fail");
    }
    let val = PARITY65_DECODER_TABLE[code_raw as usize];
    if val < 0 {
        arlog_d!(
            "decode_parity65: EDC fail (table miss for code_raw={:#x})",
            code_raw
        );
        Err("EDC fail")
    } else {
        Ok(val as u64)
    }
}

pub fn decode_hamming63(code_raw: u64) -> Result<(u64, i32), &'static str> {
    const HAMMING63_DECODER_TABLE: [i8; 64] = [
        0, 0, 0, 1, 0, 1, 1, 1, 0, 2, 4, -1, -1, 5, 3, 1, 0, 2, -1, 6, 7, -1, 3, 1, 2, 2, 3, 2, 3,
        2, 3, 3, 0, -1, 4, 6, 7, 5, -1, 1, 4, 5, 4, 4, 5, 5, 4, 5, 7, 6, 6, 6, 7, 7, 7, 6, -1, 2,
        4, 6, 7, 5, 3, -1,
    ];
    const ERROR_CORRECTED: [bool; 64] = [
        false, true, true, true, true, true, true, false, true, true, true, false, false, true,
        true, true, true, true, false, true, true, false, true, true, true, false, true, true,
        true, true, false, true, true, false, true, true, true, true, false, true, true, true,
        false, true, true, false, true, true, true, true, true, false, false, true, true, true,
        false, true, true, true, true, true, true, false,
    ];
    if code_raw >= 64 {
        arlog_d!(
            "decode_hamming63: EDC fail (code_raw={:#x} >= 64)",
            code_raw
        );
        return Err("EDC fail");
    }
    let val = HAMMING63_DECODER_TABLE[code_raw as usize];
    if val < 0 {
        arlog_d!(
            "decode_hamming63: EDC fail (table miss for code_raw={:#x})",
            code_raw
        );
        Err("EDC fail")
    } else {
        let corrected = if ERROR_CORRECTED[code_raw as usize] {
            1
        } else {
            0
        };
        Ok((val as u64, corrected))
    }
}

// =====================================================================
// Berlekamp-Massey error correction core.
// =====================================================================

/// Runs Berlekamp's iterative error-correction algorithm on a received BCH
/// codeword. Mirrors the Berlekamp-Massey body from `arPattGetID.c`'s
/// `decode_bch` function.
///
/// On success, in-place flips bit positions in `recd` to correct errors.
///
/// # Parameters
/// - `recd` — received bits, indexed `[0..length]` (LSB at index 0). At least
///   `n` entries (sized to fit `loc`).
/// - `t` — error-correction capability (max number of correctable errors).
/// - `n` — codeword length (`2^m - 1` for GF(2^m)).
/// - `length` — effective input length (used codeword bits; may be < `n` for
///   shortened codes).
/// - `alpha_to` / `index_of` — GF(2^m) lookup tables.
///
/// # Returns
/// `Ok(num_corrected)` on success (0 if no errors detected),
/// `Err` if more than `t` errors were detected and correction failed.
fn bch_correct_errors(
    recd: &mut [u8],
    t: usize,
    n: usize,
    length: usize,
    alpha_to: &[i32],
    index_of: &[i32],
) -> Result<i32, &'static str> {
    let t2 = 2 * t;
    let mut s = vec![0i32; t2 + 1];

    // Compute syndromes s[1..=t2].
    let mut syn_error = false;
    for i in 1..=t2 {
        s[i] = 0;
        for j in 0..length {
            if recd[j] != 0 {
                s[i] ^= alpha_to[(i * j) % n];
            }
        }
        if s[i] != 0 {
            syn_error = true;
        }
        s[i] = index_of[s[i] as usize];
    }

    let mut l_arr = vec![0usize; t2 + 2];

    if !syn_error {
        return Ok(0);
    }

    // Berlekamp's iterative algorithm to compute the error-locator polynomial.
    let mut elp = vec![vec![0i32; t2]; t2 + 2];
    let mut d = vec![0i32; t2 + 2];
    let mut u_lu = vec![0i32; t2 + 2];
    let mut loc = vec![0usize; n];
    let mut reg = vec![0i32; t + 1];

    d[0] = 0;
    d[1] = s[1];
    elp[0][0] = 0;
    elp[1][0] = 1;
    elp[0][1..t2].fill(-1);
    elp[1][1..t2].fill(0);
    l_arr[0] = 0;
    l_arr[1] = 0;
    u_lu[0] = -1;
    u_lu[1] = 0;
    let mut u = 0usize;

    loop {
        u += 1;
        if d[u] == -1 {
            l_arr[u + 1] = l_arr[u];
            for i in 0..=l_arr[u] {
                elp[u + 1][i] = elp[u][i];
                if elp[u][i] >= 0 {
                    elp[u][i] = index_of[elp[u][i] as usize];
                }
            }
        } else {
            // Find a previous step q with maximum (q - l_arr[q]) where d[q] != -1.
            let mut q = u as i32 - 1;
            while d[q as usize] == -1 && q > 0 {
                q -= 1;
            }
            if q > 0 {
                let mut j = q;
                loop {
                    j -= 1;
                    if d[j as usize] != -1 && u_lu[q as usize] < u_lu[j as usize] {
                        q = j;
                    }
                    if j <= 0 {
                        break;
                    }
                }
            }

            let q = q as usize;
            if l_arr[u] > l_arr[q] + u - q {
                l_arr[u + 1] = l_arr[u];
            } else {
                l_arr[u + 1] = l_arr[q] + u - q;
            }

            elp[u + 1][..t2].fill(0);
            for i in 0..=l_arr[q] {
                if elp[q][i] != -1 {
                    elp[u + 1][i + u - q] =
                        alpha_to[((d[u] + (n as i32) - d[q] + elp[q][i]) % (n as i32)) as usize];
                }
            }
            for i in 0..=l_arr[u] {
                elp[u + 1][i] ^= elp[u][i];
                if elp[u][i] >= 0 {
                    elp[u][i] = index_of[elp[u][i] as usize];
                }
            }
        }
        u_lu[u + 1] = u as i32 - l_arr[u + 1] as i32;

        if u < t2 {
            if s[u + 1] != -1 {
                d[u + 1] = alpha_to[s[u + 1] as usize];
            } else {
                d[u + 1] = 0;
            }
            for i in 1..=l_arr[u + 1] {
                if s[u + 1 - i] != -1 && elp[u + 1][i] != 0 {
                    d[u + 1] ^= alpha_to
                        [((s[u + 1 - i] + index_of[elp[u + 1][i] as usize]) % (n as i32)) as usize];
                }
            }
            if d[u + 1] >= 0 {
                d[u + 1] = index_of[d[u + 1] as usize];
            }
        }

        if u >= t2 || l_arr[u + 1] > t {
            break;
        }
    }

    u += 1;
    if l_arr[u] > t {
        arlog_d!(
            "bch_correct_errors: l[u]={} > t={} (uncorrectable)",
            l_arr[u],
            t
        );
        return Err("BCH correction failed (l > t)");
    }

    for i in 0..=l_arr[u] {
        if elp[u][i] >= 0 {
            elp[u][i] = index_of[elp[u][i] as usize];
        }
    }

    reg[1..=l_arr[u]].copy_from_slice(&elp[u][1..=l_arr[u]]);
    let mut count = 0;
    for i in 1..=n {
        let mut q_err = 1;
        for j in 1..=l_arr[u] {
            if reg[j] != -1 {
                reg[j] = (reg[j] + j as i32) % (n as i32);
                q_err ^= alpha_to[reg[j] as usize];
            }
        }
        if q_err == 0 {
            loc[count] = n - i;
            count += 1;
        }
    }

    if count != l_arr[u] {
        arlog_d!(
            "bch_correct_errors: count={} != l[u]={} (uncorrectable)",
            count,
            l_arr[u]
        );
        return Err("BCH correction failed (count != l)");
    }

    for i in 0..l_arr[u] {
        recd[loc[i]] ^= 1;
    }

    Ok(l_arr[u] as i32)
}

// =====================================================================
// Public BCH decoders.
// =====================================================================

/// Decodes a BCH-encoded matrix code (BCH(15, k, t) or BCH(31, k, t)).
///
/// Used for `Code4x4BCH*` and `Code5x5BCH*` matrix code variants.
/// For `AR_MATRIX_CODE_GLOBAL_ID`, use [`decode_bch_global_id`] instead, which
/// takes a pre-extracted 127-bit array (since 120 bits don't fit in `u64`).
pub fn decode_bch(
    matrix_code_type: ARMatrixCodeType,
    in_val: u64,
) -> Result<(u64, i32), &'static str> {
    let (t, k, n, length, alpha_to, index_of): (usize, usize, usize, usize, &[i32], &[i32]) =
        match matrix_code_type {
            ARMatrixCodeType::Code4x4BCH1393 => (1, 9, 15, 13, &BCH_15_ALPHA_TO, &BCH_15_INDEX_OF),
            ARMatrixCodeType::Code4x4BCH1355 => (2, 5, 15, 13, &BCH_15_ALPHA_TO, &BCH_15_INDEX_OF),
            ARMatrixCodeType::Code5x5BCH22125 => {
                (2, 12, 31, 22, &BCH_31_ALPHA_TO, &BCH_31_INDEX_OF)
            }
            ARMatrixCodeType::Code5x5BCH2277 => (3, 7, 31, 22, &BCH_31_ALPHA_TO, &BCH_31_INDEX_OF),
            _ => {
                arlog_e!(
                    "decode_bch: unsupported matrix code type {:?}",
                    matrix_code_type
                );
                return Err("Unsupported BCH code type");
            }
        };

    // Unpack u64 input into bit array (LSB at index 0). Buffer is sized for
    // the largest n we support (127) so the same buffer can flow into
    // `bch_correct_errors`, which uses `n` for `loc`-indexing internally.
    let mut recd = [0u8; 127];
    let mut in_bitwise = in_val;
    for r in recd[..length].iter_mut() {
        *r = (in_bitwise & 1) as u8;
        in_bitwise >>= 1;
    }

    let corrected = bch_correct_errors(&mut recd, t, n, length, alpha_to, index_of)?;

    // Repack the data bits into the output u64. Data bits live in the upper
    // `k` positions of the codeword: `recd[(length - k)..length]`.
    let mut out_p = 0u64;
    let mut out_bit = 1u64;
    for &r in &recd[(length - k)..length] {
        if r != 0 {
            out_p += out_bit;
        }
        out_bit <<= 1;
    }

    Ok((out_p, corrected))
}

/// Decodes a BCH(127, 64, 22) `AR_MATRIX_CODE_GLOBAL_ID` codeword in-place.
///
/// Mirrors the GLOBAL_ID branch of `decode_bch` in `arPattGetID.c`
/// (lines 1864-1870 in upstream): t=9, k=64, n=127, length=120.
///
/// # Parameters
/// - `recd127` — the 127-element bit array extracted from the marker grid by
///   [`crate::matrix::extract_global_id_bits`] (LSB at index 0). Mutated
///   in-place during error correction.
///
/// # Returns
/// - `Ok((global_id, errors_corrected))` on successful decode.
/// - `Err("BCH correction failed ...")` if more than 9 errors were detected.
///
/// # Notes
/// The output `global_id` is a 64-bit unsigned integer representing the
/// decoded marker identifier. Error correction can fix up to `t = 9` bit
/// errors per codeword (Hamming distance 22).
pub fn decode_bch_global_id(recd127: &mut [u8; 127]) -> Result<(u64, i32), &'static str> {
    const T: usize = 9;
    const K: usize = 64;
    const N: usize = 127;
    const LENGTH: usize = 120;

    let corrected =
        bch_correct_errors(recd127, T, N, LENGTH, &BCH_127_ALPHA_TO, &BCH_127_INDEX_OF)?;

    // Pack data bits from upper `k` positions of the codeword into a u64.
    let mut out_p = 0u64;
    let mut out_bit = 1u64;
    for &r in &recd127[(LENGTH - K)..LENGTH] {
        if r != 0 {
            out_p |= out_bit;
        }
        out_bit <<= 1;
    }

    Ok((out_p, corrected))
}

// =====================================================================
// Tests.
// =====================================================================

#[cfg(test)]
mod tests {
    use super::*;

    /// Builds the BCH(127, 64, 22) generator polynomial g(x) as a bit vector.
    ///
    /// `g(x) = ∏_{i=1..=2t} M_i(x)` where `M_i(x)` is the minimal polynomial
    /// of α^i over GF(2). Over GF(2), `M_(2j)(x) = M_j(x)`, so we only iterate
    /// over odd powers and skip duplicates via cyclotomic-coset tracking.
    ///
    /// Returns 64 generator coefficients: `g[0]` = constant term, `g[63]`
    /// = leading coefficient (degree 63).
    fn build_bch127_generator() -> Vec<u8> {
        const N: usize = 127;
        const T: usize = 9;

        // Track which conjugacy classes we've already covered. Each odd j in
        // [1, 2t] that hasn't been visited yet contributes one factor.
        let mut seen = [false; N];
        let mut g = vec![1u8]; // start with g(x) = 1

        for j in (1..=2 * T).filter(|j| j % 2 == 1) {
            if seen[j] {
                continue;
            }
            // Walk cyclotomic coset of j: {j, 2j, 4j, ...} mod N.
            let mut coset = Vec::new();
            let mut x = j;
            while !seen[x] {
                seen[x] = true;
                coset.push(x);
                x = (x * 2) % N;
            }

            // Build minimal polynomial M_j(x) = ∏ (x - α^i) for i in coset.
            // Multiply g by (x - α^i) iteratively in GF(2)[x] using GF(2^7)
            // coefficient arithmetic via BCH_127_ALPHA_TO / BCH_127_INDEX_OF.
            //
            // Working representation: polynomial coefficients are GF(2^7)
            // elements stored as i32 (0 represents zero element).
            let mut m_coeffs: Vec<i32> = vec![1]; // M(x) = 1 in GF(2^7)
            for &i in &coset {
                let alpha_i = BCH_127_ALPHA_TO[i] as i32;
                // Multiply m_coeffs by (x + α^i) (subtraction == addition in GF(2)).
                let mut new_m = vec![0i32; m_coeffs.len() + 1];
                for (idx, &c) in m_coeffs.iter().enumerate() {
                    new_m[idx + 1] ^= c;
                    new_m[idx] ^= gf128_mul(c, alpha_i);
                }
                m_coeffs = new_m;
            }
            // M_j(x) coefficients should all be in {0, 1} after the cyclotomic
            // coset is fully traversed (Galois conjugates collapse to GF(2)).
            // Convert to binary form.
            let m_bin: Vec<u8> = m_coeffs
                .iter()
                .map(|&c| {
                    debug_assert!(c == 0 || c == 1, "non-binary coefficient: {c}");
                    c as u8
                })
                .collect();

            // Multiply g by M_j: polynomial multiplication in GF(2)[x].
            let mut new_g = vec![0u8; g.len() + m_bin.len() - 1];
            for (i, &gi) in g.iter().enumerate() {
                if gi == 0 {
                    continue;
                }
                for (j, &mj) in m_bin.iter().enumerate() {
                    new_g[i + j] ^= mj;
                }
            }
            g = new_g;
        }

        // BCH(127, 64, 22) is a shortened BCH(127, 71). The generator polynomial
        // is determined by the BCH(127, 71) design — g(x) has degree
        // n - k_unshortened = 127 - 71 = 56 (i.e. 57 coefficients). Shortening
        // sets the 7 most significant message bits to zero but does not change g.
        debug_assert_eq!(g.len(), 57, "BCH(127,71) generator must have degree 56");
        g
    }

    /// GF(2^7) multiplication via the alpha_to/index_of tables.
    fn gf128_mul(a: i32, b: i32) -> i32 {
        if a == 0 || b == 0 {
            return 0;
        }
        let log_a = BCH_127_INDEX_OF[a as usize];
        let log_b = BCH_127_INDEX_OF[b as usize];
        BCH_127_ALPHA_TO[((log_a + log_b) % 127) as usize]
    }

    /// Encodes a 64-bit `global_id` into a 127-element BCH(127, 64, 22) codeword.
    ///
    /// Systematic encoding: `c(x) = m(x) · x^(n-k) + (m(x) · x^(n-k) mod g(x))`.
    /// - Message bits go into positions `[length - k .. length] = [56..120]`.
    /// - Parity bits go into positions `[0..56]`.
    /// - Positions `[120..127]` are zero (shortened code).
    ///
    /// The output layout matches what [`decode_bch_global_id`] expects.
    fn encode_bch_global_id(global_id: u64) -> [u8; 127] {
        const N_K: usize = 56; // 56 parity bits — degree of generator polynomial
        const K: usize = 64;
        const LENGTH: usize = 120;
        const G_DEG: usize = 56; // generator degree

        let g = build_bch127_generator(); // 57 binary coefficients (degree 56)

        // Build the dividend `m(x) · x^(n-k)`: message bits placed at high
        // positions [N_K..LENGTH], zeros elsewhere. Polynomial division of
        // this dividend by g(x) yields the remainder we need for the parity
        // section. The division process mutates the high positions too, so
        // we'll re-apply the original message after computing the remainder.
        let mut dividend = [0u8; 127];
        for i in 0..K {
            dividend[N_K + i] = ((global_id >> i) & 1) as u8;
        }

        // Long division: iterate from the highest non-zero position down,
        // eliminating each one by XOR-ing g(x) shifted so its leading term
        // (g[G_DEG] = 1) aligns with the current pivot.
        for i in (N_K..LENGTH).rev() {
            if dividend[i] == 1 {
                for (j, &gj) in g.iter().enumerate() {
                    if gj == 1 {
                        dividend[i - G_DEG + j] ^= 1;
                    }
                }
            }
        }

        // Assemble the systematic codeword: parity (= remainder) at low
        // positions, original message at high positions.
        let mut codeword = [0u8; 127];
        codeword[0..N_K].copy_from_slice(&dividend[0..N_K]);
        for i in 0..K {
            codeword[N_K + i] = ((global_id >> i) & 1) as u8;
        }

        codeword
    }

    // ---- decode_parity65 / decode_hamming63 sanity checks ----------------

    #[test]
    fn test_decode_parity65_zero() {
        assert_eq!(decode_parity65(0), Ok(0));
    }

    #[test]
    fn test_decode_hamming63_zero() {
        assert_eq!(decode_hamming63(0), Ok((0, 0)));
    }

    // ---- decode_bch (BCH-15 / BCH-31) sanity checks ----------------------

    #[test]
    fn test_decode_bch_4x4_zero_codeword() {
        // The all-zeros codeword is always valid → decodes to 0.
        let (id, errors) = decode_bch(ARMatrixCodeType::Code4x4BCH1393, 0).unwrap();
        assert_eq!(id, 0);
        assert_eq!(errors, 0);
    }

    #[test]
    fn test_decode_bch_5x5_zero_codeword() {
        let (id, errors) = decode_bch(ARMatrixCodeType::Code5x5BCH22125, 0).unwrap();
        assert_eq!(id, 0);
        assert_eq!(errors, 0);
    }

    // ---- BCH-127 generator polynomial sanity check -----------------------

    #[test]
    fn test_bch127_generator_degree() {
        let g = build_bch127_generator();
        // BCH(127, 64, 22) is a shortened BCH(127, 71). g(x) has degree 56,
        // determined by the LCM of minimal polynomials M_1..M_15 over GF(2^7).
        assert_eq!(g.len(), 57);
        // Leading and constant coefficients must be 1.
        assert_eq!(g[56], 1);
        assert_eq!(g[0], 1);
    }

    // ---- decode_bch_global_id tests --------------------------------------

    #[test]
    fn test_decode_bch_global_id_zero_codeword() {
        // All-zeros input is a valid codeword → decodes to 0 with no errors.
        let mut recd = [0u8; 127];
        let (id, errors) = decode_bch_global_id(&mut recd).unwrap();
        assert_eq!(id, 0);
        assert_eq!(errors, 0);
    }

    #[test]
    fn test_decode_bch_global_id_roundtrip_known_pattern() {
        // Encode → decode roundtrip for a hand-picked global_id.
        let global_id = 0x1234_5678_DEAD_BEEF_u64;
        let mut recd = encode_bch_global_id(global_id);
        let (decoded, errors) = decode_bch_global_id(&mut recd).unwrap();
        assert_eq!(decoded, global_id);
        assert_eq!(errors, 0);
    }

    #[test]
    fn test_decode_bch_global_id_roundtrip_max_value() {
        let global_id = u64::MAX;
        let mut recd = encode_bch_global_id(global_id);
        let (decoded, errors) = decode_bch_global_id(&mut recd).unwrap();
        assert_eq!(decoded, global_id);
        assert_eq!(errors, 0);
    }

    #[test]
    fn test_decode_bch_global_id_single_bit_error() {
        // Flip a single bit in the parity region → BCH must correct it.
        let global_id = 42u64;
        let mut recd = encode_bch_global_id(global_id);
        recd[7] ^= 1;
        let (decoded, errors) = decode_bch_global_id(&mut recd).unwrap();
        assert_eq!(decoded, global_id);
        assert_eq!(errors, 1);
    }

    #[test]
    fn test_decode_bch_global_id_data_bit_error() {
        // Flip a single bit in the data region → BCH must correct it.
        let global_id = 0xCAFE_BABE_u64;
        let mut recd = encode_bch_global_id(global_id);
        recd[80] ^= 1;
        let (decoded, errors) = decode_bch_global_id(&mut recd).unwrap();
        assert_eq!(decoded, global_id);
        assert_eq!(errors, 1);
    }

    #[test]
    fn test_decode_bch_global_id_nine_bit_errors_correctable() {
        // BCH(127, 64, 22) corrects up to t = 9 errors. At the boundary,
        // decoding must still succeed.
        let global_id = 0xABCD_1234_5678_9ABC_u64;
        let mut recd = encode_bch_global_id(global_id);
        for i in 0..9 {
            recd[i * 13] ^= 1; // spread errors out (positions 0, 13, 26, ...)
        }
        let (decoded, errors) = decode_bch_global_id(&mut recd).unwrap();
        assert_eq!(decoded, global_id);
        assert_eq!(errors, 9);
    }

    #[test]
    fn test_decode_bch_global_id_too_many_errors_fails() {
        // Flipping 15 well-spread bits exceeds t = 9. Decoder must error.
        let global_id = 99u64;
        let mut recd = encode_bch_global_id(global_id);
        for i in 0..15 {
            recd[i * 7] ^= 1;
        }
        let result = decode_bch_global_id(&mut recd);
        assert!(result.is_err(), "expected uncorrectable error pattern");
    }
}
