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

pub fn decode_bch(
    matrix_code_type: ARMatrixCodeType,
    in_val: u64,
) -> Result<(u64, i32), &'static str> {
    let t: usize;
    let k: usize;
    let n: usize;
    let length: usize;
    let alpha_to: &[i32];
    let index_of: &[i32];

    const BCH_15_ALPHA_TO: [i32; 15] = [1, 2, 4, 8, 3, 6, 12, 11, 5, 10, 7, 14, 15, 13, 9];
    const BCH_15_INDEX_OF: [i32; 16] = [-1, 0, 1, 4, 2, 8, 5, 10, 3, 14, 9, 7, 6, 13, 11, 12];
    const BCH_31_ALPHA_TO: [i32; 31] = [
        1, 2, 4, 8, 16, 5, 10, 20, 13, 26, 17, 7, 14, 28, 29, 31, 27, 19, 3, 6, 12, 24, 21, 15, 30,
        25, 23, 11, 22, 9, 18,
    ];
    const BCH_31_INDEX_OF: [i32; 32] = [
        -1, 0, 1, 18, 2, 5, 19, 11, 3, 29, 6, 27, 20, 8, 12, 23, 4, 10, 30, 17, 7, 22, 28, 26, 21,
        25, 9, 16, 13, 14, 24, 15,
    ];

    let mut recd = [0u8; 127];

    match matrix_code_type {
        ARMatrixCodeType::Code4x4BCH1393 | ARMatrixCodeType::Code4x4BCH1355 => {
            if matrix_code_type == ARMatrixCodeType::Code4x4BCH1393 {
                t = 1;
                k = 9;
            } else {
                t = 2;
                k = 5;
            }
            n = 15;
            length = 13;
            alpha_to = &BCH_15_ALPHA_TO;
            index_of = &BCH_15_INDEX_OF;
        }
        ARMatrixCodeType::Code5x5BCH22125 | ARMatrixCodeType::Code5x5BCH2277 => {
            if matrix_code_type == ARMatrixCodeType::Code5x5BCH22125 {
                t = 2;
                k = 12;
            } else {
                t = 3;
                k = 7;
            }
            n = 31;
            length = 22;
            alpha_to = &BCH_31_ALPHA_TO;
            index_of = &BCH_31_INDEX_OF;
        }
        _ => {
            arlog_e!(
                "decode_bch: unsupported matrix code type {:?}",
                matrix_code_type
            );
            return Err("Unsupported BCH code type");
        }
    }

    let mut in_bitwise = in_val;
    for i in 0..length {
        recd[i] = (in_bitwise & 1) as u8;
        in_bitwise >>= 1;
    }

    let mut syn_error = false;
    let t2 = 2 * t;
    let mut s = vec![0; t2 + 1];

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
    if syn_error {
        let mut elp = vec![vec![0; 18]; 20];
        let mut d = vec![0; 20];
        let mut u_lu = vec![0i32; 20];
        let mut loc = vec![0usize; 127];
        let mut reg = vec![0; 10];

        d[0] = 0;
        d[1] = s[1];
        elp[0][0] = 0;
        elp[1][0] = 1;
        for i in 1..t2 {
            elp[0][i] = -1;
            elp[1][i] = 0;
        }
        l_arr[0] = 0;
        l_arr[1] = 0;
        u_lu[0] = -1;
        u_lu[1] = 0;
        let mut u = 0;

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

                for i in 0..t2 {
                    elp[u + 1][i] = 0;
                }
                for i in 0..=l_arr[q] {
                    if elp[q][i] != -1 {
                        elp[u + 1][i + u - q] = alpha_to
                            [((d[u] + (n as i32) - d[q] + elp[q][i]) % (n as i32)) as usize];
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
                        d[u + 1] ^= alpha_to[((s[u + 1 - i] + index_of[elp[u + 1][i] as usize])
                            % (n as i32)) as usize];
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
        if l_arr[u] <= t {
            for i in 0..=l_arr[u] {
                if elp[u][i] >= 0 {
                    elp[u][i] = index_of[elp[u][i] as usize];
                }
            }

            for i in 1..=l_arr[u] {
                reg[i] = elp[u][i];
            }
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

            if count == l_arr[u] {
                for i in 0..l_arr[u] {
                    recd[loc[i]] ^= 1;
                }
            } else {
                arlog_d!("decode_bch: count != l[u] (in_val={:#x})", in_val);
                return Err("BCH correction failed (count != l)");
            }
        } else {
            arlog_d!("decode_bch: l[u] > t (in_val={:#x})", in_val);
            return Err("BCH correction failed (l > t)");
        }
    }

    let mut out_p = 0u64;
    let mut out_bit = 1u64;
    for i in (length - k)..length {
        if recd[i] != 0 {
            out_p += out_bit;
        }
        out_bit <<= 1;
    }

    let corrected = if syn_error {
        l_arr[s.len() - 1] as i32
    } else {
        0
    };
    Ok((out_p, corrected))
}
