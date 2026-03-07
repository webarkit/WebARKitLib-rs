/*
 *  pattern.rs
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

//! Pattern Template matching logic
//! Ported natively to safe Rust from arPattLoad.c and arPattGetID.c

use crate::types::{ARPattHandle, ARdouble};

pub const AR_PATT_NUM_MAX: i32 = 50;
pub const AR_PATT_SIZE1: i32 = 16;
pub const AR_TEMPLATE_MATCHING_COLOR: i32 = 0;
pub const AR_TEMPLATE_MATCHING_MONO: i32 = 1;
pub const AR_PATT_CONTRAST_THRESH1: ARdouble = 5.0;

impl ARPattHandle {
    /// Creates a new Handle for pattern matching with pre-allocated buffer capacities.
    pub fn new(patt_size: i32, pattern_count_max: i32) -> Self {
        let max_alloc = (pattern_count_max * 4) as usize;
        Self {
            patt_num: 0,
            patt_num_max: pattern_count_max,
            pattf: vec![0; pattern_count_max as usize],
            patt: vec![vec![0; (patt_size * patt_size * 3) as usize]; max_alloc],
            pattpow: vec![0.0; max_alloc],
            patt_bw: vec![vec![0; (patt_size * patt_size) as usize]; max_alloc],
            pattpow_bw: vec![0.0; max_alloc],
            patt_size,
        }
    }
}

/// Loads a pattern into the provided ARPattHandle from a text-based buffer.
/// Returns the index of the loaded pattern.
pub fn ar_patt_load_from_buffer(patt_handle: &mut ARPattHandle, buffer: &str) -> Result<i32, &'static str> {
    let mut tokens = buffer.split_whitespace();
    
    let mut patno = -1;
    for i in 0..patt_handle.patt_num_max as usize {
        if patt_handle.pattf[i] == 0 {
            patno = i as i32;
            break;
        }
    }
    
    if patno == -1 {
        return Err("Maximum pattern limit reached");
    }

    let p_idx = patno as usize;
    let size = patt_handle.patt_size as usize;

    for h in 0..4 {
        let mut l = 0;
        let mut i_out = 0;
        
        let mut read_tokens = Vec::with_capacity(size * size * 3);
        
        for _ in 0..(size * size * 3) {
            if let Some(t) = tokens.next() {
                if let Ok(val) = t.parse::<i32>() {
                    read_tokens.push(val);
                } else {
                    return Err("Failed to parse pattern number");
                }
            } else {
                return Err("Pattern data read error (unexpected EOF)");
            }
        }
        
        for i3 in 0..3 {
            for i2 in 0..size {
                for i1 in 0..size {
                    let mut j = read_tokens[i_out];
                    i_out += 1;
                    
                    j = 255 - j;
                    patt_handle.patt[p_idx * 4 + h][(i2 * size + i1) * 3 + i3] = j;
                    
                    if i3 == 0 {
                        patt_handle.patt_bw[p_idx * 4 + h][i2 * size + i1] = j;
                    } else {
                        patt_handle.patt_bw[p_idx * 4 + h][i2 * size + i1] += j;
                    }
                    
                    if i3 == 2 {
                        patt_handle.patt_bw[p_idx * 4 + h][i2 * size + i1] /= 3;
                    }
                    l += j;
                }
            }
        }
        
        l /= (size * size * 3) as i32;

        let mut m_col = 0i64;
        for i in 0..(size * size * 3) {
            patt_handle.patt[p_idx * 4 + h][i] -= l;
            m_col += (patt_handle.patt[p_idx * 4 + h][i] * patt_handle.patt[p_idx * 4 + h][i]) as i64;
        }
        
        patt_handle.pattpow[p_idx * 4 + h] = (m_col as ARdouble).sqrt();
        if patt_handle.pattpow[p_idx * 4 + h] == 0.0 {
            patt_handle.pattpow[p_idx * 4 + h] = 0.0000001;
        }

        let mut m_bw = 0i64;
        for i in 0..(size * size) {
            patt_handle.patt_bw[p_idx * 4 + h][i] -= l;
            m_bw += (patt_handle.patt_bw[p_idx * 4 + h][i] * patt_handle.patt_bw[p_idx * 4 + h][i]) as i64;
        }
        
        patt_handle.pattpow_bw[p_idx * 4 + h] = (m_bw as ARdouble).sqrt();
        if patt_handle.pattpow_bw[p_idx * 4 + h] == 0.0 {
            patt_handle.pattpow_bw[p_idx * 4 + h] = 0.0000001;
        }
    }

    patt_handle.pattf[p_idx] = 1;
    patt_handle.patt_num += 1;

    Ok(patno)
}

/// Loads a pattern from a specified file path.
pub fn ar_patt_load(patt_handle: &mut ARPattHandle, filename: &str) -> Result<i32, String> {
    use std::fs;
    
    let buffer = fs::read_to_string(filename)
        .map_err(|e| format!("Error reading pattern file '{}': {}", filename, e))?;
        
    ar_patt_load_from_buffer(patt_handle, &buffer).map_err(|e| e.to_string())
}

/// Matches the unwarped square marker against loaded templates via NCC (Normalized Cross Correlation).
pub fn pattern_match(
    patt_handle: &ARPattHandle,
    mode: i32,
    data: &[u8],
    size: i32,
    code: &mut i32,
    dir: &mut i32,
    cf: &mut ARdouble,
) -> Result<(), &'static str> {
    if size <= 0 {
        *code = 0;
        *dir = 0;
        *cf = -1.0;
        return Err("Invalid size");
    }

    let size_u = size as usize;

    if mode == AR_TEMPLATE_MATCHING_COLOR {
        let size_sqd_x3 = size_u * size_u * 3;
        if data.len() < size_sqd_x3 {
            return Err("Data array too small");
        }
        let mut input = vec![0; size_sqd_x3];

        let mut ave = 0i32;
        for i in 0..size_sqd_x3 {
            ave += (255 - data[i]) as i32;
        }
        ave /= size_sqd_x3 as i32;

        let mut sum = 0i64;
        for i in 0..size_sqd_x3 {
            input[i] = (255 - data[i]) as i32 - ave;
            sum += (input[i] * input[i]) as i64;
        }

        let datapow = (sum as ARdouble).sqrt();
        if datapow / ((size as ARdouble) * 3.0f64.sqrt()) < AR_PATT_CONTRAST_THRESH1 {
            *code = 0;
            *dir = 0;
            *cf = -1.0;
            return Err("Insufficient contrast");
        }

        let mut res1 = -1;
        let mut res2 = -1;
        let mut max = 0.0;
        
        let mut k = -1isize;
        for _ in 0..patt_handle.patt_num {
            k += 1;
            while (k as usize) < patt_handle.pattf.len() && patt_handle.pattf[k as usize] == 0 {
                k += 1;
            }
            if k as usize >= patt_handle.pattf.len() { break; }
            if patt_handle.pattf[k as usize] == 2 {
                continue;
            }
            
            for j in 0..4 {
                let pattern_ref = &patt_handle.patt[k as usize * 4 + j];
                let sum_cc = dot_product(&input, pattern_ref);
                
                let sum2 = (sum_cc as ARdouble) / patt_handle.pattpow[k as usize * 4 + j] / datapow;
                if sum2 > max {
                    max = sum2;
                    res1 = j as i32;
                    res2 = k as i32;
                }
            }
        }

        *dir = res1;
        *code = res2;
        *cf = max;
        Ok(())
    } else if mode == AR_TEMPLATE_MATCHING_MONO {
        let size_sqd = size_u * size_u;
        if data.len() < size_sqd {
            return Err("Data array too small");
        }
        
        let mut input = vec![0; size_sqd];

        let mut ave = 0i32;
        for i in 0..size_sqd {
            ave += (255 - data[i]) as i32;
        }
        ave /= size_sqd as i32;

        let mut sum = 0i64;
        for i in 0..size_sqd {
            input[i] = (255 - data[i]) as i32 - ave;
            sum += (input[i] * input[i]) as i64;
        }

        let datapow = (sum as ARdouble).sqrt();
        if datapow / (size as ARdouble) < AR_PATT_CONTRAST_THRESH1 {
            *code = 0;
            *dir = 0;
            *cf = -1.0;
            return Err("Insufficient contrast");
        }

        let mut res1 = -1;
        let mut res2 = -1;
        let mut max = 0.0;
        
        let mut k = -1isize;
        for _ in 0..patt_handle.patt_num {
            k += 1;
            while (k as usize) < patt_handle.pattf.len() && patt_handle.pattf[k as usize] == 0 {
                k += 1;
            }
            if k as usize >= patt_handle.pattf.len() { break; }
            if patt_handle.pattf[k as usize] == 2 {
                continue;
            }
            
            for j in 0..4 {
                let pattern_ref = &patt_handle.patt_bw[k as usize * 4 + j];
                let sum_cc = dot_product(&input, pattern_ref);
                
                let sum2 = (sum_cc as ARdouble) / patt_handle.pattpow_bw[k as usize * 4 + j] / datapow;
                if sum2 > max {
                    max = sum2;
                    res1 = j as i32;
                    res2 = k as i32;
                }
            }
        }

        *dir = res1;
        *code = res2;
        *cf = max;
        Ok(())
    } else {
        Err("Unsupported matching mode")
    }
}

pub fn get_cpara(world: &[[ARdouble; 2]; 4], vertex: &[[ARdouble; 2]; 4], para: &mut [[ARdouble; 3]; 3]) -> Result<(), &'static str> {
    use crate::math::{ARMat};
    let mut a = ARMat::new(8, 8);
    let mut b = ARMat::new(8, 1);

    for i in 0..4 {
        a.m[i * 16 + 0] = world[i][0];
        a.m[i * 16 + 1] = world[i][1];
        a.m[i * 16 + 2] = 1.0;
        a.m[i * 16 + 3] = 0.0;
        a.m[i * 16 + 4] = 0.0;
        a.m[i * 16 + 5] = 0.0;
        a.m[i * 16 + 6] = -world[i][0] * vertex[i][0];
        a.m[i * 16 + 7] = -world[i][1] * vertex[i][0];
        
        a.m[i * 16 + 8] = 0.0;
        a.m[i * 16 + 9] = 0.0;
        a.m[i * 16 + 10] = 0.0;
        a.m[i * 16 + 11] = world[i][0];
        a.m[i * 16 + 12] = world[i][1];
        a.m[i * 16 + 13] = 1.0;
        a.m[i * 16 + 14] = -world[i][0] * vertex[i][1];
        a.m[i * 16 + 15] = -world[i][1] * vertex[i][1];

        b.m[i * 2 + 0] = vertex[i][0];
        b.m[i * 2 + 1] = vertex[i][1];
    }

    a.self_inv()?;
    let c = (&a * &b)?;

    for i in 0..2 {
        para[i][0] = c.m[i * 3 + 0];
        para[i][1] = c.m[i * 3 + 1];
        para[i][2] = c.m[i * 3 + 2];
    }
    para[2][0] = c.m[6];
    para[2][1] = c.m[7];
    para[2][2] = 1.0;

    Ok(())
}

pub fn ar_patt_get_image(
    image_proc_mode: i32,
    patt_detect_mode: i32,
    patt_size: i32,
    sample_size: i32,
    image: &[u8],
    xsize: i32,
    ysize: i32,
    _pixel_format: crate::types::ARPixelFormat,
    vertex: &[[ARdouble; 2]; 4],
    patt_ratio: ARdouble,
    ext_patt: &mut [u8],
) -> Result<(), &'static str> {
    let mut world = [[0.0; 2]; 4];
    let mut para = [[0.0; 3]; 3];

    world[0][0] = 100.0;
    world[0][1] = 100.0;
    world[1][0] = 110.0;
    world[1][1] = 100.0;
    world[2][0] = 110.0;
    world[2][1] = 110.0;
    world[3][0] = 100.0;
    world[3][1] = 110.0;

    get_cpara(&world, vertex, &mut para)?;

    let mut lx1 = ((vertex[0][0] - vertex[1][0]).powi(2) + (vertex[0][1] - vertex[1][1]).powi(2)) as i32;
    let lx2 = ((vertex[2][0] - vertex[3][0]).powi(2) + (vertex[2][1] - vertex[3][1]).powi(2)) as i32;
    let mut ly1 = ((vertex[1][0] - vertex[2][0]).powi(2) + (vertex[1][1] - vertex[2][1]).powi(2)) as i32;
    let ly2 = ((vertex[3][0] - vertex[0][0]).powi(2) + (vertex[3][1] - vertex[0][1]).powi(2)) as i32;

    if lx2 > lx1 { lx1 = lx2; }
    if ly2 > ly1 { ly1 = ly2; }

    let lx_patt = (lx1 as ARdouble * patt_ratio * patt_ratio) as i32;
    let ly_patt = (ly1 as ARdouble * patt_ratio * patt_ratio) as i32;

    let mut xdiv2 = patt_size;
    let mut ydiv2 = patt_size;

    if image_proc_mode == 0 { // AR_IMAGE_PROC_FRAME_IMAGE
        while xdiv2 * xdiv2 < lx_patt && xdiv2 < sample_size { xdiv2 *= 2; }
        while ydiv2 * ydiv2 < ly_patt && ydiv2 < sample_size { ydiv2 *= 2; }
    } else {
        while xdiv2 * xdiv2 * 4 < lx_patt && xdiv2 < sample_size { xdiv2 *= 2; }
        while ydiv2 * ydiv2 * 4 < ly_patt && ydiv2 < sample_size { ydiv2 *= 2; }
    }
    
    if xdiv2 > sample_size { xdiv2 = sample_size; }
    if ydiv2 > sample_size { ydiv2 = sample_size; }

    let xdiv = xdiv2 / patt_size;
    let ydiv = ydiv2 / patt_size;
    
    let patt_ratio1 = (1.0 - patt_ratio) / 2.0 * 10.0;
    let patt_ratio2 = patt_ratio * 10.0;

    if patt_detect_mode == AR_TEMPLATE_MATCHING_COLOR {
        let mut ext_patt2 = vec![0u32; (patt_size * patt_size * 3) as usize];
        
        for j in 0..ydiv2 {
            let yw = (100.0 + patt_ratio1) + patt_ratio2 * (j as f64 + 0.5) / (ydiv2 as f64);
            for i in 0..xdiv2 {
                let xw = (100.0 + patt_ratio1) + patt_ratio2 * (i as f64 + 0.5) / (xdiv2 as f64);
                let d = para[2][0] * xw + para[2][1] * yw + para[2][2];
                if d == 0.0 { return Err("Division by zero in homography"); }
                
                let xc = ((para[0][0] * xw + para[0][1] * yw + para[0][2]) / d) as i32;
                let yc = ((para[1][0] * xw + para[1][1] * yw + para[1][2]) / d) as i32;

                if xc >= 0 && xc < xsize && yc >= 0 && yc < ysize {
                    // RGB assumes 3 bytes per pixel in the source buffer
                    let src_idx = ((yc * xsize + xc) * 3) as usize;
                    if src_idx + 2 < image.len() {
                        let dst_idx = (((j / ydiv) * patt_size + (i / xdiv)) * 3) as usize;
                        ext_patt2[dst_idx + 0] += image[src_idx + 0] as u32; // R
                        ext_patt2[dst_idx + 1] += image[src_idx + 1] as u32; // G
                        ext_patt2[dst_idx + 2] += image[src_idx + 2] as u32; // B
                    }
                }
            }
        }
        
        for i in 0..(patt_size * patt_size * 3) as usize {
            if i < ext_patt.len() {
                ext_patt[i] = (ext_patt2[i] / (xdiv * ydiv) as u32) as u8;
            }
        }
    } else {
        let mut ext_patt2 = vec![0u32; (patt_size * patt_size) as usize];
        
        for j in 0..ydiv2 {
            let yw = (100.0 + patt_ratio1) + patt_ratio2 * (j as f64 + 0.5) / (ydiv2 as f64);
            for i in 0..xdiv2 {
                let xw = (100.0 + patt_ratio1) + patt_ratio2 * (i as f64 + 0.5) / (xdiv2 as f64);
                let d = para[2][0] * xw + para[2][1] * yw + para[2][2];
                if d == 0.0 { return Err("Division by zero in homography"); }
                
                let xc = ((para[0][0] * xw + para[0][1] * yw + para[0][2]) / d) as i32;
                let yc = ((para[1][0] * xw + para[1][1] * yw + para[1][2]) / d) as i32;

                if xc >= 0 && xc < xsize && yc >= 0 && yc < ysize {
                    // Assuming Luma takes 1 byte per pixel! Wait, if image is RGB, Luma would be 3 bytes?
                    // if it's RGB buffer, we must convert to luma. Assuming Luma/Mono buffer passed in natively:
                    let src_idx = (yc * xsize + xc) as usize;
                    if src_idx < image.len() {
                        let dst_idx = ((j / ydiv) * patt_size + (i / xdiv)) as usize;
                        ext_patt2[dst_idx] += image[src_idx] as u32;
                    }
                }
            }
        }
        
        for i in 0..(patt_size * patt_size) as usize {
            if i < ext_patt.len() {
                ext_patt[i] = (ext_patt2[i] / (xdiv * ydiv) as u32) as u8;
            }
        }
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pattern_load_and_match() {
        let mut handle = ARPattHandle::new(AR_PATT_SIZE1, AR_PATT_NUM_MAX);
        
        let mut mock_patt = String::new();
        // 4 orientations * size * size * 3 colors
        for _ in 0..(4 * AR_PATT_SIZE1 * AR_PATT_SIZE1 * 3) {
            mock_patt.push_str("128 ");
        }
        
        let patno = ar_patt_load_from_buffer(&mut handle, &mock_patt).unwrap();
        assert_eq!(patno, 0);
        assert_eq!(handle.patt_num, 1);
        
        // Mock extracted pattern with high contrast
        let mut mock_data = vec![0; (AR_PATT_SIZE1 * AR_PATT_SIZE1 * 3) as usize];
        for i in 0..mock_data.len() {
            if i % 2 == 0 {
                mock_data[i] = 10;
            } else {
                mock_data[i] = 240;
            }
        }
        
        let mut code = 0;
        let mut dir = 0;
        let mut cf = 0.0;
        let result = pattern_match(&handle, AR_TEMPLATE_MATCHING_COLOR, &mock_data, AR_PATT_SIZE1, &mut code, &mut dir, &mut cf);
        assert!(result.is_ok());
    }
}

#[inline]
fn dot_product(a: &[i32], b: &[i32]) -> i64 {
    #[cfg(feature = "simd-pattern")]
    {
        #[cfg(all(target_arch = "wasm32", target_feature = "simd128"))]
        {
            return unsafe { dot_product_simd_wasm(a, b) };
        }
        #[cfg(all(target_arch = "x86_64", target_feature = "sse4.1"))]
        {
            if is_x86_feature_detected!("sse4.1") {
                return unsafe { dot_product_simd_x86(a, b) };
            }
        }
    }

    #[cfg(not(any(
        all(target_arch = "wasm32", target_feature = "simd128"),
        all(target_arch = "x86_64", target_feature = "sse4.1")
    )))]
    dot_product_scalar(a, b)
}

pub fn dot_product_scalar(a: &[i32], b: &[i32]) -> i64 {
    let mut sum = 0i64;
    for i in 0..a.len() {
        sum += (a[i] * b[i]) as i64;
    }
    sum
}

#[cfg(all(feature = "simd-pattern", target_arch = "wasm32", target_feature = "simd128"))]
#[target_feature(enable = "simd128")]
unsafe fn dot_product_simd_wasm(a: &[i32], b: &[i32]) -> i64 {
    use std::arch::wasm32::*;
    
    let mut sum_v = i64x2_splat(0);
    let chunks_len = a.len() / 4;
    
    let mut a_ptr = a.as_ptr();
    let mut b_ptr = b.as_ptr();
    
    for _ in 0..chunks_len {
        let va = v128_load(a_ptr as *const v128);
        let vb = v128_load(b_ptr as *const v128);
        
        // Low parts
        let va_low = i64x2_extend_low_i32x4(va);
        let vb_low = i64x2_extend_low_i32x4(vb);
        sum_v = i64x2_add(sum_v, i64x2_mul(va_low, vb_low));
        
        // High parts
        let va_high = i64x2_extend_high_i32x4(va);
        let vb_high = i64x2_extend_high_i32x4(vb);
        sum_v = i64x2_add(sum_v, i64x2_mul(va_high, vb_high));
        
        a_ptr = a_ptr.add(4);
        b_ptr = b_ptr.add(4);
    }
    
    let mut res = [0i64; 2];
    v128_store(res.as_mut_ptr() as *mut v128, sum_v);
    let mut total = res[0] + res[1];
    
    let rem_start = chunks_len * 4;
    for i in rem_start..a.len() {
        total += (a[i] * b[i]) as i64;
    }
    
    total
}

#[cfg(all(feature = "simd-pattern", target_arch = "x86_64", target_feature = "sse4.1"))]
#[target_feature(enable = "sse4.1")]
pub unsafe fn dot_product_simd_x86(a: &[i32], b: &[i32]) -> i64 {
    use std::arch::x86_64::*;
    
    let mut sum_v = _mm_setzero_si128(); // i64x2
    let chunks_len = a.len() / 4;
    
    let mut a_ptr = a.as_ptr();
    let mut b_ptr = b.as_ptr();
    
    for _ in 0..chunks_len {
        let va = _mm_loadu_si128(a_ptr as *const __m128i);
        let vb = _mm_loadu_si128(b_ptr as *const __m128i);
        
        // Use 32-bit multiplication (SSE4.1). 
        // This is safe for pattern matching where (255*255)*768 << 2^31.
        let prod = _mm_mullo_epi32(va, vb);
        sum_v = _mm_add_epi32(sum_v, prod);
        
        a_ptr = a_ptr.add(4);
        b_ptr = b_ptr.add(4);
    }
    
    let mut res = [0i32; 4];
    _mm_storeu_si128(res.as_mut_ptr() as *mut __m128i, sum_v);
    let mut total = (res[0] as i64) + (res[1] as i64) + (res[2] as i64) + (res[3] as i64);
    
    let rem_start = chunks_len * 4;
    for i in rem_start..a.len() {
        total += (a[i] * b[i]) as i64;
    }
    
    total
}
