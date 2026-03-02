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
                let mut sum_cc = 0i64;
                let pattern_ref = &patt_handle.patt[k as usize * 4 + j];
                
                for i in 0..size_sqd_x3 {
                    sum_cc += (input[i] * pattern_ref[i]) as i64;
                }
                
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
                let mut sum_cc = 0i64;
                let pattern_ref = &patt_handle.patt_bw[k as usize * 4 + j];
                
                for i in 0..size_sqd {
                    sum_cc += (input[i] * pattern_ref[i]) as i64;
                }
                
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
