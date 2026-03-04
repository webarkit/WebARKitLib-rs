/*
 *  image_proc.rs
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

//! Image Processing and Thresholding Utilities for WebARKitLib
//! Translated from ARToolKit C headers (arImageProc.h, arImageProc.c)

/// Structure holding settings for an instance of the image-processing pipeline
#[derive(Debug, Clone)]
pub struct ARImageProcInfo {
    /// Width of image buffer
    pub image_x: i32,
    /// Height of image buffer
    pub image_y: i32,
    /// Extra buffer, allocated as required for filtering
    pub image2: Option<Vec<u8>>,
    /// Intermediate buffer for separable filters
    pub image_temp_u16: Option<Vec<u16>>,
    /// Luminance histogram
    pub hist_bins: [u32; 256],
    /// Luminance cumulative density function
    pub cdf_bins: [u32; 256],
    /// Minimum luminance
    pub min: u8,
    /// Maximum luminance
    pub max: u8,
}

impl ARImageProcInfo {
    /// Initialise image processing
    pub fn new(xsize: i32, ysize: i32) -> Self {
        Self {
            image_x: xsize,
            image_y: ysize,
            image2: None,
            image_temp_u16: None,
            hist_bins: [0; 256],
            cdf_bins: [0; 256],
            min: 0,
            max: 0,
        }
    }

    /// Calculate luminance histogram
    pub fn luma_hist(&mut self, data: &[u8]) -> Result<(), &'static str> {
        let expected_size = (self.image_x * self.image_y) as usize;
        if data.len() < expected_size {
            return Err("Data array is smaller than the specified image dimensions");
        }

        self.hist_bins.fill(0);
        for &pixel in data.iter().take(expected_size) {
            self.hist_bins[pixel as usize] += 1;
        }

        Ok(())
    }

    /// Calculate image histogram and cumulative density function
    pub fn luma_hist_and_cdf(&mut self, data: &[u8]) -> Result<(), &'static str> {
        self.luma_hist(data)?;

        let mut cdf_current = 0;
        for i in 0..=255 {
            self.cdf_bins[i] = cdf_current + self.hist_bins[i];
            cdf_current = self.cdf_bins[i];
        }

        Ok(())
    }

    /// Calculate image histogram, cumulative density function, and luminance value at a given histogram percentile
    pub fn luma_hist_and_cdf_and_percentile(&mut self, data: &[u8], percentile: f32) -> Result<u8, &'static str> {
        if !(0.0..=1.0).contains(&percentile) {
            return Err("Percentile must be between 0.0 and 1.0");
        }

        self.luma_hist_and_cdf(data)?;

        let count = (self.image_x * self.image_y) as f32;
        let required_cd = (count * percentile) as u32;

        let mut i = 0;
        while i < 256 && self.cdf_bins[i] < required_cd {
            i += 1;
        }

        let mut j = i;
        while j < 256 && self.cdf_bins[j] == required_cd {
            j += 1;
        }

        Ok(((i + j) / 2) as u8)
    }

    /// Calculate image histogram, cumulative density function, and median luminance value
    pub fn luma_hist_and_cdf_and_median(&mut self, data: &[u8]) -> Result<u8, &'static str> {
        self.luma_hist_and_cdf_and_percentile(data, 0.5)
    }

    /// Calculate image histogram, and binarize image using Otsu's method
    pub fn luma_hist_and_otsu(&mut self, data: &[u8]) -> Result<u8, &'static str> {
        self.luma_hist(data)?;

        let mut sum = 0.0;
        for i in 1..=255 {
            sum += (self.hist_bins[i] * i as u32) as f32;
        }

        let count = (self.image_x * self.image_y) as f32;
        let mut sum_b = 0.0;
        let mut w_b = 0.0;
        let mut var_max = 0.0;
        let mut threshold = 0;

        for i in 0..=255 {
            w_b += self.hist_bins[i] as f32;
            if w_b == 0.0 { continue; }

            let w_f = count - w_b;
            if w_f == 0.0 { break; }

            sum_b += (i as u32 * self.hist_bins[i]) as f32;

            let m_b = sum_b / w_b;
            let m_f = (sum - sum_b) / w_f;

            let var_between = w_b * w_f * (m_b - m_f) * (m_b - m_f);

            if var_between > var_max {
                var_max = var_between;
                threshold = i as u8;
            }
        }

        Ok(threshold)
    }

    /// Calculate image histogram, cumulative density function, and minimum and maximum luminance values
    pub fn luma_hist_and_cdf_and_levels(&mut self, data: &[u8]) -> Result<(), &'static str> {
        self.luma_hist_and_cdf(data)?;

        let mut l = 0;
        while l < 256 && self.cdf_bins[l] == 0 {
            l += 1;
        }
        self.min = l as u8;

        let max_cd = (self.image_x * self.image_y) as u32;
        while l < 256 && self.cdf_bins[l] < max_cd {
            l += 1;
        }
        self.max = l as u8;

        Ok(())
    }

    /// Calculate image histogram, and box filter image
    pub fn luma_hist_and_box_filter_with_bias(&mut self, data: &[u8], box_size: i32, bias: i32) -> Result<(), &'static str> {
        self.luma_hist(data)?;

        let img_size = (self.image_x * self.image_y) as usize;
        if self.image2.is_none() || self.image2.as_ref().unwrap().len() != img_size {
            self.image2 = Some(vec![0; img_size]);
        }
        if self.image_temp_u16.is_none() || self.image_temp_u16.as_ref().unwrap().len() != img_size {
            self.image_temp_u16 = Some(vec![0; img_size]);
        }

        let kernel_size_half = box_size / 2;
        let image_temp_u16 = self.image_temp_u16.as_mut().unwrap();
        let image2 = self.image2.as_mut().unwrap();

        // Pass 1: Horizontal Sum
        #[cfg(feature = "simd-image")]
        {
            #[cfg(all(target_arch = "x86_64", target_feature = "sse4.1"))]
            {
                if is_x86_feature_detected!("sse4.1") {
                    unsafe { box_filter_h_simd_x86(data, image_temp_u16, self.image_x, self.image_y, kernel_size_half); }
                } else {
                    box_filter_h_scalar(data, image_temp_u16, self.image_x, self.image_y, kernel_size_half);
                }
            }
            #[cfg(not(target_arch = "x86_64"))]
            box_filter_h_scalar(data, image_temp_u16, self.image_x, self.image_y, kernel_size_half);
        }
        #[cfg(not(feature = "simd-image"))]
        box_filter_h_scalar(data, image_temp_u16, self.image_x, self.image_y, kernel_size_half);

        // Pass 2: Vertical Sum and Normalize
        #[cfg(feature = "simd-image")]
        {
            #[cfg(all(target_arch = "x86_64", target_feature = "sse4.1"))]
            {
                if is_x86_feature_detected!("sse4.1") {
                    unsafe { box_filter_v_simd_x86(image_temp_u16, image2, self.image_x, self.image_y, kernel_size_half, bias); }
                } else {
                    box_filter_v_scalar(image_temp_u16, image2, self.image_x, self.image_y, kernel_size_half, bias);
                }
            }
            #[cfg(not(target_arch = "x86_64"))]
            box_filter_v_scalar(image_temp_u16, image2, self.image_x, self.image_y, kernel_size_half, bias);
        }
        #[cfg(not(feature = "simd-image"))]
        box_filter_v_scalar(image_temp_u16, image2, self.image_x, self.image_y, kernel_size_half, bias);

        Ok(())
    }
}

pub fn box_filter_h_scalar(data: &[u8], image_temp_u16: &mut [u16], width: i32, height: i32, half: i32) {
    for j in 0..height {
        let row_offset = (j * width) as usize;
        for i in 0..width {
            let mut val = 0u16;
            for kernel_i in -half..=half {
                let ii = i + kernel_i;
                if ii >= 0 && ii < width {
                    val += data[row_offset + ii as usize] as u16;
                }
            }
            image_temp_u16[row_offset + i as usize] = val;
        }
    }
}

pub fn box_filter_v_scalar(image_temp_u16: &[u16], image2: &mut [u8], width: i32, height: i32, half: i32, bias: i32) {
    for i in 0..width {
        for j in 0..height {
            let mut val = 0u32;
            
            let v_start = if j - half < 0 { 0 } else { j - half };
            let v_end = if j + half >= height { height - 1 } else { j + half };
            let h_start = if i - half < 0 { 0 } else { i - half };
            let h_end = if i + half >= width { width - 1 } else { i + half };
            
            let count = ((v_end - v_start + 1) * (h_end - h_start + 1)) as u32;

            for jj in v_start..=v_end {
                val += image_temp_u16[(jj * width + i) as usize] as u32;
            }

            let mut pixel = (val / count) as i32;
            if bias != 0 {
                pixel += bias;
            }
            
            image2[(j * width + i) as usize] = pixel.clamp(0, 255) as u8;
        }
    }
}

/// Convert an RGBA buffer to a grayscale (luma) buffer using BT.601 coefficients.
pub fn rgba_to_gray(rgba: &[u8]) -> Vec<u8> {
    #[cfg(feature = "simd-image")]
    {
        #[cfg(all(target_arch = "wasm32", target_feature = "simd128"))]
        {
            return unsafe { rgba_to_gray_simd_wasm(rgba) };
        }
        #[cfg(all(target_arch = "x86_64", target_feature = "sse4.1"))]
        {
            if is_x86_feature_detected!("sse4.1") {
                return unsafe { rgba_to_gray_simd_x86(rgba) };
            }
        }
    }
    
    rgba_to_gray_scalar(rgba)
}

pub fn rgba_to_gray_scalar(rgba: &[u8]) -> Vec<u8> {
    let mut gray = Vec::with_capacity(rgba.len() / 4);
    for chunk in rgba.chunks_exact(4) {
        let r = chunk[0] as f32;
        let g = chunk[1] as f32;
        let b = chunk[2] as f32;
        gray.push((0.2989 * r + 0.5870 * g + 0.1140 * b + 0.5) as u8);
    }
    gray
}

/// Convert an RGB buffer to a grayscale (luma) buffer using BT.601 coefficients.
pub fn rgb_to_gray(rgb: &[u8]) -> Vec<u8> {
    #[cfg(feature = "simd-image")]
    {
        // SIMD for RGB (3-byte) is more complex due to alignment.
        // For now, we only provide scalar or focus on RGBA which is common in WASM.
    }
    
    rgb_to_gray_scalar(rgb)
}

fn rgb_to_gray_scalar(rgb: &[u8]) -> Vec<u8> {
    let mut gray = Vec::with_capacity(rgb.len() / 3);
    for chunk in rgb.chunks_exact(3) {
        let r = chunk[0] as f32;
        let g = chunk[1] as f32;
        let b = chunk[2] as f32;
        gray.push((0.2989 * r + 0.5870 * g + 0.1140 * b + 0.5) as u8);
    }
    gray
}

#[cfg(all(feature = "simd-image", target_arch = "wasm32", target_feature = "simd128"))]
#[target_feature(enable = "simd128")]
unsafe fn rgba_to_gray_simd_wasm(rgba: &[u8]) -> Vec<u8> {
    use std::arch::wasm32::*;
    
    let mut gray = Vec::with_capacity(rgba.len() / 4);
    let chunks_len = rgba.len() / 16;
    
    // Fixed point coefficients for (77R + 151G + 28B) >> 8
    let coeffs = i16x8_make(77, 151, 28, 0, 77, 151, 28, 0);
    
    let mut rgba_ptr = rgba.as_ptr();
    
    for _ in 0..chunks_len {
        let v = v128_load(rgba_ptr as *const v128);
        
        let low = u16x8_extend_low_u8x16(v);
        let high = u16x8_extend_high_u8x16(v);
        
        let dot_low = i32x4_dot_i16x8(low, coeffs);
        let dot_high = i32x4_dot_i16x8(high, coeffs);
        
        // sum = [R0*77+G0*151, B0*28, R1*77+G1*151, B1*28]
        // We need to add (0,1) and (2,3)
        let sum_low = i32x4_add(dot_low, i32x4_shuffle::<1, 1, 3, 3>(dot_low, dot_low));
        let sum_high = i32x4_add(dot_high, i32x4_shuffle::<1, 1, 3, 3>(dot_high, dot_high));
        
        let res_low = u32x4_shr(sum_low, 8);
        let res_high = u32x4_shr(sum_high, 8);
        
        // Pack [L0, L1, L2, L3]
        let res = i32x4_shuffle::<0, 2, 4, 6>(res_low, res_high);
        
        let mut out = [0u32; 4];
        v128_store(out.as_mut_ptr() as *mut v128, res);
        gray.push(out[0] as u8);
        gray.push(out[1] as u8);
        gray.push(out[2] as u8);
        gray.push(out[3] as u8);

        rgba_ptr = rgba_ptr.add(16);
    }
    
    let rem_start = chunks_len * 4;
    for chunk in rgba.chunks_exact(4).skip(rem_start) {
        let r = chunk[0] as i32;
        let g = chunk[1] as i32;
        let b = chunk[2] as i32;
        gray.push(((r * 77 + g * 151 + b * 28 + 128) >> 8) as u8);
    }
    
    gray
}

#[cfg(all(feature = "simd-image", target_arch = "x86_64", target_feature = "sse4.1"))]
#[target_feature(enable = "sse4.1")]
pub unsafe fn rgba_to_gray_simd_x86(rgba: &[u8]) -> Vec<u8> {
    use std::arch::x86_64::*;
    
    let mut gray = Vec::with_capacity(rgba.len() / 4);
    let chunks_len = rgba.len() / 16;
    
    // Fixed point coefficients: R=77, G=151, B=28
    let coeffs = _mm_setr_epi16(77, 151, 28, 0, 77, 151, 28, 0);
    
    let mut rgba_ptr = rgba.as_ptr();
    
    for _ in 0..chunks_len {
        let v = _mm_loadu_si128(rgba_ptr as *const __m128i);
        
        // Expand u8 to i16
        let low = _mm_cvtepu8_epi16(v);
        let high = _mm_cvtepu8_epi16(_mm_srli_si128(v, 8));
        
        // madd: [R0*77 + G0*151, B0*28 + 0, R1*77 + G1*151, B1*28 + 0]
        let dot_low = _mm_madd_epi16(low, coeffs);
        let dot_high = _mm_madd_epi16(high, coeffs);
        
        // sum pairs: [sum0, sum1, sum2, sum3]
        // Actually, madd gives [p0, p1, p2, p3] where we want p0+p1 and p2+p3.
        // Shuffle [p1, p0, p3, p2]: _MM_SHUFFLE(2, 3, 0, 1) = 10 11 00 01 = 0xB1.
        let sum_low = _mm_add_epi32(dot_low, _mm_shuffle_epi32(dot_low, 0xB1));
        let sum_high = _mm_add_epi32(dot_high, _mm_shuffle_epi32(dot_high, 0xB1));
        
        // Values are at indices (0, 2) in sum_low and sum_high.
        // Shr 8 and pack.
        let luma_low = _mm_srli_epi32(sum_low, 8);
        let luma_high = _mm_srli_epi32(sum_high, 8);
        
        // Pack into u8.
        let res = _mm_shuffle_epi8(_mm_unpacklo_epi64(luma_low, luma_high), 
                                  _mm_setr_epi8(0, 4, 8, 12, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1));
        
        let mut out = [0u32; 4];
        _mm_storeu_si128(out.as_mut_ptr() as *mut __m128i, res);
        
        let bytes = out[0].to_le_bytes();
        gray.push(bytes[0]);
        gray.push(bytes[1]);
        gray.push(bytes[2]);
        gray.push(bytes[3]);

        rgba_ptr = rgba_ptr.add(16);
    }
    
    let rem_start = chunks_len * 4;
    for chunk in rgba.chunks_exact(4).skip(rem_start) {
        let r = chunk[0] as i32;
        let g = chunk[1] as i32;
        let b = chunk[2] as i32;
        gray.push(((r * 77 + g * 151 + b * 28 + 128) >> 8) as u8);
    }
    
    gray
}

#[cfg(all(feature = "simd-image", target_arch = "x86_64", target_feature = "sse4.1"))]
#[target_feature(enable = "sse4.1")]
pub unsafe fn box_filter_h_simd_x86(data: &[u8], image_temp_u16: &mut [u16], width: i32, height: i32, half: i32) {
    // Horizontal scalar for now (sliding window is faster but scalar is baseline)
    box_filter_h_scalar(data, image_temp_u16, width, height, half);
}

#[cfg(all(feature = "simd-image", target_arch = "x86_64", target_feature = "sse4.1"))]
#[target_feature(enable = "sse4.1")]
pub unsafe fn box_filter_v_simd_x86(image_temp_u16: &[u16], image2: &mut [u8], width: i32, height: i32, half: i32, bias: i32) {
    use std::arch::x86_64::*;
    
    for i_base in (0..width as usize).step_by(8) {
        let remaining = (width as usize).saturating_sub(i_base);
        if remaining < 8 { 
            for i in i_base..width as usize {
                for j in 0..height {
                    let mut val = 0u32;
                    let v_start = if j - half < 0 { 0 } else { j - half };
                    let v_end = if j + half >= height { height - 1 } else { j + half };
                    let h_start = if i as i32 - half < 0 { 0 } else { i as i32 - half };
                    let h_end = if i as i32 + half >= width { width - 1 } else { i as i32 + half };
                    let count = ((v_end - v_start + 1) * (h_end - h_start + 1)) as u32;

                    for jj in v_start..=v_end {
                        val += image_temp_u16[(jj * width + i as i32) as usize] as u32;
                    }
                    let mut pixel = (val / count) as i32;
                    if bias != 0 { pixel += bias; }
                    image2[(j * width + i as i32) as usize] = pixel.clamp(0, 255) as u8;
                }
            }
            continue;
        }

        for j in 0..height {
            let mut sum_low_v = _mm_setzero_si128();
            let mut sum_high_v = _mm_setzero_si128();
            
            let v_start = if j - half < 0 { 0 } else { j - half };
            let v_end = if j + half >= height { height - 1 } else { j + half };
            
            for jj in v_start..=v_end {
                let row_ptr = image_temp_u16.as_ptr().add((jj * width) as usize + i_base);
                let v = _mm_loadu_si128(row_ptr as *const __m128i); 
                
                sum_low_v = _mm_add_epi32(sum_low_v, _mm_cvtepu16_epi32(v));
                sum_high_v = _mm_add_epi32(sum_high_v, _mm_cvtepu16_epi32(_mm_srli_si128(v, 8)));
            }
            
            for i_off in 0..8 {
                let actual_i = i_base + i_off;
                let h_start = if actual_i as i32 - half < 0 { 0 } else { actual_i as i32 - half };
                let h_end = if actual_i as i32 + half >= width { width - 1 } else { actual_i as i32 + half };
                let count = ((v_end - v_start + 1) * (h_end - h_start + 1)) as u32;

                let mut res_arr = [0i32; 4];
                let val = if i_off < 4 {
                    _mm_storeu_si128(res_arr.as_mut_ptr() as *mut __m128i, sum_low_v);
                    res_arr[i_off] as u32
                } else {
                    _mm_storeu_si128(res_arr.as_mut_ptr() as *mut __m128i, sum_high_v);
                    res_arr[i_off - 4] as u32
                };

                let mut pixel = (val / count) as i32;
                if bias != 0 { pixel += bias; }
                image2[(j * width + actual_i as i32) as usize] = pixel.clamp(0, 255) as u8;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_image_proc_hist() {
        let mut ipi = ARImageProcInfo::new(4, 4);
        
        let mut gradient = vec![];
        for i in 0..16 {
            gradient.push((i * 10) as u8);
        }

        ipi.luma_hist(&gradient).unwrap();
        assert_eq!(ipi.hist_bins[0], 1);
        assert_eq!(ipi.hist_bins[10], 1);
        assert_eq!(ipi.hist_bins[150], 1);
        assert_eq!(ipi.hist_bins[160], 0);

        ipi.luma_hist_and_cdf(&gradient).unwrap();
        assert_eq!(ipi.cdf_bins[150], 16);
        assert_eq!(ipi.cdf_bins[0], 1);
    }

    #[test]
    fn test_otsu_thresholding() {
        let mut ipi = ARImageProcInfo::new(5, 5);
        let mut img = vec![200; 25]; // Peak at 200
        img[6] = 100; img[7] = 100; // Peak at 100
        img[11] = 100; img[12] = 100;
        
        let thresh = ipi.luma_hist_and_otsu(&img).unwrap();
        // Since peaks are 100 and 200, threshold should be around 100.
        assert!(thresh >= 100 && thresh < 200);
    }
}
