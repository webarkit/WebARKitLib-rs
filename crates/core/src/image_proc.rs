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

        let kernel_size_half = box_size / 2;
        let image2 = self.image2.as_mut().unwrap();

        for j in 0..self.image_y {
            for i in 0..self.image_x {
                let mut val = 0;
                let mut count = 0;

                for kernel_j in -kernel_size_half..=kernel_size_half {
                    let jj = j + kernel_j;
                    if jj < 0 || jj >= self.image_y { continue; }

                    let row_offset = (jj * self.image_x) as usize;
                    for kernel_i in -kernel_size_half..=kernel_size_half {
                        let ii = i + kernel_i;
                        if ii < 0 || ii >= self.image_x { continue; }
                        val += data[row_offset + ii as usize] as i32;
                        count += 1;
                    }
                }

                let mut pixel = val / count;
                if bias != 0 {
                    pixel += bias;
                }
                
                image2[(j * self.image_x + i) as usize] = pixel.clamp(0, 255) as u8;
            }
        }

        Ok(())
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
