//! Parameter loading and manipulation utilities
//! Translated from ARToolKit C headers (param.h)

use byteorder::{BigEndian, ReadBytesExt};
use std::io::{self, Read};
use crate::types::ARParam;

impl ARParam {
    /// Load ARParam from a byte stream (Endian-safe cross-platform BigEndian deserialization)
    pub fn load<R: Read>(mut reader: R) -> io::Result<Self> {
        let mut param = ARParam::default();

        // ARToolKit 5 parameter files are encoded in BigEndian format.
        param.xsize = reader.read_i32::<BigEndian>()?;
        param.ysize = reader.read_i32::<BigEndian>()?;

        // Load 3x4 projection matrix
        for row in 0..3 {
            for col in 0..4 {
                param.mat[row][col] = reader.read_f64::<BigEndian>()?;
            }
        }

        // Load distortion factors. AR_DIST_FACTOR_NUM_MAX is 9.
        for i in 0..crate::types::AR_DIST_FACTOR_NUM_MAX {
            if let Ok(val) = reader.read_f64::<BigEndian>() {
                param.dist_factor[i] = val;
            } else {
                break; // End of file or buffer handled gracefully for older parameter files.
            }
        }

        Ok(param)
    }

    /// Legacy ARToolKit v1 specific finalizer (swaps dist_factor 2 and 3)
    pub fn finalize_version_1(&mut self) {
        self.dist_factor.swap(2, 3);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Cursor;

    #[test]
    fn test_arparam_load_big_endian() {
        // Construct a dummy byte array representing BigEndian encoded param data
        // xsize (4), ysize(4), mat(3*4 * 8), dist_factor(9 * 8)
        
        let mut buffer = Vec::new();
        // xsize = 640
        buffer.extend_from_slice(&640i32.to_be_bytes());
        // ysize = 480
        buffer.extend_from_slice(&480i32.to_be_bytes());

        // Fill mat with 1.0
        for _ in 0..12 {
            buffer.extend_from_slice(&1.0f64.to_be_bytes());
        }

        // Fill dist_factor with 2.0
        for _ in 0..9 {
            buffer.extend_from_slice(&2.0f64.to_be_bytes());
        }

        let cursor = Cursor::new(buffer);
        let param = ARParam::load(cursor).expect("Failed to load params");

        assert_eq!(param.xsize, 640);
        assert_eq!(param.ysize, 480);
        assert_eq!(param.mat[0][0], 1.0);
        assert_eq!(param.dist_factor[0], 2.0);
    }

    #[test]
    fn test_arparam_finalize_version_1() {
        let mut param = ARParam::default();
        param.dist_factor[2] = 10.0;
        param.dist_factor[3] = 20.0;

        param.finalize_version_1();

        assert_eq!(param.dist_factor[2], 20.0);
        assert_eq!(param.dist_factor[3], 10.0);
    }
}
