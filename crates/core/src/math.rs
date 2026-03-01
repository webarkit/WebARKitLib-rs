//! Matrix and Vector Data Structures for WebARKitLib
//! Translated from ARToolKit C headers (matrix.h)

use crate::types::ARdouble;

/// Matrix structure
#[derive(Debug, Clone, PartialEq)]
#[repr(C)]
pub struct ARMat {
    pub m: *mut ARdouble,
    pub row: i32,
    pub clm: i32,
}

impl Default for ARMat {
    fn default() -> Self {
        Self {
            m: std::ptr::null_mut(),
            row: 0,
            clm: 0,
        }
    }
}

/// Float Matrix structure (Explicit f32 variant)
#[derive(Debug, Clone, PartialEq)]
#[repr(C)]
pub struct ARMatf {
    pub m: *mut f32,
    pub row: i32,
    pub clm: i32,
}

impl Default for ARMatf {
    fn default() -> Self {
        Self {
            m: std::ptr::null_mut(),
            row: 0,
            clm: 0,
        }
    }
}

/// Vector structure
#[derive(Debug, Clone, PartialEq)]
#[repr(C)]
pub struct ARVec {
    pub v: *mut ARdouble,
    pub clm: i32,
}

impl Default for ARVec {
    fn default() -> Self {
        Self {
            v: std::ptr::null_mut(),
            clm: 0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_armat_default_initialization() {
        let mat = ARMat::default();
        assert_eq!(mat.m, std::ptr::null_mut());
        assert_eq!(mat.row, 0);
        assert_eq!(mat.clm, 0);
    }

    #[test]
    fn test_armatf_default_initialization() {
        let matf = ARMatf::default();
        assert_eq!(matf.m, std::ptr::null_mut());
        assert_eq!(matf.row, 0);
        assert_eq!(matf.clm, 0);
    }

    #[test]
    fn test_arvec_default_initialization() {
        let vec = ARVec::default();
        assert_eq!(vec.v, std::ptr::null_mut());
        assert_eq!(vec.clm, 0);
    }
}
