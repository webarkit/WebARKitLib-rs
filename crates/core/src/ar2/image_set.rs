/*
 *  image_set.rs
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

//! AR2 Image Set (.iset) file I/O.
//!
//! Ported from `AR2/imageSet.c`.
//!
//! The `.iset` file contains an image pyramid — a base image at the highest
//! DPI, plus DPI values for each additional scale. The base image is stored
//! as an embedded JPEG; downscaled layers are generated at load time via
//! area-averaging.
//!
//! ## File format (new / JPEG-based)
//!
//! ```text
//! [i32 LE]   num          — number of scales
//! [bytes]    JPEG data    — embedded grayscale JPEG (scale 0)
//! [f32 LE × (num-1)]      — DPI values for scales 1..num-1 (at EOF)
//! ```
//!
//! The base scale's DPI is read from the JPEG's JFIF density header.
//!
//! ## Legacy format (ARToolKit v4.x)
//!
//! ```text
//! For each scale:
//!   [i32 LE]  xsize
//!   [i32 LE]  ysize
//!   [f32 LE]  dpi
//!   [u8 × xsize*ysize]  raw grayscale pixels
//! ```

use byteorder::{LittleEndian, ReadBytesExt};
use std::io::{self, Cursor, Read};
use std::path::Path;

/// A single image scale in the pyramid.
#[derive(Debug, Clone)]
pub struct AR2ImageT {
    /// Grayscale pixel data (row-major, 1 byte per pixel).
    pub img_bw: Vec<u8>,
    /// Width in pixels.
    pub xsize: i32,
    /// Height in pixels.
    pub ysize: i32,
    /// Resolution in dots-per-inch.
    pub dpi: f32,
}

/// An image pyramid loaded from an `.iset` file.
#[derive(Debug, Clone)]
pub struct AR2ImageSetT {
    /// Image scales, ordered from highest DPI (index 0) to lowest.
    pub scale: Vec<AR2ImageT>,
}

impl AR2ImageSetT {
    /// Number of scales in the pyramid.
    pub fn num(&self) -> usize {
        self.scale.len()
    }

    /// Load an `.iset` file from disk.
    ///
    /// Tries the new JPEG-based format first; falls back to legacy raw pixel
    /// format if the JPEG decode fails.
    pub fn load<P: AsRef<Path>>(path: P) -> io::Result<Self> {
        let data = std::fs::read(path)?;
        Self::from_bytes(&data)
    }

    /// Parse an `.iset` from an in-memory byte slice.
    pub fn from_bytes(data: &[u8]) -> io::Result<Self> {
        let mut cursor = Cursor::new(data);

        // Read number of scales.
        let num = cursor.read_i32::<LittleEndian>()?;
        if num <= 0 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("invalid scale count: {}", num),
            ));
        }
        let num = num as usize;

        // Try JPEG format first: bytes after the i32 should start with JPEG SOI.
        let jpeg_offset = cursor.position() as usize;
        if data.len() > jpeg_offset + 2
            && data[jpeg_offset] == 0xFF
            && data[jpeg_offset + 1] == 0xD8
        {
            match Self::read_jpeg_format(data, jpeg_offset, num) {
                Ok(set) => return Ok(set),
                Err(_) => {
                    // Fall through to legacy.
                    log::debug!("JPEG decode failed, trying legacy format");
                }
            }
        }

        // Legacy format: rewind and read raw pixel data.
        cursor.set_position(0);
        Self::read_legacy_format(&mut cursor)
    }

    /// Parse the new JPEG-based .iset format.
    fn read_jpeg_format(data: &[u8], jpeg_offset: usize, num: usize) -> io::Result<AR2ImageSetT> {
        // The JPEG blob runs from jpeg_offset to (end - 4*(num-1)).
        let trailing_dpi_bytes = if num > 1 { 4 * (num - 1) } else { 0 };
        if data.len() < jpeg_offset + trailing_dpi_bytes + 2 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "file too short for JPEG + DPI data",
            ));
        }
        let jpeg_end = data.len() - trailing_dpi_bytes;
        let jpeg_data = &data[jpeg_offset..jpeg_end];

        // Decode JPEG.
        let mut decoder = jpeg_decoder::Decoder::new(Cursor::new(jpeg_data));
        let pixels = decoder
            .decode()
            .map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e))?;
        let info = decoder
            .info()
            .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "no JPEG info"))?;

        // Must be grayscale (1 component).
        if info.pixel_format != jpeg_decoder::PixelFormat::L8 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "JPEG is not grayscale",
            ));
        }

        let base_xsize = info.width as i32;
        let base_ysize = info.height as i32;

        // DPI from JFIF density — the jpeg-decoder crate doesn't expose JFIF
        // density directly, so we parse it manually from the raw JPEG data.
        let base_dpi = parse_jfif_dpi(jpeg_data).unwrap_or(72.0);

        // Read trailing DPI values for scales 1..num-1.
        let mut dpi_values = Vec::with_capacity(num);
        dpi_values.push(base_dpi);

        if num > 1 {
            let mut dpi_cursor = Cursor::new(&data[jpeg_end..]);
            for _ in 1..num {
                let dpi = dpi_cursor.read_f32::<LittleEndian>()?;
                dpi_values.push(dpi);
            }
        }

        // Build the image pyramid.
        let mut scales = Vec::with_capacity(num);

        // Scale 0 = the JPEG-decoded image.
        scales.push(AR2ImageT {
            img_bw: pixels,
            xsize: base_xsize,
            ysize: base_ysize,
            dpi: base_dpi,
        });

        // Generate downscaled layers via area-averaging.
        for &target_dpi in &dpi_values[1..num] {
            let layer = gen_image_layer(
                &scales[0].img_bw,
                base_xsize,
                base_ysize,
                base_dpi,
                target_dpi,
            );
            scales.push(layer);
        }

        Ok(AR2ImageSetT { scale: scales })
    }

    /// Parse legacy (ARToolKit v4.x) raw pixel format.
    fn read_legacy_format<R: Read>(reader: &mut R) -> io::Result<AR2ImageSetT> {
        let num = reader.read_i32::<LittleEndian>()?;
        if num <= 0 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("invalid scale count: {}", num),
            ));
        }
        let num = num as usize;

        let mut scales = Vec::with_capacity(num);
        for _ in 0..num {
            let xsize = reader.read_i32::<LittleEndian>()?;
            let ysize = reader.read_i32::<LittleEndian>()?;
            let dpi = reader.read_f32::<LittleEndian>()?;

            let pixel_count = (xsize as usize) * (ysize as usize);
            let mut img_bw = vec![0u8; pixel_count];
            reader.read_exact(&mut img_bw)?;

            scales.push(AR2ImageT {
                img_bw,
                xsize,
                ysize,
                dpi,
            });
        }

        Ok(AR2ImageSetT { scale: scales })
    }
}

/// Parse JFIF APP0 marker to extract DPI.
///
/// JFIF APP0 structure (after SOI 0xFFD8):
///   FF E0 <length:2> "JFIF\0" <version:2> <units:1> <Xdensity:2> <Ydensity:2>
/// If units == 1, density is in DPI.
fn parse_jfif_dpi(jpeg_data: &[u8]) -> Option<f32> {
    // Minimum: SOI(2) + APP0 marker(2) + length(2) + "JFIF\0"(5) + version(2) + units(1) + Xd(2) + Yd(2) = 18
    if jpeg_data.len() < 18 {
        return None;
    }
    // Check SOI.
    if jpeg_data[0] != 0xFF || jpeg_data[1] != 0xD8 {
        return None;
    }
    // Check APP0.
    if jpeg_data[2] != 0xFF || jpeg_data[3] != 0xE0 {
        return None;
    }
    // Check "JFIF\0" identifier at offset 4 (after 2-byte length).
    if &jpeg_data[4 + 2..4 + 2 + 5] != b"JFIF\0" {
        return None;
    }
    let units = jpeg_data[4 + 2 + 5 + 2]; // offset 13
    let x_density = u16::from_be_bytes([jpeg_data[14], jpeg_data[15]]);

    if units == 1 && x_density > 0 {
        Some(x_density as f32)
    } else {
        None
    }
}

/// Generate a downscaled image layer via area-averaging.
///
/// Ported from `ar2GenImageLayer2` in the C source.
pub(crate) fn gen_image_layer(
    src: &[u8],
    src_xsize: i32,
    src_ysize: i32,
    src_dpi: f32,
    target_dpi: f32,
) -> AR2ImageT {
    let scale = target_dpi / src_dpi;
    let dst_xsize = ((src_xsize as f32) * scale) as i32;
    let dst_ysize = ((src_ysize as f32) * scale) as i32;

    if dst_xsize <= 0 || dst_ysize <= 0 {
        return AR2ImageT {
            img_bw: Vec::new(),
            xsize: 0,
            ysize: 0,
            dpi: target_dpi,
        };
    }

    let mut dst = vec![0u8; (dst_xsize * dst_ysize) as usize];

    // Area-average downscale.
    let inv_scale = src_dpi / target_dpi;
    for dy in 0..dst_ysize {
        for dx in 0..dst_xsize {
            let sx_start = (dx as f32 * inv_scale) as i32;
            let sy_start = (dy as f32 * inv_scale) as i32;
            let sx_end = (((dx + 1) as f32) * inv_scale).ceil() as i32;
            let sy_end = (((dy + 1) as f32) * inv_scale).ceil() as i32;

            let sx_end = sx_end.min(src_xsize);
            let sy_end = sy_end.min(src_ysize);

            let mut sum = 0u32;
            let mut count = 0u32;
            for sy in sy_start..sy_end {
                for sx in sx_start..sx_end {
                    sum += src[(sy * src_xsize + sx) as usize] as u32;
                    count += 1;
                }
            }

            dst[(dy * dst_xsize + dx) as usize] = if count > 0 { (sum / count) as u8 } else { 0 };
        }
    }

    AR2ImageT {
        img_bw: dst,
        xsize: dst_xsize,
        ysize: dst_ysize,
        dpi: target_dpi,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_jfif_dpi_120() {
        // Minimal JFIF APP0 header: SOI + APP0 marker + length + "JFIF\0" + version + units=1 + Xd=120 + Yd=120
        let mut header = vec![
            0xFF, 0xD8, // SOI
            0xFF, 0xE0, // APP0
            0x00, 0x10, // length = 16
            b'J', b'F', b'I', b'F', 0x00, // identifier
            0x01, 0x01, // version 1.1
            0x01, // units = DPI
            0x00, 0x78, // X density = 120
            0x00, 0x78, // Y density = 120
        ];
        // Pad to minimum size.
        header.extend_from_slice(&[0u8; 16]);

        assert_eq!(parse_jfif_dpi(&header), Some(120.0));
    }

    #[test]
    fn test_gen_image_layer_halves() {
        // 4x4 white image at 200 DPI → 2x2 at 100 DPI.
        let src = vec![255u8; 16];
        let layer = gen_image_layer(&src, 4, 4, 200.0, 100.0);
        assert_eq!(layer.xsize, 2);
        assert_eq!(layer.ysize, 2);
        assert_eq!(layer.dpi, 100.0);
        assert_eq!(layer.img_bw.len(), 4);
        assert!(layer.img_bw.iter().all(|&v| v == 255));
    }

    #[test]
    fn test_legacy_format_roundtrip() {
        // Build a minimal legacy .iset in memory.
        use byteorder::WriteBytesExt;
        let mut buf = Vec::new();
        buf.write_i32::<LittleEndian>(1).unwrap(); // num = 1
        buf.write_i32::<LittleEndian>(2).unwrap(); // xsize
        buf.write_i32::<LittleEndian>(2).unwrap(); // ysize
        buf.write_f32::<LittleEndian>(100.0).unwrap(); // dpi
        buf.extend_from_slice(&[10, 20, 30, 40]); // pixels

        let set = AR2ImageSetT::from_bytes(&buf).unwrap();
        assert_eq!(set.num(), 1);
        assert_eq!(set.scale[0].xsize, 2);
        assert_eq!(set.scale[0].ysize, 2);
        assert_eq!(set.scale[0].dpi, 100.0);
        assert_eq!(set.scale[0].img_bw, vec![10, 20, 30, 40]);
    }
}
