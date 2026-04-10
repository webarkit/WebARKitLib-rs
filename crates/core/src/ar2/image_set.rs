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

use byteorder::{LittleEndian, ReadBytesExt, WriteBytesExt};
use std::io::{self, BufWriter, Cursor, Read, Write};
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
            let layer = gen_image_layer2(
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

    /// Save the image set to an `.iset` file on disk.
    ///
    /// Writes the **legacy (ARToolKit v4.x) raw pixel format** so that the
    /// resulting file is readable by both the Rust [`load()`](Self::load)
    /// and the C/C++ `ar2LoadImageSet()` functions.
    ///
    /// ## Binary layout (little-endian)
    ///
    /// ```text
    /// [i32 LE]  num_scales          — number of pyramid levels
    /// For each scale:
    ///   [i32 LE]  xsize             — width in pixels
    ///   [i32 LE]  ysize             — height in pixels
    ///   [f32 LE]  dpi               — resolution in dots-per-inch
    ///   [u8 × xsize*ysize]          — raw grayscale pixels (row-major)
    /// ```
    pub fn save<P: AsRef<Path>>(&self, path: P) -> io::Result<()> {
        let file = std::fs::File::create(path)?;
        let mut w = BufWriter::new(file);
        self.write_to(&mut w)?;
        w.flush()
    }

    /// Serialize the image set into an in-memory byte vector.
    ///
    /// Uses the same legacy binary format as [`save()`](Self::save).
    pub fn to_bytes(&self) -> io::Result<Vec<u8>> {
        let mut buf = Vec::new();
        self.write_to(&mut buf)?;
        Ok(buf)
    }

    /// Write the image set to any [`Write`] implementor.
    ///
    /// This is the shared serialization core used by both [`save()`](Self::save)
    /// and [`to_bytes()`](Self::to_bytes). The field ordering mirrors
    /// [`read_legacy_format()`] so that `write_to` → `read_legacy_format` is a
    /// lossless roundtrip.
    fn write_to<W: Write>(&self, writer: &mut W) -> io::Result<()> {
        if self.scale.is_empty() {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "image set has no scales to write",
            ));
        }

        // Number of scales.
        writer.write_i32::<LittleEndian>(self.scale.len() as i32)?;

        for img in &self.scale {
            // Per-scale header.
            writer.write_i32::<LittleEndian>(img.xsize)?;
            writer.write_i32::<LittleEndian>(img.ysize)?;
            writer.write_f32::<LittleEndian>(img.dpi)?;

            // Raw grayscale pixel data.
            writer.write_all(&img.img_bw)?;
        }

        Ok(())
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

/// Generate a downscaled image layer from a colour or grayscale source.
///
/// Ported from `ar2GenImageLayer1` in the C source. This is the entry-point
/// variant that handles colour-to-grayscale conversion during downscale.
///
/// # Arguments
///
/// * `src` — Raw pixel data. Length must be `src_xsize * src_ysize * nc`.
/// * `src_xsize` — Source image width in pixels.
/// * `src_ysize` — Source image height in pixels.
/// * `nc` — Number of colour channels: `1` for grayscale, `3` for RGB.
/// * `src_dpi` — Source resolution in dots-per-inch.
/// * `target_dpi` — Desired output resolution in dots-per-inch.
///
/// For `nc == 3` the RGB channels are averaged to produce a single grayscale
/// value per pixel. For `nc == 1` the behaviour is identical to
/// [`gen_image_layer2()`].
pub(crate) fn gen_image_layer1(
    src: &[u8],
    src_xsize: i32,
    src_ysize: i32,
    nc: i32,
    src_dpi: f32,
    target_dpi: f32,
) -> AR2ImageT {
    let scale = target_dpi / src_dpi;
    let dst_xsize = ((src_xsize as f32) * scale + 0.5) as i32;
    let dst_ysize = ((src_ysize as f32) * scale + 0.5) as i32;

    if dst_xsize <= 0 || dst_ysize <= 0 {
        return AR2ImageT {
            img_bw: Vec::new(),
            xsize: 0,
            ysize: 0,
            dpi: target_dpi,
        };
    }

    let mut dst = vec![0u8; (dst_xsize * dst_ysize) as usize];
    let inv_scale = src_dpi / target_dpi;
    let nc_u = nc as usize;

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
                    let base = ((sy * src_xsize + sx) as usize) * nc_u;
                    // Sum all channels (for nc=1 this is just the grayscale
                    // value; for nc=3 we sum R+G+B).
                    for ch in 0..nc_u {
                        sum += src[base + ch] as u32;
                    }
                    count += 1;
                }
            }

            // Divide by count * nc to average across both pixels and channels,
            // producing a single grayscale value.
            let divisor = count * (nc as u32);
            dst[(dy * dst_xsize + dx) as usize] = if divisor > 0 {
                (sum / divisor) as u8
            } else {
                0
            };
        }
    }

    AR2ImageT {
        img_bw: dst,
        xsize: dst_xsize,
        ysize: dst_ysize,
        dpi: target_dpi,
    }
}

/// Generate a downscaled image layer from an already-grayscale source via
/// area-averaging.
///
/// Ported from `ar2GenImageLayer2` in the C source. Unlike
/// [`gen_image_layer1()`] this function assumes the input is already
/// single-channel grayscale and does not perform colour conversion.
///
/// # Arguments
///
/// * `src` — Grayscale pixel data. Length must be `src_xsize * src_ysize`.
/// * `src_xsize` — Source image width in pixels.
/// * `src_ysize` — Source image height in pixels.
/// * `src_dpi` — Source resolution in dots-per-inch.
/// * `target_dpi` — Desired output resolution in dots-per-inch.
pub(crate) fn gen_image_layer2(
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
    fn test_gen_image_layer2_halves() {
        // 4x4 white image at 200 DPI → 2x2 at 100 DPI.
        let src = vec![255u8; 16];
        let layer = gen_image_layer2(&src, 4, 4, 200.0, 100.0);
        assert_eq!(layer.xsize, 2);
        assert_eq!(layer.ysize, 2);
        assert_eq!(layer.dpi, 100.0);
        assert_eq!(layer.img_bw.len(), 4);
        assert!(layer.img_bw.iter().all(|&v| v == 255));
    }

    /// Test gen_image_layer1 with RGB input: average R+G+B per pixel.
    #[test]
    fn test_gen_image_layer1_rgb() {
        // 4×4 RGB image (12 bytes per row, 48 bytes total).
        // Each pixel: R=60, G=120, B=180 → grayscale avg = (60+120+180)/3 = 120.
        let mut src = Vec::new();
        for _ in 0..(4 * 4) {
            src.extend_from_slice(&[60u8, 120, 180]);
        }
        let layer = gen_image_layer1(&src, 4, 4, 3, 200.0, 100.0);
        assert_eq!(layer.xsize, 2);
        assert_eq!(layer.ysize, 2);
        assert_eq!(layer.dpi, 100.0);
        assert_eq!(layer.img_bw.len(), 4);
        // Each output pixel averages 4 source pixels that are each 120 gray.
        assert!(layer.img_bw.iter().all(|&v| v == 120));
    }

    /// Test gen_image_layer1 with grayscale (nc=1) behaves like layer2.
    #[test]
    fn test_gen_image_layer1_grayscale() {
        let src = vec![200u8; 16];
        let layer = gen_image_layer1(&src, 4, 4, 1, 200.0, 100.0);
        assert_eq!(layer.xsize, 2);
        assert_eq!(layer.ysize, 2);
        assert!(layer.img_bw.iter().all(|&v| v == 200));
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

    /// Roundtrip test: build → save() → load() → compare.
    ///
    /// Verifies that the save/load cycle is lossless for a 2-level image
    /// pyramid written in legacy raw format.
    #[test]
    fn test_image_set_save_load_roundtrip() {
        // Level 0: 8×8 at 200 DPI — checkerboard pattern.
        let mut pixels_0 = vec![0u8; 64];
        for y in 0..8i32 {
            for x in 0..8i32 {
                pixels_0[(y * 8 + x) as usize] = if (x + y) % 2 == 0 { 200 } else { 50 };
            }
        }

        // Level 1: 4×4 at 100 DPI — gradient.
        let mut pixels_1 = vec![0u8; 16];
        for i in 0..16 {
            pixels_1[i] = (i as u8) * 16;
        }

        let original = AR2ImageSetT {
            scale: vec![
                AR2ImageT {
                    img_bw: pixels_0,
                    xsize: 8,
                    ysize: 8,
                    dpi: 200.0,
                },
                AR2ImageT {
                    img_bw: pixels_1,
                    xsize: 4,
                    ysize: 4,
                    dpi: 100.0,
                },
            ],
        };

        let tmp = tempfile::NamedTempFile::new().unwrap();
        original.save(tmp.path()).unwrap();
        let loaded = AR2ImageSetT::load(tmp.path()).unwrap();

        assert_eq!(loaded.num(), original.num());
        for (orig, load) in original.scale.iter().zip(loaded.scale.iter()) {
            assert_eq!(load.xsize, orig.xsize);
            assert_eq!(load.ysize, orig.ysize);
            assert_eq!(load.dpi, orig.dpi);
            assert_eq!(load.img_bw, orig.img_bw);
        }
    }
}
