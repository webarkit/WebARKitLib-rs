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

use super::Ar2Error;
use byteorder::{LittleEndian, ReadBytesExt, WriteBytesExt};
use std::io::{self, BufWriter, Cursor, Read, Write};
use std::path::Path;

/// Minimum image dimension used by KPM for feature detection.
///
/// Matches `KPM_MINIMUM_IMAGE_SIZE` in `markerCreator.cpp`.
const KPM_MINIMUM_IMAGE_SIZE: i32 = 28;

/// Default JPEG quality for `.iset` file encoding.
///
/// Matches `AR2_DEFAULT_JPEG_IMAGE_QUALITY` in the C source (`imageSet.c`).
/// Value 80 provides a good balance between file size (~300 KB for typical
/// markers) and visual fidelity.
const AR2_DEFAULT_JPEG_IMAGE_QUALITY: u8 = 80;

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
    /// Writes the **JPEG-compressed format** matching the C++
    /// `ar2WriteImageSet()`: only the base scale (highest DPI) is stored
    /// as an embedded JPEG; lower scales are regenerated at load time.
    /// This produces files ~30× smaller than the raw pixel format.
    ///
    /// The JPEG quality defaults to 80, matching `AR2_DEFAULT_JPEG_IMAGE_QUALITY`.
    ///
    /// ## Binary layout
    ///
    /// ```text
    /// [i32 LE]           num          — number of scales
    /// [bytes]            JPEG data    — grayscale JPEG of scale[0] with DPI in JFIF
    /// [f32 LE × (num-1)] DPI values   — for scales 1..num-1
    /// ```
    pub fn save<P: AsRef<Path>>(&self, path: P) -> io::Result<()> {
        let file = std::fs::File::create(path)?;
        let mut w = BufWriter::new(file);
        self.write_jpeg_to(&mut w, AR2_DEFAULT_JPEG_IMAGE_QUALITY)?;
        w.flush()
    }

    /// Save the image set using the **legacy raw pixel format**.
    ///
    /// This produces larger files but is lossless. Useful for debugging
    /// or when exact pixel values must be preserved.
    pub fn save_raw<P: AsRef<Path>>(&self, path: P) -> io::Result<()> {
        let file = std::fs::File::create(path)?;
        let mut w = BufWriter::new(file);
        self.write_raw_to(&mut w)?;
        w.flush()
    }

    /// Serialize the image set into an in-memory byte vector.
    ///
    /// Uses the JPEG-compressed format. See [`save()`](Self::save).
    pub fn to_bytes(&self) -> io::Result<Vec<u8>> {
        let mut buf = Vec::new();
        self.write_jpeg_to(&mut buf, AR2_DEFAULT_JPEG_IMAGE_QUALITY)?;
        Ok(buf)
    }

    /// Serialize the image set as raw pixels into an in-memory byte vector.
    ///
    /// Uses the legacy format. See [`save_raw()`](Self::save_raw).
    pub fn to_bytes_raw(&self) -> io::Result<Vec<u8>> {
        let mut buf = Vec::new();
        self.write_raw_to(&mut buf)?;
        Ok(buf)
    }

    /// Write the image set in JPEG-compressed format.
    ///
    /// Matches the C++ `ar2WriteImageSet()` format: `num(i32)` +
    /// JPEG blob (scale 0 with DPI embedded in JFIF) + trailing
    /// DPI values (`f32 × (num-1)`).
    fn write_jpeg_to<W: Write>(&self, writer: &mut W, quality: u8) -> io::Result<()> {
        if self.scale.is_empty() {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "image set has no scales to write",
            ));
        }

        let num = self.scale.len();
        let base = &self.scale[0];

        // 1. Write number of scales.
        writer.write_i32::<LittleEndian>(num as i32)?;

        // 2. Encode scale[0] as grayscale JPEG and embed DPI in JFIF header.
        let jpeg_data = encode_grayscale_jpeg(
            &base.img_bw,
            base.xsize as u32,
            base.ysize as u32,
            base.dpi,
            quality,
        )?;
        writer.write_all(&jpeg_data)?;

        // 3. Write DPI values for scales 1..num-1.
        for img in &self.scale[1..] {
            writer.write_f32::<LittleEndian>(img.dpi)?;
        }

        Ok(())
    }

    /// Write the image set in legacy raw pixel format.
    ///
    /// This is the shared serialization core for [`save_raw()`](Self::save_raw)
    /// and [`to_bytes_raw()`](Self::to_bytes_raw). The field ordering mirrors
    /// [`read_legacy_format()`] so that `write_raw_to` → `read_legacy_format`
    /// is a lossless roundtrip.
    fn write_raw_to<W: Write>(&self, writer: &mut W) -> io::Result<()> {
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

// ---------------------------------------------------------------------------
// Image set generation
// ---------------------------------------------------------------------------

/// Compute the list of DPI values for an AR2 image pyramid.
///
/// Ports `setDPI()` from `markerCreator.cpp`. Steps DPI levels from
/// `dpi_min` up to `dpi_max = dpi` using cube-root-of-2 increments
/// (`× 2^(1/3)` ≈ 1.2599), matching the C tool exactly so that the
/// generated `.iset` files are interoperable with C-generated markers.
///
/// # Returns
///
/// A `Vec<f32>` of DPI values ordered **highest to lowest** (index 0 =
/// source DPI, last index = minimum DPI). Always contains at least one entry.
pub(crate) fn compute_dpi_levels(dpi: f32, xsize: i32, ysize: i32) -> Vec<f32> {
    let min_dim = xsize.min(ysize) as f32;

    // Minimum DPI: truncated to 3 decimal places, matching C's
    // `truncf(...* 1000.0) / 1000.0`.
    let dpi_min = ((KPM_MINIMUM_IMAGE_SIZE as f32 / min_dim) * dpi * 1000.0).trunc() / 1000.0;
    let dpi_min = dpi_min.max(1.0); // safety floor
    let dpi_max = dpi;

    let cube_root_2 = 2.0f32.powf(1.0 / 3.0);

    // Count how many levels are needed (ports C's dpi_num loop).
    //
    // C code (simplified):
    //   for (i = 1;; i++) { dpiWork *= 2^(1/3); if (dpiWork >= dpiMax*0.95) break; }
    //   dpi_num = i + 1;
    let dpi_num = if (dpi_min - dpi_max).abs() < 1e-4 {
        1
    } else {
        let mut dpi_work = dpi_min;
        let mut i: usize = 1;
        loop {
            dpi_work *= cube_root_2;
            if dpi_work >= dpi_max * 0.95 {
                break;
            }
            i += 1;
        }
        i + 1
    };

    // Build the list from lowest DPI upward, then reverse so index 0 is
    // the highest DPI — matching C's `dpi_list[0]` = highest.
    let mut levels: Vec<f32> = Vec::with_capacity(dpi_num);
    let mut dpi_work = dpi_min;
    for _ in 0..dpi_num {
        levels.push(dpi_work);
        dpi_work *= cube_root_2;
        if dpi_work >= dpi_max * 0.95 {
            dpi_work = dpi_max;
        }
    }
    levels.reverse();
    levels
}

/// Generate an AR2 image pyramid from a raw pixel buffer.
///
/// Ports C's `ar2GenImageSet()` from `AR2/imageSet.c`. Builds a
/// multi-resolution pyramid by downscaling the source image to each DPI
/// level computed by [`compute_dpi_levels()`], using area-averaging.
///
/// # Arguments
///
/// * `image` — Raw pixel data. Length must be `xsize * ysize * nc`.
/// * `xsize`  — Image width in pixels.
/// * `ysize`  — Image height in pixels.
/// * `nc`     — Number of colour channels: `1` for grayscale, `3` for RGB.
/// * `dpi`    — Source image resolution in dots-per-inch.
///
/// # Returns
///
/// An [`AR2ImageSetT`] whose `scale` vector is ordered highest-to-lowest DPI,
/// with all layers in grayscale (1 byte per pixel). The returned set is ready
/// to be written with [`AR2ImageSetT::save()`] to produce a `.iset` file
/// compatible with both the Rust loader and C's `ar2LoadImageSet()`.
///
/// # Errors
///
/// Returns [`Ar2Error::InvalidInput`] if:
/// - `image` is empty or dimensions are non-positive,
/// - `image.len() != xsize * ysize * nc`,
/// - `nc` is not 1 or 3,
/// - `dpi` is not positive.
pub fn ar2_gen_image_set(
    image: &[u8],
    xsize: i32,
    ysize: i32,
    nc: i32,
    dpi: f32,
) -> Result<AR2ImageSetT, Ar2Error> {
    if image.is_empty() || xsize <= 0 || ysize <= 0 {
        return Err(Ar2Error::InvalidInput(
            "image is empty or has zero dimensions",
        ));
    }
    let expected = (xsize as usize) * (ysize as usize) * (nc as usize);
    if image.len() != expected {
        return Err(Ar2Error::InvalidInput(
            "image buffer length does not match xsize * ysize * nc",
        ));
    }
    if nc != 1 && nc != 3 {
        return Err(Ar2Error::InvalidInput(
            "nc must be 1 (grayscale) or 3 (RGB)",
        ));
    }
    if dpi <= 0.0 {
        return Err(Ar2Error::InvalidInput("dpi must be positive"));
    }

    let dpi_levels = compute_dpi_levels(dpi, xsize, ysize);

    // Build each level from the original source using gen_image_layer1,
    // which handles colour-to-grayscale conversion (nc parameter) and
    // area-averaging downscale — matching C's `ar2GenImageSet` loop.
    let mut scales = Vec::with_capacity(dpi_levels.len());
    for &target_dpi in &dpi_levels {
        let layer = gen_image_layer1(image, xsize, ysize, nc, dpi, target_dpi);
        scales.push(layer);
    }

    Ok(AR2ImageSetT { scale: scales })
}

/// Encode grayscale pixels as a JPEG with DPI embedded in the JFIF header.
///
/// This mirrors the C `jpgwrite()` function from `AR2/jpeg.c`, which:
/// 1. Encodes the grayscale image as a JPEG at the given quality level.
/// 2. Sets the JFIF APP0 density fields to the specified DPI so that
///    `ar2LoadImageSet()` (and our [`parse_jfif_dpi()`]) can recover
///    the resolution at load time.
///
/// # JFIF density patch
///
/// The JFIF APP0 header has this layout after `SOI (0xFFD8)`:
///
/// ```text
/// offset 2:  FF E0           — APP0 marker
/// offset 4:  <length:2>      — segment length
/// offset 6:  "JFIF\0"        — identifier (5 bytes)
/// offset 11: <version:2>     — e.g. 1.1
/// offset 13: <units:1>       — 0=no units, 1=DPI, 2=dots/cm
/// offset 14: <Xdensity:2>    — big-endian u16
/// offset 16: <Ydensity:2>    — big-endian u16
/// ```
///
/// We patch bytes 13–17 to set `units=1` and `X/Y density = dpi`.
fn encode_grayscale_jpeg(
    pixels: &[u8],
    width: u32,
    height: u32,
    dpi: f32,
    quality: u8,
) -> io::Result<Vec<u8>> {
    use image::codecs::jpeg::JpegEncoder;
    use image::ColorType;

    let mut jpeg_buf: Vec<u8> = Vec::new();
    {
        let mut encoder = JpegEncoder::new_with_quality(&mut jpeg_buf, quality);
        encoder
            .encode(pixels, width, height, ColorType::L8.into())
            .map_err(|e| io::Error::new(io::ErrorKind::Other, e))?;
    }

    // Patch the JFIF APP0 header to embed DPI.
    // Ensure we have a valid JFIF header to patch.
    if jpeg_buf.len() >= 18
        && jpeg_buf[0] == 0xFF
        && jpeg_buf[1] == 0xD8
        && jpeg_buf[2] == 0xFF
        && jpeg_buf[3] == 0xE0
        && &jpeg_buf[6..11] == b"JFIF\0"
    {
        let dpi_u16 = (dpi as u16).max(1);
        let dpi_bytes = dpi_u16.to_be_bytes();
        jpeg_buf[13] = 1; // units = DPI
        jpeg_buf[14] = dpi_bytes[0]; // X density high byte
        jpeg_buf[15] = dpi_bytes[1]; // X density low byte
        jpeg_buf[16] = dpi_bytes[0]; // Y density high byte
        jpeg_buf[17] = dpi_bytes[1]; // Y density low byte
    } else {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "JPEG encoder did not produce a valid JFIF header; cannot embed DPI",
        ));
    }

    Ok(jpeg_buf)
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

    /// Verify cube-root-of-2 stepping for a known input.
    ///
    /// 800×600 @ 72 DPI:
    ///   dpi_min = trunc((28/600)*72*1000)/1000 = trunc(3360)/1000 = 3.36
    ///   Levels step from 3.36 up to 72 by ×2^(1/3).
    #[test]
    fn test_compute_dpi_levels_standard() {
        let levels = compute_dpi_levels(72.0, 800, 600);
        // Must have at least 2 levels (dpi_max and dpi_min differ)
        assert!(
            levels.len() >= 2,
            "expected ≥ 2 levels, got {}",
            levels.len()
        );
        // Index 0 must be the highest DPI (= source DPI)
        assert!(
            (levels[0] - 72.0).abs() < 0.5,
            "highest level should be ~72 dpi, got {}",
            levels[0]
        );
        // Last entry must be the lowest DPI
        let last = *levels.last().unwrap();
        assert!(
            last < levels[0],
            "lowest level should be < highest, got {} vs {}",
            last,
            levels[0]
        );
        // Each level should be strictly decreasing
        for i in 1..levels.len() {
            assert!(
                levels[i] < levels[i - 1],
                "levels not monotonically decreasing at index {}: {} >= {}",
                i,
                levels[i],
                levels[i - 1]
            );
        }
    }

    /// Single-level case: when image is tiny, dpi_min ≈ dpi_max → 1 level.
    #[test]
    fn test_compute_dpi_levels_single() {
        // 28×28 @ 72 DPI: dpi_min = trunc((28/28)*72*1000)/1000 = 72.0 = dpi_max
        let levels = compute_dpi_levels(72.0, 28, 28);
        assert_eq!(levels.len(), 1);
        assert!((levels[0] - 72.0).abs() < 1e-3);
    }

    /// Verify ar2_gen_image_set produces the right number of scales and
    /// correct pixel dimensions at each level.
    #[test]
    fn test_ar2_gen_image_set_dimensions() {
        // 64×64 grayscale @ 72 DPI
        let src = vec![128u8; 64 * 64];
        let set = ar2_gen_image_set(&src, 64, 64, 1, 72.0).expect("gen failed");
        let levels = compute_dpi_levels(72.0, 64, 64);
        assert_eq!(set.num(), levels.len(), "scale count must match dpi_levels");
        // Each level must have positive dimensions
        for (i, img) in set.scale.iter().enumerate() {
            assert!(
                img.xsize > 0 && img.ysize > 0,
                "level {} has zero dimensions",
                i
            );
            assert_eq!(
                img.img_bw.len(),
                (img.xsize * img.ysize) as usize,
                "level {} pixel buffer size mismatch",
                i
            );
            assert!(
                (img.dpi - levels[i]).abs() < 1e-3,
                "level {} DPI mismatch: {} vs {}",
                i,
                img.dpi,
                levels[i]
            );
        }
    }

    /// ar2_gen_image_set with RGB input (nc=3) must produce grayscale output.
    #[test]
    fn test_ar2_gen_image_set_rgb_to_grayscale() {
        // 8×8 RGB: each pixel R=60, G=120, B=180 → avg = 120
        let mut src = Vec::new();
        for _ in 0..(8 * 8) {
            src.extend_from_slice(&[60u8, 120, 180]);
        }
        let set = ar2_gen_image_set(&src, 8, 8, 3, 72.0).expect("gen failed");
        // Level 0 (full resolution) — all pixels should be 120
        assert!(
            set.scale[0].img_bw.iter().all(|&v| v == 120),
            "grayscale conversion incorrect"
        );
    }

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

    /// Roundtrip test: build → save_raw() → load() → compare.
    ///
    /// Verifies that the save/load cycle is lossless for a 2-level image
    /// pyramid written in legacy raw format.
    #[test]
    fn test_image_set_save_raw_load_roundtrip() {
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
        original.save_raw(tmp.path()).unwrap();
        let loaded = AR2ImageSetT::load(tmp.path()).unwrap();

        assert_eq!(loaded.num(), original.num());
        for (orig, load) in original.scale.iter().zip(loaded.scale.iter()) {
            assert_eq!(load.xsize, orig.xsize);
            assert_eq!(load.ysize, orig.ysize);
            assert_eq!(load.dpi, orig.dpi);
            assert_eq!(load.img_bw, orig.img_bw);
        }
    }

    /// JPEG roundtrip test: build → save() → load() → compare dimensions/DPI.
    ///
    /// JPEG is lossy so pixel values won't match exactly, but dimensions,
    /// DPI, and scale count must be preserved. Pixel values are checked
    /// within a tolerance of ±5.
    #[test]
    fn test_image_set_save_jpeg_load_roundtrip() {
        // 16×16 gradient at 150 DPI — single scale for simplicity.
        let mut pixels = vec![0u8; 256];
        for i in 0..256 {
            pixels[i] = i as u8;
        }

        let original = AR2ImageSetT {
            scale: vec![AR2ImageT {
                img_bw: pixels.clone(),
                xsize: 16,
                ysize: 16,
                dpi: 150.0,
            }],
        };

        let tmp = tempfile::NamedTempFile::new().unwrap();
        original.save(tmp.path()).unwrap();
        let loaded = AR2ImageSetT::load(tmp.path()).unwrap();

        assert_eq!(loaded.num(), 1);
        assert_eq!(loaded.scale[0].xsize, 16);
        assert_eq!(loaded.scale[0].ysize, 16);
        assert!((loaded.scale[0].dpi - 150.0).abs() < 0.01);

        // Check pixels within tolerance (JPEG is lossy).
        for (orig, loaded) in pixels.iter().zip(loaded.scale[0].img_bw.iter()) {
            let diff = (*orig as i16 - *loaded as i16).unsigned_abs();
            assert!(
                diff <= 5,
                "pixel difference too large: orig={}, loaded={}, diff={}",
                orig,
                loaded,
                diff
            );
        }
    }

    /// Verify encode_grayscale_jpeg embeds DPI correctly in JFIF header.
    #[test]
    fn test_encode_grayscale_jpeg_dpi() {
        let pixels = vec![128u8; 8 * 8];
        let jpeg = encode_grayscale_jpeg(&pixels, 8, 8, 220.0, 80).unwrap();

        // Verify JPEG SOI marker.
        assert_eq!(jpeg[0], 0xFF);
        assert_eq!(jpeg[1], 0xD8);

        // Verify DPI was embedded.
        let dpi = parse_jfif_dpi(&jpeg);
        assert_eq!(dpi, Some(220.0));
    }

    /// Multi-scale JPEG roundtrip: verifies trailing DPI values are preserved.
    #[test]
    fn test_image_set_jpeg_multiscale_roundtrip() {
        let original = AR2ImageSetT {
            scale: vec![
                AR2ImageT {
                    img_bw: vec![100u8; 16 * 16],
                    xsize: 16,
                    ysize: 16,
                    dpi: 200.0,
                },
                AR2ImageT {
                    img_bw: vec![100u8; 8 * 8],
                    xsize: 8,
                    ysize: 8,
                    dpi: 100.0,
                },
            ],
        };

        let bytes = original.to_bytes().unwrap();
        let loaded = AR2ImageSetT::from_bytes(&bytes).unwrap();

        assert_eq!(loaded.num(), 2);
        // Scale 0: from JPEG decode.
        assert_eq!(loaded.scale[0].xsize, 16);
        assert_eq!(loaded.scale[0].ysize, 16);
        assert!((loaded.scale[0].dpi - 200.0).abs() < 0.01);
        // Scale 1: regenerated from scale 0 at trailing DPI.
        assert!((loaded.scale[1].dpi - 100.0).abs() < 0.01);
    }
}
