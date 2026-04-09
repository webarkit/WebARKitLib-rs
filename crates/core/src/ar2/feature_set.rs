/*
 *  feature_set.rs
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

//! AR2 Feature Set (.fset) file I/O.
//!
//! Ported from `AR2/featureSet.c`.
//!
//! The `.fset` file stores AR2 feature points extracted at multiple DPI
//! scales. Each scale contains a list of feature coordinates with their
//! marker-space positions and a similarity score.
//!
//! ## Binary format (little-endian)
//!
//! ```text
//! [i32]  num_scales
//! For each scale:
//!   [i32]  scale       — scale index
//!   [f32]  maxdpi      — maximum DPI for this scale
//!   [f32]  mindpi      — minimum DPI for this scale
//!   [i32]  num_coords  — number of feature coordinates
//!   For each coord:
//!     [i32]  x         — pixel x position
//!     [i32]  y         — pixel y position
//!     [f32]  mx        — marker x coordinate
//!     [f32]  my        — marker y coordinate
//!     [f32]  maxSim    — maximum similarity score
//! ```

use byteorder::{LittleEndian, ReadBytesExt, WriteBytesExt};
use std::io::{self, BufWriter, Cursor, Read, Write};
use std::path::Path;

/// A single feature coordinate.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AR2FeatureCoordT {
    /// Pixel x position in the image.
    pub x: i32,
    /// Pixel y position in the image.
    pub y: i32,
    /// Marker-space x coordinate.
    pub mx: f32,
    /// Marker-space y coordinate.
    pub my: f32,
    /// Maximum similarity score.
    pub max_sim: f32,
}

/// Feature points at a single DPI scale.
#[derive(Debug, Clone)]
pub struct AR2FeaturePointsT {
    /// Feature coordinates at this scale.
    pub coord: Vec<AR2FeatureCoordT>,
    /// Scale index.
    pub scale: i32,
    /// Maximum DPI for this scale.
    pub maxdpi: f32,
    /// Minimum DPI for this scale.
    pub mindpi: f32,
}

impl AR2FeaturePointsT {
    /// Number of feature coordinates at this scale.
    pub fn num(&self) -> usize {
        self.coord.len()
    }
}

/// A feature set loaded from an `.fset` file.
#[derive(Debug, Clone)]
pub struct AR2FeatureSetT {
    /// Feature points grouped by scale.
    pub list: Vec<AR2FeaturePointsT>,
}

impl AR2FeatureSetT {
    /// Number of scales in the feature set.
    pub fn num(&self) -> usize {
        self.list.len()
    }

    /// Load an `.fset` file from disk.
    pub fn load<P: AsRef<Path>>(path: P) -> io::Result<Self> {
        let data = std::fs::read(path)?;
        Self::from_bytes(&data)
    }

    /// Parse an `.fset` from an in-memory byte slice.
    pub fn from_bytes(data: &[u8]) -> io::Result<Self> {
        let mut cursor = Cursor::new(data);
        Self::read_from(&mut cursor)
    }

    /// Save the feature set to an `.fset` file on disk.
    ///
    /// Writes the same little-endian binary layout that [`load()`](Self::load)
    /// and the C function `ar2LoadFeatureSet()` expect, so the resulting file
    /// is compatible with both the Rust and C/C++ readers.
    ///
    /// ## Binary layout (little-endian)
    ///
    /// ```text
    /// [i32]  num_scales
    /// For each scale:
    ///   [i32]  scale       — scale index
    ///   [f32]  maxdpi      — maximum DPI for this scale
    ///   [f32]  mindpi      — minimum DPI for this scale
    ///   [i32]  num_coords  — number of feature coordinates
    ///   For each coord:
    ///     [i32]  x         — pixel x position
    ///     [i32]  y         — pixel y position
    ///     [f32]  mx        — marker x coordinate (millimetres)
    ///     [f32]  my        — marker y coordinate (millimetres)
    ///     [f32]  maxSim    — maximum similarity score
    /// ```
    pub fn save<P: AsRef<Path>>(&self, path: P) -> io::Result<()> {
        let file = std::fs::File::create(path)?;
        let mut w = BufWriter::new(file);
        self.write_to(&mut w)?;
        w.flush()
    }

    /// Serialize the feature set into an in-memory byte vector.
    ///
    /// Uses the same binary format as [`save()`](Self::save).
    pub fn to_bytes(&self) -> io::Result<Vec<u8>> {
        let mut buf = Vec::new();
        self.write_to(&mut buf)?;
        Ok(buf)
    }

    /// Write the feature set to any [`Write`] implementor.
    ///
    /// This is the shared serialization core used by both [`save()`](Self::save)
    /// and [`to_bytes()`](Self::to_bytes). The field ordering mirrors
    /// [`read_from()`] exactly so that `write_to` → `read_from` is a
    /// lossless roundtrip.
    fn write_to<W: Write>(&self, writer: &mut W) -> io::Result<()> {
        if self.list.is_empty() {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "feature set has no scales to write",
            ));
        }

        // Number of scales.
        writer.write_i32::<LittleEndian>(self.list.len() as i32)?;

        for pts in &self.list {
            // Scale header.
            writer.write_i32::<LittleEndian>(pts.scale)?;
            writer.write_f32::<LittleEndian>(pts.maxdpi)?;
            writer.write_f32::<LittleEndian>(pts.mindpi)?;
            writer.write_i32::<LittleEndian>(pts.coord.len() as i32)?;

            // Feature coordinates.
            for c in &pts.coord {
                writer.write_i32::<LittleEndian>(c.x)?;
                writer.write_i32::<LittleEndian>(c.y)?;
                writer.write_f32::<LittleEndian>(c.mx)?;
                writer.write_f32::<LittleEndian>(c.my)?;
                writer.write_f32::<LittleEndian>(c.max_sim)?;
            }
        }

        Ok(())
    }

    fn read_from<R: Read>(reader: &mut R) -> io::Result<Self> {
        let num = reader.read_i32::<LittleEndian>()?;
        if num <= 0 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("invalid feature set scale count: {}", num),
            ));
        }
        let num = num as usize;

        let mut list = Vec::with_capacity(num);
        for _ in 0..num {
            let scale = reader.read_i32::<LittleEndian>()?;
            let maxdpi = reader.read_f32::<LittleEndian>()?;
            let mindpi = reader.read_f32::<LittleEndian>()?;
            let coord_count = reader.read_i32::<LittleEndian>()?;

            if coord_count < 0 {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidData,
                    format!("invalid coord count: {}", coord_count),
                ));
            }
            let coord_count = coord_count as usize;

            let mut coord = Vec::with_capacity(coord_count);
            for _ in 0..coord_count {
                let x = reader.read_i32::<LittleEndian>()?;
                let y = reader.read_i32::<LittleEndian>()?;
                let mx = reader.read_f32::<LittleEndian>()?;
                let my = reader.read_f32::<LittleEndian>()?;
                let max_sim = reader.read_f32::<LittleEndian>()?;

                coord.push(AR2FeatureCoordT {
                    x,
                    y,
                    mx,
                    my,
                    max_sim,
                });
            }

            list.push(AR2FeaturePointsT {
                coord,
                scale,
                maxdpi,
                mindpi,
            });
        }

        Ok(AR2FeatureSetT { list })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_feature_set_roundtrip() {
        use byteorder::WriteBytesExt;
        let mut buf = Vec::new();

        // 1 scale with 2 features.
        buf.write_i32::<LittleEndian>(1).unwrap(); // num scales
        buf.write_i32::<LittleEndian>(0).unwrap(); // scale
        buf.write_f32::<LittleEndian>(200.0).unwrap(); // maxdpi
        buf.write_f32::<LittleEndian>(50.0).unwrap(); // mindpi
        buf.write_i32::<LittleEndian>(2).unwrap(); // num coords

        // Coord 0
        buf.write_i32::<LittleEndian>(10).unwrap(); // x
        buf.write_i32::<LittleEndian>(20).unwrap(); // y
        buf.write_f32::<LittleEndian>(1.5).unwrap(); // mx
        buf.write_f32::<LittleEndian>(2.5).unwrap(); // my
        buf.write_f32::<LittleEndian>(0.95).unwrap(); // maxSim

        // Coord 1
        buf.write_i32::<LittleEndian>(30).unwrap();
        buf.write_i32::<LittleEndian>(40).unwrap();
        buf.write_f32::<LittleEndian>(3.5).unwrap();
        buf.write_f32::<LittleEndian>(4.5).unwrap();
        buf.write_f32::<LittleEndian>(0.80).unwrap();

        let fset = AR2FeatureSetT::from_bytes(&buf).unwrap();
        assert_eq!(fset.num(), 1);
        assert_eq!(fset.list[0].scale, 0);
        assert_eq!(fset.list[0].maxdpi, 200.0);
        assert_eq!(fset.list[0].mindpi, 50.0);
        assert_eq!(fset.list[0].num(), 2);
        assert_eq!(fset.list[0].coord[0].x, 10);
        assert_eq!(fset.list[0].coord[0].y, 20);
        assert_eq!(fset.list[0].coord[1].max_sim, 0.80);
    }

    /// Roundtrip test: build → save() → load() → compare.
    ///
    /// Verifies that the save/load cycle is lossless for a feature set
    /// with 2 scales and 3 feature coordinates each.
    #[test]
    fn test_feature_set_save_load_roundtrip() {
        let original = AR2FeatureSetT {
            list: vec![
                AR2FeaturePointsT {
                    coord: vec![
                        AR2FeatureCoordT { x: 10, y: 20, mx: 3.528, my: 16.933, max_sim: 0.42 },
                        AR2FeatureCoordT { x: 55, y: 80, mx: 19.403, my: 10.795, max_sim: 0.37 },
                        AR2FeatureCoordT { x: 100, y: 5, mx: 35.278, my: 19.261, max_sim: 0.51 },
                    ],
                    scale: 0,
                    maxdpi: 200.0,
                    mindpi: 100.0,
                },
                AR2FeaturePointsT {
                    coord: vec![
                        AR2FeatureCoordT { x: 5, y: 10, mx: 1.764, my: 8.467, max_sim: 0.55 },
                        AR2FeatureCoordT { x: 25, y: 40, mx: 8.819, my: 5.397, max_sim: 0.48 },
                        AR2FeatureCoordT { x: 50, y: 2, mx: 17.639, my: 9.631, max_sim: 0.60 },
                    ],
                    scale: 1,
                    maxdpi: 100.0,
                    mindpi: 50.0,
                },
            ],
        };

        let tmp = tempfile::NamedTempFile::new().unwrap();
        original.save(tmp.path()).unwrap();
        let loaded = AR2FeatureSetT::load(tmp.path()).unwrap();

        assert_eq!(loaded.num(), original.num());
        for (orig_pts, load_pts) in original.list.iter().zip(loaded.list.iter()) {
            assert_eq!(load_pts.scale, orig_pts.scale);
            assert_eq!(load_pts.maxdpi, orig_pts.maxdpi);
            assert_eq!(load_pts.mindpi, orig_pts.mindpi);
            assert_eq!(load_pts.num(), orig_pts.num());
            for (oc, lc) in orig_pts.coord.iter().zip(load_pts.coord.iter()) {
                assert_eq!(lc.x, oc.x);
                assert_eq!(lc.y, oc.y);
                assert!((lc.mx - oc.mx).abs() < 1e-4, "mx mismatch");
                assert!((lc.my - oc.my).abs() < 1e-4, "my mismatch");
                assert!((lc.max_sim - oc.max_sim).abs() < 1e-4, "max_sim mismatch");
            }
        }
    }

    /// Verify that to_bytes() output is loadable by from_bytes().
    #[test]
    fn test_feature_set_to_bytes_from_bytes_roundtrip() {
        let original = AR2FeatureSetT {
            list: vec![AR2FeaturePointsT {
                coord: vec![AR2FeatureCoordT {
                    x: 42, y: 99, mx: 7.5, my: 12.3, max_sim: 0.65,
                }],
                scale: 0,
                maxdpi: 150.0,
                mindpi: 75.0,
            }],
        };

        let bytes = original.to_bytes().unwrap();
        let loaded = AR2FeatureSetT::from_bytes(&bytes).unwrap();

        assert_eq!(loaded.num(), 1);
        assert_eq!(loaded.list[0].coord[0].x, 42);
        assert_eq!(loaded.list[0].coord[0].y, 99);
    }
}
