/*
 *  ref_data_set.rs
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

//! Reference-image collection for the KPM database.
//!
//! [`RefDataSet`] is a simple growable collection of [`RefImage`]
//! entries. It is used to stage reference images before they are
//! submitted to the backend for feature extraction.

use crate::types::RefImage;

/// A collection of reference images to be registered in the KPM database.
///
/// Images are added one at a time via [`add`](RefDataSet::add) and can
/// be iterated over with [`iter`](RefDataSet::iter).
pub struct RefDataSet {
    images: Vec<RefImage>,
}

impl RefDataSet {
    /// Creates an empty reference data set.
    pub fn new() -> Self {
        Self { images: Vec::new() }
    }

    /// Appends a reference image to the set.
    pub fn add(&mut self, image: RefImage) {
        self.images.push(image);
    }

    /// Returns the number of images in the set.
    pub fn len(&self) -> usize {
        self.images.len()
    }

    /// Returns `true` if the set contains no images.
    pub fn is_empty(&self) -> bool {
        self.images.is_empty()
    }

    /// Returns an iterator over the reference images.
    pub fn iter(&self) -> impl Iterator<Item = &RefImage> {
        self.images.iter()
    }
}

impl Default for RefDataSet {
    fn default() -> Self {
        Self::new()
    }
}
