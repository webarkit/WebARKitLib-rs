/*
 *  kpm_ffi.rs
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

//! Raw bindgen-generated FFI bindings for `kpm_c_api.h`.
//!
//! This module is only compiled when the **`ffi-backend`** feature is
//! enabled. It re-exports all symbols produced by `bindgen` in
//! `build.rs`, including the opaque [`KpmOpaqueHandle`] type and the
//! four `extern "C"` functions: [`kpm_create`], [`kpm_destroy`],
//! [`kpm_add_ref_image`], and [`kpm_query`].
//!
//! **Do not call these functions directly** — use
//! [`CppFreakMatcher`](crate::kpm::CppFreakMatcher) instead, which wraps
//! them with safe Rust types and null-pointer guards.

#[cfg(feature = "ffi-backend")]
mod bindings {
    #![allow(non_upper_case_globals, non_camel_case_types, non_snake_case)]
    include!(concat!(env!("OUT_DIR"), "/kpm_bindings.rs"));
}

#[cfg(feature = "ffi-backend")]
pub use bindings::*;
