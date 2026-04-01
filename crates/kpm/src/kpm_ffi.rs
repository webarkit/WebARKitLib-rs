#[cfg(feature = "ffi-backend")]
mod bindings {
    #![allow(non_upper_case_globals, non_camel_case_types, non_snake_case)]
    include!(concat!(env!("OUT_DIR"), "/kpm_bindings.rs"));
}

#[cfg(feature = "ffi-backend")]
pub use bindings::*;
