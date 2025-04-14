use param::ARparam;

#[link(name = "bindings")]
mod param;

// src/main.rs
extern "C" {
    //fn arParamrChangeSizeWrapper(a: i32, b: i32) -> bool;
    fn arParamDispWrapper(param: ARparam) -> i32;
}