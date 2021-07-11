use super::{
    obb::*,
    fields::*
};
use bevy::prelude::*;

#[repr(C)]
struct SdfOpSpecificBlock {
    pub vec4s: [Vec4; 2],
    pub floats: [f32; 2]
}

#[repr(C)]
struct SdfChildIndicesBlock {
    pub left_index: u32,
    pub right_index: u32,
}

#[repr(C)]
struct SdfOperationBlock {
    pub op_code: u32,
    pub op_specific: SdfOpSpecificBlock,
    pub gen_transform: Mat4,
    pub child_indices: SdfChildIndicesBlock,
}