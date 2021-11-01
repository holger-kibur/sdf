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
    pub bounding_box: SdfBoundingBoxBlock,
}

#[repr(C)]
struct SdfBoundingBoxBlock {
    pub matrix: Mat4,
    pub scale: Vec3,
    pub full_inverse: Mat4,
    pub trans_inverse: Mat4,
}