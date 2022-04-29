use super::{
    obb::*,
    faux_shader::*,
};
use bevy::{
    prelude::*,
    reflect::TypeUuid,
};

#[derive(Debug, Clone, Copy)]
#[repr(C)]
pub struct SdfOpSpecificBlock {
    pub mat4s: [Mat4; 2],
    pub vec4s: [Vec4; 3],
    pub floats: [f32; 2],
}

impl SdfOpSpecificBlock {
    pub const ZERO: SdfOpSpecificBlock = SdfOpSpecificBlock {
        mat4s: [Mat4::ZERO; 2],
        vec4s: [Vec4::ZERO; 3],
        floats: [0.0; 2],
    };
}

#[derive(Debug, Clone, Copy)]
#[repr(C)]
pub struct SdfOperationBlock {
    pub op_code: u32,
    pub is_primitive: bool,
    pub parent_is_union: bool,
    pub len: u32,
    pub level: u32,
    pub op_specific: SdfOpSpecificBlock,
    pub bounding_box: SdfBoundingBoxBlock,
    pub other_box: SdfBoundingBoxBlock,
}

impl SdfOperationBlock {
    pub const ZERO: SdfOperationBlock = SdfOperationBlock {
        op_code: 0,
        is_primitive: false,
        parent_is_union: false,
        len: 0,
        level: 0,
        op_specific: SdfOpSpecificBlock::ZERO,
        bounding_box: SdfBoundingBoxBlock::ZERO,
        other_box: SdfBoundingBoxBlock::ZERO,
    };
}

#[derive(Debug, Clone, Copy)]
#[repr(C)]
pub struct SdfOperationUptreeBlock {
    pub op_code: u32,
    pub parent_is_union: bool,
    pub op_specific: SdfOpSpecificBlock,
    pub level: u32,
}

#[derive(Debug, Clone, Copy)]
#[repr(C)]
pub struct SdfBoundingBoxBlock {
    pub matrix: Mat4,
    pub scale: Vec4,
    pub full_inverse: Mat4,
    pub trans_inverse: Mat4,
}

impl SdfBoundingBoxBlock {
    pub const ZERO: SdfBoundingBoxBlock = SdfBoundingBoxBlock {
        matrix: Mat4::ZERO,
        scale: Vec4::ZERO,
        full_inverse: Mat4::ZERO,
        trans_inverse: Mat4::ZERO,
    };
}

#[derive(TypeUuid)]
#[uuid = "b2ad9d5c-eb4e-517b-98d7-1162e78ddadb"]
pub struct SdfTreeBuffer {
    pub downtree_buffer: Vec<SdfOperationBlock>,
    pub uptree_buffer: Vec<SdfOperationUptreeBlock>,
    pub buffer_len: u32,
}

impl SdfTreeBuffer {
    pub fn make_empty() -> SdfTreeBuffer {
        SdfTreeBuffer {
            downtree_buffer: Vec::new(),
            uptree_buffer: Vec::new(),
            buffer_len: 0,
        }
    }
}