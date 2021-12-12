use super::{
    obb::*,
    fields::*,
    faux_shader::*,
};
use bevy::{
    prelude::*,
    reflect::TypeUuid,
    core::Byteable,
    render::renderer::RenderResources,
};

#[derive(Debug, Clone, Copy)]
#[repr(C)]
pub struct SdfOpSpecificBlock {
    pub vec4s: [Vec4; 2],
    pub floats: [f32; 2],
}

impl SdfOpSpecificBlock {
    pub const ZERO: SdfOpSpecificBlock = SdfOpSpecificBlock {
        vec4s: [Vec4::ZERO; 2],
        floats: [0.0; 2],
    };
}

#[derive(Debug, Clone, Copy)]
#[repr(C)]
pub struct SdfOperationBlock {
    pub op_code: u32,
    pub is_primitive: bool,
    pub level: u32,
    pub left_child_len: u32,
    pub right_child_len: u32,
    pub union_block: SdfOpSpecificBlock,
    pub op_specific: SdfOpSpecificBlock,
    pub bounding_box: SdfBoundingBoxBlock,
}

impl SdfOperationBlock {
    pub const ZERO: SdfOperationBlock = SdfOperationBlock {
        op_code: 0,
        is_primitive: false,
        level: 0,
        left_child_len: 0,
        right_child_len: 0,
        union_block: SdfOpSpecificBlock::ZERO,
        op_specific: SdfOpSpecificBlock::ZERO,
        bounding_box: SdfBoundingBoxBlock::ZERO,
    };
}

unsafe impl Byteable for SdfOperationBlock {}

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

#[derive(RenderResources, TypeUuid)]
#[uuid = "b2ad9d5c-eb4e-517b-98d7-1162e78ddadb"]
pub struct SdfTreeBuffer {
    #[render_resources(buffer)]
    pub op_buffer: Vec<SdfOperationBlock>,
}