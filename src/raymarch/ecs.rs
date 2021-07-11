use bevy::{
    prelude::*,
    core::Byteable,
    render::{
        renderer::RenderResources,
        pipeline::PipelineDescriptor,
    }
};
use super::dist_field::*;

#[repr(C)]
struct SdfInfo {
    pub sdf_type: u32,
    pub sdf_index: u32,
    pub position: Vec3,
    pub color: Vec4
}

unsafe impl Byteable for SdfInfo {}

#[derive(RenderResources)]
struct SdfInfoBuffer {
    pub buffer_length: u32,
    #[render_resources(buffer)]
    pub info_buffer: Vec<SdfInfo>,
}

struct SdfBuffer {
    objects: Vec
}

struct SdfRenderContext {
    pub pipeline: Handle<PipelineDescriptor>,
    pub object_buffer: Handle<>
}

