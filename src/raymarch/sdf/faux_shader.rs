use super::{
    component::*,
    obb::CmpFloat,
};
use bevy::prelude::*;

#[derive(Clone, Copy)]
struct DowntreeResult {
    point: Vec4,
    ints: [i32; 2],
    f32s: [f32; 2],
}

fn maxdist(bbox: SdfBoundingBoxBlock, point: Vec4) -> f32 {
    [
        Vec4::new(1.0, 1.0, 1.0, 1.0),
        Vec4::new(-1.0, 1.0, 1.0, 1.0),
        Vec4::new(1.0, -1.0, 1.0, 1.0),
        Vec4::new(-1.0, -1.0, 1.0, 1.0),
        Vec4::new(1.0, 1.0, -1.0, 1.0),
        Vec4::new(-1.0, 1.0, -1.0, 1.0),
        Vec4::new(1.0, -1.0, -1.0, 1.0),
        Vec4::new(-1.0, -1.0, -1.0, 1.0),
    ].iter()
        .map(|x| CmpFloat(((bbox.matrix * *x) - point).length()))
        .max().unwrap().0
}

fn mindist(bbox: SdfBoundingBoxBlock, point: Vec4) -> f32 {
    let trans = bbox.full_inverse * point;
    let q_local = (trans.abs() - Vec4::splat(1.0)) * bbox.scale;
    q_local.max(Vec4::ZERO).length() + q_local.x.max(q_local.y.max(q_local.z)).min(0.0)
}

fn prim_dispatch(code: u32, op_specific: SdfOpSpecificBlock, point: Vec4) -> f32 {

}

fn downtree_dispatch(code: u32, op_specific: SdfOpSpecificBlock, point: Vec4) -> DowntreeResult {

}

fn inv_downtree_dispatch(code: u32, op_specific: SdfOpSpecificBlock, result: DowntreeResult) -> Vec4 {

}

fn uptree_dispatch(code: u32, op_specific: SdfOpSpecificBlock, left_dist: f32, right_dist: f32) -> f32 {

}

pub fn build_blend_tree(sdf_tree: &SdfTreeBuffer, point: Vec4) -> (usize, f32) {
    let buf = sdf_tree.op_buffer;
    let index = 0_usize;
    while index < buf.len() {
        
    }
}