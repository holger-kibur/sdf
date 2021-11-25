use super::{
    component::*,
    obb::CmpFloat,
};
use bevy::prelude::*;

#[derive(Copy, Clone)]
struct NnStackFrame {
    op: SdfOperationBlock,
    upstack: bool,
    left: bool,
    parent: usize,
    point: Vec4,
    index: usize,
    left_ret: f32,
    right_ret: f32,
}

impl NnStackFrame {
    const ZERO: NnStackFrame = NnStackFrame {
        op: SdfOperationBlock::ZERO,
        upstack: false,
        left: false,
        parent: 0,
        point: Vec4::ZERO,
        index: 0,
        left_ret: 0.0,
        right_ret: 0.0,
    };
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

fn downtree_dispatch(code: u32, op_specific: SdfOpSpecificBlock, point: Vec4) -> Vec4 {

}

pub fn binary_nearest_neighbor(sdf_tree: &SdfTreeBuffer, point: Vec4) -> (usize, f32) {
    let stack = vec![NnStackFrame::ZERO; 128];
    let stack_pointer = 0_usize;
    stack[stack_pointer] = NnStackFrame {
        op: sdf_tree.op_buffer[0],
        upstack: true,
        left: false,
        parent: 0,
        point,
        index: 0,
        left_ret: 0.0,
        right_ret: 0.0,
    };
    let current_nearest = f32::INFINITY;
    let current_nearest_index = 0;
    while stack_pointer >= 0 {
        let stack_frame = stack[stack_pointer];
        let op = stack_frame.op;
        if stack_frame.upstack {
            if op.is_primitive {
                let dist = prim_dispatch(op.op_code, op.op_specific, stack_frame.point);
                if stack_frame.left {
                    stack[stack_frame.parent].left_ret = dist;
                } else {
                    stack[stack_frame.parent].right_ret = dist;
                }
                stack_pointer -= 1;
            } else if op.is_union {
                let left_child = sdf_tree.op_buffer[op.left_child as usize];
                let left_mindist = mindist(left_child.bounding_box, stack_frame.point);
                let left_maxdist = maxdist(left_child.bounding_box, stack_frame.point);
                let right_child = sdf_tree.op_buffer[op.right_child as usize];
                let right_mindist = mindist(right_child.bounding_box, stack_frame.point);
                let right_maxdist = maxdist(right_child.bounding_box, stack_frame.point);
                stack[stack_pointer].upstack = false;
                if left_maxdist < right_mindist {
                    stack[stack_pointer + 1] = NnStackFrame {
                        op: left_child,
                        upstack: true,
                        left: true,
                        parent: stack_pointer,
                        point: downtree_dispatch(op.op_code, op.op_specific, stack_frame.point),
                        index: op.left_child as usize,
                        left_ret: f32::INFINITY,
                        right_ret: f32::INFINITY,
                    };
                    stack_pointer += 1;
                } else if right_maxdist < left_mindist {
                    stack[stack_pointer + 1] = NnStackFrame {
                        op: right_child,
                        upstack: true,
                        left: false,
                        parent: stack_pointer,
                        point: downtree_dispatch(op.op_code, op.op_specific, stack_frame.point),
                        index: op.right_child as usize,
                        left_ret: f32::INFINITY,
                        right_ret: f32::INFINITY,
                    };
                    stack_pointer += 1;
                } else {
                    stack[stack_pointer + 1] = NnStackFrame {
                        op: right_child,
                        upstack: true,
                        left: false,
                        parent: stack_pointer,
                        point: downtree_dispatch(op.op_code, op.op_specific, stack_frame.point),
                        index: op.right_child as usize,
                        left_ret: f32::INFINITY,
                        right_ret: f32::INFINITY,
                    };
                    stack[stack_pointer + 2] = NnStackFrame {
                        op: left_child,
                        upstack: true,
                        left: true,
                        parent: stack_pointer,
                        point: downtree_dispatch(op.op_code, op.op_specific, stack_frame.point),
                        index: op.left_child as usize,
                        left_ret: f32::INFINITY,
                        right_ret: f32::INFINITY,
                    };
                    stack_pointer += 2;
                }
            } else {
                
            }
        } else {
            
        }
    }
    (current_nearest_index, current_nearest)
}