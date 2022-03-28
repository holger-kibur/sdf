use super::{
    component::*,
    obb::CmpFloat,
};
use bevy::prelude::*;

#[derive(Clone, Copy)]
struct DowntreeResult {
    point: Vec4,
}

#[derive(Clone)]
struct LevelStackEntry {
    branch_points: [Vec4; 2],
    branch_dists: [f32; 2],
    fill_idx: u32,
}

impl LevelStackEntry {
    pub const ZERO: Self = LevelStackEntry {
        branch_points: [Vec4::ZERO; 2],
        branch_dists: [0_f32; 2],
        fill_idx: 0,
    };
}

fn min(a: f32, b: f32) -> f32 {
    if a < b { a } else { b }
}

fn max(a: f32, b: f32) -> f32 {
    if a > b { a } else { b }
}

fn clamp(a: f32, lmin: f32, lmax: f32) -> f32 {
    max(lmin, min(a, lmax))
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
    match code {
        // Sphere
        0 => point.truncate().length() - op_specific.floats[0],

        other => panic!("Unsupported primitive op code: {}", other),
    }
}

fn downtree_dispatch(code: u32, op_specific: SdfOpSpecificBlock, point: Vec4) -> [Vec4; 2] {
    match code {
        // Union
        0 => [point, point],

        // CAA Clone
        1 => [
            point - op_specific.vec4s[0] * (point / op_specific.vec4s[0]).round().clamp(op_specific.vec4s[1], op_specific.vec4s[2]),
            Vec4::ZERO,
        ],

        other => panic!("Unsupported downtree op code: {}", other),
    }
}

fn uptree_dispatch(code: u32, op_specific: SdfOpSpecificBlock, left_dist: f32, right_dist: f32) -> f32 {
    match code {
        // Union
        0 => min(left_dist, right_dist),

        // CAA Clone
        1 => right_dist,
        
        other => panic!("Unsupported downtree op code: {}", other),
    }
}

pub fn nearest_neighbor(sdf_tree: &SdfTreeBuffer, point: Vec4) -> f32 {
    let mut dt_index = 0;
    let mut ut_index = 0;
    let mut last_dt_level = 0;
    let mut point_stack: Vec<LevelStackEntry> = vec![LevelStackEntry::ZERO; 256];

    point_stack[1] = LevelStackEntry {
        branch_points: [point, Vec4::ZERO],
        branch_dists: [f32::INFINITY, f32::INFINITY],
        fill_idx: 0,
    };

    while dt_index < sdf_tree.buffer_len as usize {
        let dt_block = &sdf_tree.downtree_buffer[dt_index];
        let (dt_point, dt_ut_prune_cmp) = {
            let this_frame = &point_stack[dt_block.level as usize];
            (this_frame.branch_points[this_frame.fill_idx as usize],
                this_frame.branch_dists[0])
        };

        // Apply union pruning
        if dt_block.parent_is_union {
            let this_mindist = mindist(dt_block.bounding_box, dt_point);
            if this_mindist > 0_f32
                && (this_mindist > dt_ut_prune_cmp
                    || this_mindist > maxdist(dt_block.other_box, dt_point)) {
                dt_index += 1 + dt_block.len as usize;
                ut_index += 1 + dt_block.len as usize;
                continue;
            }
        }   

        // Apply uptree algorithm
        if dt_block.level < last_dt_level {
            let mut last_ut_level = u32::MAX;
            loop {
                // Read current block
                let ut_block = &sdf_tree.uptree_buffer[ut_index];

                // Evaluate break condition
                if ut_block.level >= last_ut_level {
                    break;
                }

                // Perform uptree operation
                let (lbranch_dist, rbranch_dist) = {
                    let child_frame = &point_stack[ut_block.level as usize + 1];
                    (child_frame.branch_dists[0], child_frame.branch_dists[1])
                };
                let ut_frame = &mut point_stack[ut_block.level as usize];
                ut_frame.branch_dists[ut_frame.fill_idx as usize] = uptree_dispatch(
                    ut_block.op_code,
                    ut_block.op_specific,
                    lbranch_dist,
                    rbranch_dist);
                ut_frame.fill_idx += 1;

                // Increment
                last_ut_level = ut_block.level;
                ut_index += 1;
            }
        }

        // Primitive case
        if dt_block.is_primitive {
            let this_frame = &mut point_stack[dt_block.level as usize];
            this_frame.branch_dists[this_frame.fill_idx as usize] = prim_dispatch(
                dt_block.op_code,
                dt_block.op_specific,
                dt_block.bounding_box.trans_inverse * dt_point);
            this_frame.fill_idx += 1;
            ut_index += 1;
        } 
        // Non-primitive case
        else {
            let child_frame = &mut point_stack[dt_block.level as usize + 1];
            child_frame.branch_points = downtree_dispatch(
                dt_block.op_code,
                dt_block.op_specific,
                dt_block.bounding_box.trans_inverse * dt_point);
            child_frame.fill_idx = 0;
            child_frame.branch_dists = [f32::INFINITY; 2];
        }

        // Increment
        last_dt_level = dt_block.level;
        dt_index += 1;
    }

    // Finish propagating distance values to root
    while ut_index < sdf_tree.buffer_len as usize {
        // Read current block
        let ut_block = &sdf_tree.uptree_buffer[ut_index];

        // Perform uptree operation
        let (lbranch_dist, rbranch_dist) = {
            let child_frame = &point_stack[ut_block.level as usize + 1];
            (child_frame.branch_dists[0], child_frame.branch_dists[1])
        };
        let ut_frame = &mut point_stack[ut_block.level as usize];
        ut_frame.branch_dists[ut_frame.fill_idx as usize] = uptree_dispatch(
            ut_block.op_code,
            ut_block.op_specific,
            lbranch_dist,
            rbranch_dist);
        ut_frame.fill_idx += 1;

        // Increment
        ut_index += 1;
    }

    point_stack[1].branch_dists[0]
}