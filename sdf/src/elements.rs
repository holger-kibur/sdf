use std::fmt;
use bevy::prelude::*;
use super::{
    node::*, 
    obb::*,
    component::*,
};

pub struct SdfElementInfo {
    pub num_acc_slots: usize,
    pub num_drawn_slots: usize,
    pub is_primitive: bool,
    pub op_id: u32,
    pub is_union: bool,
}

impl SdfElementInfo {
    pub fn primitive_info(op_id: u32) -> Self {
        SdfElementInfo {
            num_acc_slots: 0,
            num_drawn_slots: 0,
            is_primitive: true,
            op_id,
            is_union: false,
        }
    }

    pub fn strict_info(op_id: u32, num_acc_slots: usize, num_drawn_slots: usize) -> Self {
        SdfElementInfo {
            num_acc_slots,
            num_drawn_slots,
            is_primitive: false,
            op_id,
            is_union: false,
        }
    }

    pub fn union_info(op_id: u32) -> Self {
        SdfElementInfo {
            num_acc_slots: 0,
            num_drawn_slots: usize::MAX,
            is_primitive: false,
            op_id,
            is_union: true,
        }
    }

    pub fn num_slots(&self) -> usize {
        self.num_acc_slots + self.num_drawn_slots
    }
}

impl PartialEq for SdfElementInfo {
    fn eq(&self, other: &Self) -> bool {
        self.num_acc_slots == other.num_acc_slots
            && self.num_drawn_slots == other.num_drawn_slots
            && self.is_primitive == other.is_primitive
            && self.op_id == other.op_id
            && self.is_union == other.is_union
    }
}

pub trait SdfElement: fmt::Debug {
    fn get_info(&self) -> SdfElementInfo;
    fn get_bbox(&self, slots_bboxes: &[SdfBoundingBox]) -> SdfBoundingBox;
    fn downtree_transform(&self, point: Vec3) -> Vec3 {
        point
    }
    fn uptree_transform(&self, point: Vec3) -> Vec3 {
        point
    }
    fn distance_to(&self, point: Vec3) -> f32 {
        point.length()
    }
    fn clone(&self) -> Box<dyn SdfElement>;
    fn get_dt_specific_block(&self) -> SdfOpSpecificBlock {
        SdfOpSpecificBlock::ZERO
    }
    fn get_ut_specific_block(&self) -> SdfOpSpecificBlock {
        SdfOpSpecificBlock::ZERO
    }
    fn expand(&self, this_node: &SdfNode) -> ExpandedSdfNode;
}

#[derive(Debug)]
pub struct SdfSphere {
    pub radius: f32,
}

impl SdfElement for SdfSphere {
    fn get_info(&self) -> SdfElementInfo {
        SdfElementInfo::primitive_info(0)
    }

    fn get_bbox(&self, _slots_bboxes: &[SdfBoundingBox]) -> SdfBoundingBox {
        SdfBoundingBox::from_transform(Transform::from_scale(Vec3::splat(self.radius)))
    }

    fn clone(&self) -> Box<dyn SdfElement> {
        Box::new(SdfSphere {
            radius: self.radius
        })
    }

    fn distance_to(&self, point: Vec3) -> f32 {
        point.length() - self.radius
    }

    fn get_dt_specific_block(&self) -> SdfOpSpecificBlock {
        let mut ret = SdfOpSpecificBlock::ZERO;
        ret.floats[0] = self.radius;
        ret
    }

    fn expand(&self, this_node: &SdfNode) -> ExpandedSdfNode {
        ExpandedSdfNode::primitive(this_node.bbox.unwrap(), self.clone())
    }
}

#[derive(Debug)]
struct SdfBoxFrame {
    pub dimension: Vec3,
    pub thickness: f32,
}

impl SdfElement for SdfBoxFrame {
    fn get_info(&self) -> SdfElementInfo {
        SdfElementInfo::primitive_info(5)
    }

    fn get_bbox(&self, _slots_bboxes: &[SdfBoundingBox]) -> SdfBoundingBox {
        SdfBoundingBox::from_transform(Transform::from_scale(self.dimension))
    }

    fn clone(&self) -> Box<dyn SdfElement> {
        Box::new(SdfBoxFrame {
            dimension: self.dimension,
            thickness: self.thickness,
        })
    }
    
    fn get_dt_specific_block(&self) -> SdfOpSpecificBlock {
        let mut ret = SdfOpSpecificBlock::ZERO;
        ret.vec4s[0] = self.dimension.extend(0.0);
        ret.floats[0] = self.thickness;
        ret
    }
    
    fn expand(&self, this_node: &SdfNode) -> ExpandedSdfNode {
        ExpandedSdfNode::primitive(this_node.bbox.unwrap(), self.clone())
    }
}

// Operations

// Basic smooth union
#[derive(Debug)]
pub struct SdfUnion {
    pub smooth_radius: f32,
}

impl SdfElement for SdfUnion {
    fn get_info(&self) -> SdfElementInfo {
        SdfElementInfo::union_info(0)
    }

    fn get_bbox(&self, slots_bboxes: &[SdfBoundingBox]) -> SdfBoundingBox {
        SdfBoundingBox::merge(slots_bboxes)
    }

    fn expand(&self, this_node: &SdfNode) -> ExpandedSdfNode {
        match this_node.slots.len() {
            1 => this_node.slots.get(0).unwrap().expanded(),
            0 => ExpandedSdfNode::null(),
            _ => {
                fn recurse(this_intern: &SdfUnion, this_node: &SdfNode, index_vec: Vec<usize>) -> ExpandedSdfNode {
                    if index_vec.len() == 1 {
                        return this_node.slots[index_vec[0]].expanded();
                    }

                    let bboxes = index_vec.iter()
                        .map(|i| this_node.slots[*i].bbox.unwrap())
                        .collect::<Vec<SdfBoundingBox>>();
                    let merged_box = SdfBoundingBox::merge(bboxes.as_slice());
                    let (left_child_inds, right_child_inds) = merged_box.split(bboxes.as_slice());
                    
                    ExpandedSdfNode::operation(
                        [
                            Box::new(recurse(this_intern, this_node, left_child_inds)),
                            Box::new(recurse(this_intern, this_node, right_child_inds)),
                        ],
                        merged_box,
                        this_intern.clone(),
                    )
                }
                recurse(self, this_node, (0..this_node.slots.len()).collect())
            },
        }
    }

    fn clone(&self) -> Box<dyn SdfElement> {
        Box::new(SdfUnion {
            smooth_radius: self.smooth_radius,
        })
    }

    fn get_ut_specific_block(&self) -> SdfOpSpecificBlock {
        let mut ret = SdfOpSpecificBlock::ZERO;
        ret.floats[0] = self.smooth_radius;
        ret
    }
}

// Continuous, Axis Aligned clone operation
#[derive(Debug)]
pub struct SdfCaaClone {
    pub displacement: Vec3,
    pub neg_limit: Vec3,
    pub pos_limit: Vec3,
}

impl SdfElement for SdfCaaClone {
    fn get_info(&self) -> SdfElementInfo {
        SdfElementInfo::strict_info(1, 0, 1)
    }

    fn get_bbox(&self, _slots_bboxes: &[SdfBoundingBox]) -> SdfBoundingBox {
        SdfBoundingBox::from_transform(
            Transform::from_translation((self.neg_limit + self.pos_limit) / 2_f32 * self.displacement)
                .with_scale((self.pos_limit - self.neg_limit) * self.displacement)
        )
    }

    fn downtree_transform(&self, point: Vec3) -> Vec3 {
        let lattice_transform = point - (self.displacement / 2.0);
        Vec3::new(
            lattice_transform.x % self.displacement.x,
            lattice_transform.y % self.displacement.y,
            lattice_transform.z % self.displacement.z,
        )
    }

    fn clone(&self) -> Box<dyn SdfElement> {
        Box::new(SdfCaaClone {
            displacement: self.displacement,
            neg_limit: self.neg_limit,
            pos_limit: self.pos_limit,
        })
    }

    fn get_dt_specific_block(&self) -> SdfOpSpecificBlock {
        let mut ret = SdfOpSpecificBlock::ZERO;
        ret.vec4s[0] = self.displacement.extend(0.0);
        ret.vec4s[1] = self.neg_limit.extend(0.0);
        ret.vec4s[2] = self.pos_limit.extend(0.0);
        ret
    }

    fn expand(&self, this_node: &SdfNode) -> ExpandedSdfNode {
        ExpandedSdfNode::simple_operation(this_node.slots.get(0).unwrap().expanded(), self.clone())
    }
}

// Surface Sin Wave
// #[derive(Debug)]
// pub struct SdfSurfaceSin {
//     pub period: f32,
//     pub amplitude: f32,
// }

// impl SdfElement for SdfSurfaceSin {
//     fn get_info(&self) -> SdfElementInfo {
//         SdfElementInfo::strict_info(4, 0, 1)
//     }

//     fn get_bbox(&self, slots_bboxes: &[SdfBoundingBox]) -> SdfBoundingBox {
//         slots_bboxes[0].apply_transform(Transform::from_scale(Vec3::splat(self.amplitude)))
//     }

//     fn downtree_transform(&self, point: Vec3) -> Vec3 {
//         let azimuth = f32::atan2(-point.z, -point.x) + std::f32::consts::PI;
//         let zenith = f32::atan(-point.y / f32::sqrt(point.x * point.x + point.z * point.z)) + std::f32::consts::FRAC_PI_2;
//         let proj_zenith = (1.0 - zenith.cos()) * std::f32::consts::PI;
//         let scale = self.amplitude * (self.period * azimuth).sin() * (self.period * proj_zenith).sin();
//         point * scale
//     }

//     fn clone(&self) -> Box<dyn SdfElement> {
//         Box::new(SdfSurfaceSin {
//             period: self.period,
//             amplitude: self.amplitude,
//         })
//     }
// }