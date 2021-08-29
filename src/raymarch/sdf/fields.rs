use std::ops::Range;
use std::collections::VecDeque;
use stable_vec::StableVec;
use std::fmt;
use bevy::{
    prelude::*,
};
use super::obb::*;

pub struct SdfNode {
    pub slots: StableVec<SdfNode>,
    intern: Box<dyn SdfElement>,
    bbox: Option<SdfBoundingBox>,
}

impl SdfNode {
    pub fn new<T: SdfElement + 'static>(intern: T) -> Self {
        SdfNode {
            slots: StableVec::with_capacity(usize::min(intern.get_info().num_slots, 128)),
            intern: Box::new(intern),
            bbox: None,
        }
    }

    pub fn set_slot(&mut self, child_node: SdfNode) -> Result<(), &'static str> {
        let intern_info = self.intern.get_info();
        if intern_info.is_primitive {
            Err("Can't set slots on primitive!")
        } else if self.slots.num_elements() >= intern_info.num_slots {
            Err("Slots already full!")
        } else {
            self.slots.push(child_node);
            Ok(())
        }
    }

    pub fn calc_bbox_assign(&mut self) -> SdfBoundingBox {
        assert!(
            (0..self.intern.get_info().min_slots).all(|ind| self.slots.has_element_at(ind)),
            "Tried calculating bounding box of SDF node without all required slots filled!"    
        );
        if let Some(bbox) = self.bbox {
            bbox
        } else {
            let bbox = self.intern.get_bbox_from_slots(
                self.slots.iter_mut()
                    .map(|(_, node)| node.calc_bbox_assign())
                    .filter(|bbox| !bbox.is_zero()) // Filter after bbox calculation so entire tree is initialized
                    .collect::<Vec<SdfBoundingBox>>()
                    .as_slice()
            );
            self.bbox = Some(bbox);
            bbox
        }
    }

    fn box_and_expand(mut self) -> SdfNode {
        self.calc_bbox_assign();
        self.tree_expand()
    }

    pub fn is_finished(&self) -> bool {
        self.bbox.is_some()
    }

    pub fn get_tree_expansion(&self) -> SdfNode {
        self.full_clone().tree_expand()
    }

    fn tree_expand(&mut self) -> SdfNode {
        assert!(self.is_finished(), "Tried expanding an unfinished SDF node!");
        let intern_info = self.intern.get_info();
        if intern_info.is_union {
            if self.slots.num_elements() >= 2 {
                let (left_child_inds, right_child_inds) = self.bbox.unwrap().split(
                    &self.slots.iter_mut()
                        .map(|(_, child)| child.calc_bbox_assign())
                        .collect::<Vec<SdfBoundingBox>>()
                        .as_slice()
                );
                let mut left_children: StableVec<SdfNode> = left_child_inds.iter().map(|child_ind| self.slots.remove(*child_ind).unwrap()).collect();
                let mut right_children: StableVec<SdfNode> = right_child_inds.iter().map(|child_ind| self.slots.remove(*child_ind).unwrap()).collect();
                if left_children.num_elements() == 1 && right_children.num_elements() == 1 {
                    SdfNode {
                        slots: {
                            let mut div_slots: StableVec<SdfNode> = StableVec::with_capacity(2);
                            div_slots.push(left_children.remove_first().unwrap().tree_expand());
                            div_slots.push(right_children.remove_first().unwrap().tree_expand());
                            div_slots
                        },
                        intern: self.intern.clone(),
                        bbox: self.bbox
                    }
                } else {
                    SdfNode {
                        slots: {
                            let mut div_slots: StableVec<SdfNode> = StableVec::with_capacity(2);
                            div_slots.push(
                                SdfNode {
                                    slots: left_children,
                                    intern: self.intern.clone(),
                                    bbox: None
                                }.box_and_expand()
                            );
                            div_slots.push(
                                SdfNode {
                                    slots: right_children,
                                    intern: self.intern.clone(),
                                    bbox: None
                                }.box_and_expand()
                            );
                            div_slots
                        },
                        intern: self.intern.clone(),
                        bbox: self.bbox
                    }
                }
            } else {
                self.slots.remove_first().unwrap().tree_expand()
            }
        } else {
            SdfNode {
                slots: self.slots.iter_mut().map(|(_, child)| child.tree_expand()).collect(),
                intern: self.intern.clone(),
                bbox: self.bbox,
            }
        }
    }

    pub fn downtree(&self, point: Vec3) -> Vec3 {
        self.intern.downtree_transform(point)
    }

    pub fn full_clone(&self) -> Self {
        SdfNode {
            slots: self.slots.iter().map(|(_, child)| child.full_clone()).collect(),
            intern: self.intern.clone(),
            bbox: self.bbox,
        }
    }

    pub fn bf_display(&self) {
        let mut disp_queue: VecDeque<(usize, &SdfNode)> = VecDeque::new();
        disp_queue.push_back((0, self));
        while !disp_queue.is_empty() {
            let (level, front) = disp_queue.pop_front().unwrap();
            for (_, child) in front.slots.iter() {
                disp_queue.push_back((level + 1, child));
            }
            println!("{}: {:?}, slots: {}", level, front.intern, front.slots.num_elements());
            front.bbox.unwrap_or(SdfBoundingBox::zero()).get_info()
        }
    }

    pub fn could_contain(&self, point: Vec3) -> bool {
        assert!(
            self.is_finished(),
            "Tried testing point-in-bbox on an unfinished SDF node!"
        );
        self.bbox.unwrap().contains(point)
    }

    pub fn diverges(&self, point: Vec3) -> Vec<&SdfNode> {
        assert!(
            self.is_finished(), 
            "Tried testing point-tree divergence on an unfinished SDF node!"
        );
        if self.bbox.unwrap().contains(point) {
            self.interior_diverges(point)
        } else {
            Vec::new()
        }
    }

    fn interior_diverges(&self, point: Vec3) -> Vec<&SdfNode> {
        let diverge_list: Vec<&SdfNode> = self.slots.iter()
            .filter(|(_, node)| node.could_contain(self.downtree(point)))
            .map(|(_, node)| node.interior_diverges(point))
            .flatten()
            .collect();
        if diverge_list.len() == 0 {
            vec![self]
        } else {
            diverge_list
        }
    }

    // pub fn nearest_neighbor(&self, point: Vec3) -> &SdfNode {
    //     // Intentionally scuffed nearest neighbor search because we want this
    //     // to be as close to the shader code as possible.
    //     let mut index_stack: VecDeque<usize> = VecDeque::new();
    //     index_stack.push_back(0);
    //     while !index_stack.is_empty() {
    //         // This part won't be in the shader code because the tree will be flattened.
    //         let (parent_ref, trans_point) = index_stack[..-1].iter().fold(
    //             (self, point),
    //             |acc, x| (&acc.0.slots[*x], acc.0.downtree(acc.1))
    //         );
    //         // Starting from here is the shader code
    //         let top_index = *index_stack.back().unwrap();
    //         if !parent_ref[top_index].slots.is_empty() {

    //         }
    //     }
    // }
}

pub struct SdfElementInfo {
    pub num_slots: usize,
    pub is_primitive: bool,
    pub op_id: u32,
    pub is_union: bool,
    pub min_slots: usize,
}

impl SdfElementInfo {
    pub fn primitive_info(op_id: u32) -> Self {
        SdfElementInfo {
            num_slots: 0,
            is_primitive: true,
            op_id,
            is_union: false,
            min_slots: 0,
        }
    }

    pub fn strict_info(op_id: u32, num_slots: usize) -> Self {
        SdfElementInfo {
            num_slots,
            is_primitive: false,
            op_id,
            is_union: false,
            min_slots: num_slots,
        }
    }

    pub fn union_info(op_id: u32) -> Self {
        SdfElementInfo {
            num_slots: usize::MAX,
            is_primitive: false,
            op_id,
            is_union: true,
            min_slots: 2,
        }
    }
}

pub struct SdfBuilder {
    root: SdfNode,
}

impl SdfBuilder {
    pub fn primitive<T: SdfElement + 'static>(prim: T) -> Self {
        assert!(prim.get_info().is_primitive);
        SdfBuilder {
            root: SdfNode::new(prim)
        }
    }

    pub fn operation<T: SdfElement + 'static>(self, op: T) -> Self {
        let mut new_node = SdfNode::new(op);
        new_node.set_slot(self.root).expect("Couldn't set operation child");
        SdfBuilder {
            root: new_node
        }
    }

    pub fn with(mut self, node: SdfBuilder) -> Self {
        self.root.set_slot(node.root).expect("Couldn't set operation slot");
        self
    }

    pub fn transform(mut self, trans: Transform) -> Self {
        self.root.bbox = Some(self.root.calc_bbox_assign().apply_transform(trans));
        self
    }

    pub fn finalize(mut self) -> SdfNode {
        self.root.calc_bbox_assign();
        self.root
    }
}

pub trait SdfElement: fmt::Debug {
    fn get_info(&self) -> SdfElementInfo;
    // Actual logic
    fn get_bbox_from_slots(&self, slots_bboxes: &[SdfBoundingBox]) -> SdfBoundingBox;
    // Doesn't have to be implemented for primitives, since they are leaf nodes
    fn downtree_transform(&self, point: Vec3) -> Vec3 {
        point
    }
    fn clone(&self) -> Box<dyn SdfElement>;
}

// Primitives

#[derive(Debug)]
pub struct SdfSphere {
    pub radius: f32,
}

impl SdfElement for SdfSphere {
    fn get_info(&self) -> SdfElementInfo {
        SdfElementInfo::primitive_info(0)
    }

    fn get_bbox_from_slots(&self, _slots_bboxes: &[SdfBoundingBox]) -> SdfBoundingBox {
        SdfBoundingBox::from_transform(Transform::from_scale(Vec3::splat(self.radius)))
    }
    
    fn clone(&self) -> Box<dyn SdfElement> {
        Box::new(SdfSphere {
            radius: self.radius
        })
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

    fn get_bbox_from_slots(&self, _slots_bboxes: &[SdfBoundingBox]) -> SdfBoundingBox {
        SdfBoundingBox::from_transform(Transform::from_scale(self.dimension))
    }

    fn clone(&self) -> Box<dyn SdfElement> {
        Box::new(SdfBoxFrame {
            dimension: self.dimension,
            thickness: self.thickness,
        })
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
        SdfElementInfo::union_info(1)
    }

    fn get_bbox_from_slots(&self, slots_bboxes: &[SdfBoundingBox]) -> SdfBoundingBox {
        SdfBoundingBox::merge(slots_bboxes)
    }

    fn clone(&self) -> Box<dyn SdfElement> {
        Box::new(SdfUnion {
            smooth_radius: self.smooth_radius,
        })
    }
}

// Continuous, Axis Aligned clone operation
#[derive(Debug)]
pub struct SdfCaaClone {
    pub displacement: Vec3,
    pub bounds: Vec3,
}

impl SdfElement for SdfCaaClone {
    fn get_info(&self) -> SdfElementInfo {
        SdfElementInfo::strict_info(3, 1)
    }

    fn get_bbox_from_slots(&self, _slots_bboxes: &[SdfBoundingBox]) -> SdfBoundingBox {
        SdfBoundingBox::from_transform(Transform::from_scale(self.bounds))
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
            bounds: self.bounds,
        })
    }
}

// Surface Sin Wave
#[derive(Debug)]
pub struct SdfSurfaceSin {
    pub period: f32,
    pub amplitude: f32,
}

impl SdfElement for SdfSurfaceSin {
    fn get_info(&self) -> SdfElementInfo {
        SdfElementInfo::strict_info(4, 1)
    }

    fn get_bbox_from_slots(&self, slots_bboxes: &[SdfBoundingBox]) -> SdfBoundingBox {
        slots_bboxes[0].apply_transform(Transform::from_scale(Vec3::splat(self.amplitude)))
    }

    fn downtree_transform(&self, point: Vec3) -> Vec3 {
        let azimuth = f32::atan2(-point.z, -point.x) + std::f32::consts::PI;
        let zenith = f32::atan(-point.y / f32::sqrt(point.x * point.x + point.z * point.z)) + std::f32::consts::FRAC_PI_2;
        let proj_zenith = (1.0 - zenith.cos()) * std::f32::consts::PI;
        let scale = self.amplitude * (self.period * azimuth).sin() * (self.period * proj_zenith).sin();
        point * scale
    }

    fn clone(&self) -> Box<dyn SdfElement> {
        Box::new(SdfSurfaceSin {
            period: self.period,
            amplitude: self.amplitude,
        })
    }
}

#[derive(Debug)]
pub struct SdfLineSegment {
    pub origin: Vec3,
    pub direction: Vec3,
    pub length: f32,
}