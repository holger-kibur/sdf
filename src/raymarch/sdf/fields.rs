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
            self.intern.get_info().required_range.all(|ind| self.slots.has_element_at(ind)),
            "Tried calculating bounding box of SDF node without all required slots filled!"    
        );
        let bbox = self.intern.get_bbox_from_slots(
            self.slots.iter_mut()
                .map(|(_, node)| node.calc_bbox_assign())
                .collect::<Vec<SdfBoundingBox>>()
                .as_slice()
        );
        self.bbox = Some(bbox);
        bbox
    }

    pub fn is_finished(&self) -> bool {
        self.bbox.is_some()
    }

    pub fn get_tree_expansion(&self) -> SdfNode {
        self.full_clone().tree_expand()
    }

    fn tree_expand(&mut self) -> SdfNode {
        assert!(self.is_finished(), "Tried expanding an unfinished SDF node!");
        println!("Start tree expansion {:?}, {}", self.intern, self.slots.num_elements());
        let intern_info = self.intern.get_info();
        if intern_info.tree_expand && self.slots.num_elements() >= 2 {
            let (left_child_inds, right_child_inds) = self.bbox.unwrap().split(
                (intern_info.child_range.start..intern_info.child_range.end.min(self.slots.num_elements()))
                    .map(|i| self.slots.get(i).unwrap().bbox.unwrap())
                    .collect::<Vec<SdfBoundingBox>>()
                    .as_slice()
            );
            let mut left_children: StableVec<SdfNode> = left_child_inds.iter().map(|child_ind| self.slots.remove(*child_ind).unwrap()).collect();
            let mut right_children: StableVec<SdfNode> = right_child_inds.iter().map(|child_ind| self.slots.remove(*child_ind).unwrap()).collect();
            if left_children.num_elements() == 1 && right_children.num_elements() == 1 {
                println!("leaf binary expansion");
                SdfNode {
                    slots: {
                        let mut div_slots: StableVec<SdfNode> = StableVec::with_capacity(2);
                        div_slots.push(left_children.remove_first().unwrap().tree_expand());
                        div_slots.push(right_children.remove_first().unwrap().tree_expand());
                        div_slots
                    },
                    intern: Box::new(SdfBinaryUnion {}),
                    bbox: self.bbox
                }
            } else {
                println!("non-binary expansion");
                SdfNode {
                    slots: {
                        let mut div_slots: StableVec<SdfNode> = StableVec::with_capacity(2);
                        div_slots.push(
                            SdfNode {
                                slots: left_children,
                                intern: self.intern.clone(),
                                bbox: None
                            }.tree_expand()
                        );
                        div_slots.push(
                            SdfNode {
                                slots: right_children,
                                intern: self.intern.clone(),
                                bbox: None
                            }.tree_expand()
                        );
                        div_slots
                    },
                    intern: Box::new(SdfBinaryUnion {}),
                    bbox: self.bbox
                }
            }
        } else {
            println!("non-expandable interior node: {:?}, slots: {}", self.intern, self.slots.num_elements());
            SdfNode {
                slots: self.slots.iter_mut().map(|(_, child)| child.tree_expand()).collect(),
                intern: self.intern.clone(),
                bbox: self.bbox,
            }
        }
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
}

pub struct SdfElementInfo {
    pub num_slots: usize,
    pub is_primitive: bool,
    pub op_id: u32,
    pub tree_expand: bool,
    pub child_range: Range<usize>,
    pub required_range: Range<usize>,
}

impl SdfElementInfo {
    pub fn primitive_info(op_id: u32) -> Self {
        SdfElementInfo {
            num_slots: 0,
            is_primitive: true,
            op_id,
            tree_expand: false,
            child_range: 0..0,
            required_range: 0..0,
        }
    }

    pub fn strict_info(op_id: u32, num_slots: usize) -> Self {
        SdfElementInfo {
            num_slots,
            is_primitive: false,
            op_id,
            tree_expand: false,
            child_range: 0..num_slots,
            required_range: 0..num_slots,
        }
    }
}

#[allow(non_snake_case)] // Dynamic constants should act as constants, so name them accordingly
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

// Operations

// Basic smooth union
#[derive(Debug)]
pub struct SdfUnion {
    pub smooth_radius: f32,
}

impl SdfElement for SdfUnion {
    fn get_info(&self) -> SdfElementInfo {
        SdfElementInfo {
            num_slots: usize::MAX,
            is_primitive: false,
            op_id: 1,
            tree_expand: true,
            child_range: 0..usize::MAX,
            required_range: 0..2,
        }
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

// Binary Union (for tree construction)
#[derive(Debug)]
pub struct SdfBinaryUnion {}

impl SdfElement for SdfBinaryUnion {
    fn get_info(&self) -> SdfElementInfo {
        SdfElementInfo::strict_info(2, 2)
    }

    fn get_bbox_from_slots(&self, slots_bboxes: &[SdfBoundingBox]) -> SdfBoundingBox {
        SdfBoundingBox::merge(slots_bboxes)
    }
    
    fn clone(&self) -> Box<dyn SdfElement> {
        Box::new(SdfBinaryUnion {})
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