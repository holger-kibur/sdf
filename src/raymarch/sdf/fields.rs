use std::ops::Range;
use std::collections::VecDeque;
use stable_vec::StableVec;
use std::fmt;
use bevy::{
    prelude::*,
};
use super::obb::*;

macro_rules! dyn_assoc_const {
    ($name:ident: $type:ty = $($value:tt)+) => {
        fn $name(&self) -> $type {
            $($value)+
        }
    };
}

pub struct SdfNode {
    slots: StableVec<SdfNode>,
    intern: Box<dyn SdfElement>,
    bbox: Option<SdfBoundingBox>,
}

impl SdfNode {
    pub fn new<T: SdfElement + 'static>(intern: T) -> Self {
        SdfNode {
            slots: StableVec::with_capacity(usize::min(intern.NUM_SLOTS(), 128)),
            intern: Box::new(intern),
            bbox: None,
        }
    }

    pub fn set_slot(&mut self, child_node: SdfNode) -> Result<(), &'static str> {
        if self.intern.IS_PRIMITIVE() {
            Err("Can't set slots on primitive!")
        } else if self.slots.num_elements() >= self.intern.NUM_SLOTS() {
            Err("Slots already full!")
        } else {
            self.slots.push(child_node);
            Ok(())
        }
    }

    pub fn invalidate_bbox(&mut self) {
        self.bbox = None;
    }

    pub fn calculate_bbox(&mut self) -> SdfBoundingBox {
        let bbox = self.intern.get_bbox_from_slots(
            self.slots.iter_mut()
                .map(|(_, node)| node.calculate_bbox())
                .collect::<Vec<SdfBoundingBox>>()
                .as_slice()
        );
        self.bbox = Some(bbox);
        bbox
    }

    pub fn tree_expand(&self) -> SdfNode {
        let mut new_tree = self.full_clone();
        new_tree.get_tree_expansion();
        new_tree
    }

    fn get_tree_expansion(&mut self) -> SdfNode {
        println!("Start tree expansion {:?}, {}", self.intern, self.slots.num_elements());
        if self.intern.TREE_EXPAND() && self.slots.num_elements() >= 2 {
            let (left_child_inds, right_child_inds) = self.calculate_bbox().split(
                &self.slots.iter_mut().map(|(_, child)| child.calculate_bbox()).collect()
            );
            let mut left_children: StableVec<SdfNode> = 
                left_child_inds.iter().map(|child_ind| self.slots.remove(*child_ind)).collect();
            let mut right_children: StableVec<SdfNode> = 
                right_child_inds.iter().map(|child_ind| self.slots.remove(*child_ind)).collect();
            if self.slots.num_elements() == 2 {
                println!("leaf binary expansion");
                assert!(left_children.len() == 1);
                assert!(right_children.len() == 1);
                SdfNode {
                    slots: vec![
                        left_children.pop().unwrap().get_tree_expansion(),
                        right_children.pop().unwrap().get_tree_expansion(),
                    ],
                    intern: Box::new(SdfBinaryUnion {}),
                    bbox: self.bbox
                }
            } else {
                println!("non-binary expansion");
                SdfNode {
                    slots: vec![
                        SdfNode {
                            slots: left_children,
                            intern: self.intern.clone(),
                            bbox: None
                        }.get_tree_expansion(),
                        SdfNode {
                            slots: right_children,
                            intern: self.intern.clone(),
                            bbox: None
                        }.get_tree_expansion()
                    ],
                    intern: Box::new(SdfBinaryUnion {}),
                    bbox: self.bbox
                }
            }
        } else {
            println!("non-expandable interior node: {:?}, slots: {}", self.intern, self.slots.len());
            SdfNode {
                slots: self.slots.iter_mut().map(|child| child.get_tree_expansion()).collect(),
                intern: self.intern.clone(),
                bbox: self.bbox,
            }
        }
    }

    pub fn internal_clone(&self) -> Self {
        SdfNode {
            slots: Vec::with_capacity(self.intern.NUM_SLOTS()),
            intern: self.intern.clone(),
            bbox: self.bbox,
        }
    }

    pub fn full_clone(&self) -> Self {
        SdfNode {
            slots: self.slots.iter().map(|child| child.full_clone()).collect(),
            intern: self.intern.clone(),
            bbox: self.bbox,
        }
    }

    pub fn bf_display(&self) {
        let mut disp_queue: VecDeque<(usize, &SdfNode)> = VecDeque::new();
        disp_queue.push_back((0, self));
        while disp_queue.len() > 0 {
            let (level, front) = disp_queue.pop_front().unwrap();
            for child in front.slots.iter() {
                disp_queue.push_back((level + 1, child));
            }
            println!("{}: {:?}, slots: {}", level, front.intern, front.slots.len());
        }
    }
}

pub trait SdfElement: fmt::Debug {
    // Dynamic constants
    fn NUM_SLOTS(&self) -> usize;
    fn IS_PRIMITIVE(&self) -> bool;
    fn OP_ID(&self) -> u32;
    fn CHILD_SLOT_RANGE(&self) -> Range<usize> {
        0..0 // Empty range for primitives, since they don't have slots
    }
    fn TREE_EXPAND(&self) -> bool;

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
        assert!(prim.IS_PRIMITIVE());
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
        SdfBuilder {
            root: self.root
        }
    }

    pub fn finalize(self) -> SdfNode {
        self.root
    }
}

// Primitives

#[derive(Debug)]
pub struct SdfSphere {
    pub radius: f32,
}

impl SdfElement for SdfSphere {
    dyn_assoc_const!(NUM_SLOTS: usize = 0);
    dyn_assoc_const!(IS_PRIMITIVE: bool = true);
    dyn_assoc_const!(OP_ID: u32 = 0);
    dyn_assoc_const!(TREE_EXPAND: bool = false);

    fn get_bbox_from_slots(&self, slots_bboxes: &[SdfBoundingBox]) -> SdfBoundingBox {
        SdfBoundingBox::from_srt(
            Vec3::splat(self.radius),
            Vec3::ZERO,
            Vec3::ZERO,
        )
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
    dyn_assoc_const!(NUM_SLOTS: usize = usize::MAX);
    dyn_assoc_const!(IS_PRIMITIVE: bool = false);
    dyn_assoc_const!(CHILD_SLOT_RANGE: Range<usize> = 0..usize::MAX);
    dyn_assoc_const!(OP_ID: u32 = 1);
    dyn_assoc_const!(TREE_EXPAND: bool = true);

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
    dyn_assoc_const!(NUM_SLOTS: usize = 2);
    dyn_assoc_const!(IS_PRIMITIVE: bool = false);
    dyn_assoc_const!(CHILD_SLOT_RANGE: Range<usize> = 0..2);
    dyn_assoc_const!(OP_ID: u32 = 2);
    dyn_assoc_const!(TREE_EXPAND: bool = false);

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
    dyn_assoc_const!(NUM_SLOTS: usize = 1);
    dyn_assoc_const!(IS_PRIMITIVE: bool = false);
    dyn_assoc_const!(CHILD_SLOT_RANGE: Range<usize> = 0..1);
    dyn_assoc_const!(OP_ID: u32 = 3);
    dyn_assoc_const!(TREE_EXPAND: bool = false);

    fn get_bbox_from_slots(&self, slots_bboxes: &[SdfBoundingBox]) -> SdfBoundingBox {
        SdfBoundingBox::from_srt(
            self.bounds,
            Vec3::ZERO,
            Vec3::ZERO,
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
    dyn_assoc_const!(NUM_SLOTS: usize = 1);
    dyn_assoc_const!(IS_PRIMITIVE: bool = false);
    dyn_assoc_const!(CHILD_SLOT_RANGE: Range<usize> = 0..1);
    dyn_assoc_const!(OP_ID: u32 = 4);
    dyn_assoc_const!(TREE_EXPAND: bool = false);

    fn get_bbox_from_slots(&self, slots_bboxes: &[SdfBoundingBox]) -> SdfBoundingBox {
        slots_bboxes[0].apply_scale(Vec3::splat(self.amplitude))
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