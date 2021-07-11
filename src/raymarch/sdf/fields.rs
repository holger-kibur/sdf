use enum_dispatch::enum_dispatch;
use std::ops::{Range, RangeBounds};
use bevy::{
    prelude::*,
    reflect::TypeUuid,
};
use super::obb::*;

macro_rules! dyn_assoc_const {
    ($name:ident: $type:ty = $($value:tt)+) => {
        fn $name(&self) -> $type {
            $($value)+
        }
    };
}

#[derive(TypeUuid)]
#[uuid = "56cca261-abd7-462c-98b7-6dc4aef22765"]
pub enum SdfEntity {
    
}

pub struct SdfNode {
    slots: Vec<SdfNode>,
    intern: Box<dyn SdfElement>,
    bbox: Option<SdfBoundingBox>,
}

impl SdfNode {
    pub fn new<T: SdfElement + 'static>(intern: T) -> Self {
        SdfNode {
            slots: Vec::with_capacity(intern.NUM_SLOTS()),
            intern: Box::new(intern),
            bbox: None,
        }
    }

    pub fn set_slot(&mut self, child_node: SdfNode) -> Result<(), &'static str> {
        if self.intern.IS_PRIMITIVE() {
            Err("Can't set slots on primitive!")
        } else if self.slots.len() >= self.intern.NUM_SLOTS() {
            Err("Slots already full!")
        } else {
            self.slots.push(child_node);
            Ok(())
        }
    }

    pub fn invalidate_bbox(&mut self) {
        self.bbox = None;
    }

    pub fn get_bbox(&mut self) -> SdfBoundingBox {
        if let Some(bbox) = self.bbox {
            bbox
        } else {
            let bbox = self.intern.get_bbox_from_slots(
                self.slots.iter()
                    .map(|node| node.get_bbox())
                    .collect::<Vec<SdfBoundingBox>>()
                    .as_slice()
            );
            self.bbox = Some(bbox);
            bbox
        }
    }

    pub fn children(&self) -> &[SdfNode] {
        &self.slots[self.intern.CHILD_SLOT_RANGE()]
    }

    pub fn get_tree_expansion(&self) -> SdfNode {
        if self.intern.TREE_EXPAND() && self.slots.len() >= 2 {
            let (left_children, right_children) = self.get_bbox().split(
                self.slots.iter().map(|child| child.get_bbox()).collect()
            );
            if self.slots.len() == 2 {
                
            } else {
                let left_union = SdfNode {
                    slots: left_children,
                    intern: self.intern.clone(),
                    bbox: None
                }.get_tree_expansion();
                let right_union = SdfNode {
                    slots: right_children,
                    intern: self.intern.clone(),
                    bbox: None
                }.get_tree_expansion();
                SdfNode {
                    slots: vec![left_union, right_union],
                    intern: Box::new(SdfBinaryUnion {}),
                    bbox: self.bbox
                }
            }
        } else {
            SdfNode {
                slots: self.slots.iter().map(|child| child.get_tree_expansion()).collect(),
                intern: self.intern.clone(),
                bbox: self.bbox,
            }
        }
    }
}

pub trait SdfElement {
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
        let new_node = SdfNode::new(op);
        new_node.set_slot(self.root);
        SdfBuilder {
            root: new_node
        }
    }

    pub fn with(self, node: SdfBuilder) -> Self {
        self.root.set_slot(node.root).expect("Couldn't set operation slot");
        SdfBuilder {
            root: self.root
        }
    }
}

// Primitives

struct SdfSphere {
    radius: f32,
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
struct SdfUnion {
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

// Binary Union
struct SdfBinaryUnion {}

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
struct SdfCaaClone {
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
struct SdfSurfaceSin {
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

// let cloned_wavy = SdfBuilder::new()
//     .primitive(SdfSphere {
//         radius: 0.5
//     })
//     .operation(SdfSurfaceSin {
//         period: 1.0,
//         amplitude: 1.0,
//     })
//     .operation(SdfCaaClone {
//         displacement: Vec3::new(3.0, 3.0, 0.0),
//         bounds: Vec3::new(20.0, 20.0, 20.0),
//     })
//     .finalize()

// let new_obj = SdfBuilder::new()
//     .primitive(SdfSphere {
//         radius: 0.5
//     })
//     .function(SurfaceWave {
//         amp: 1.0
//     })
//     .function(LineExtrusion {})
//     .with(
//         SdfBuilder::new().primitive(SdfLine {
//             length: 5.0,
//         }})
//     )
//     .function(PointClone {})
//     .with(
//         SdfBuilder::new().primitive(SdfPoint)
//     )