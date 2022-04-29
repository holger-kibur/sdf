use super::{
    component::*,
    elements::{SdfElement, SdfUnion},
    expanded_node::*,
    obb::*,
};
use bevy::prelude::*;
use std::collections::VecDeque;

pub struct SdfNode {
    pub slots: Vec<SdfNode>,
    pub bbox: SdfBoundingBox,
    intern: Box<dyn SdfElement>,
}

impl SdfNode {
    pub fn from_slots(intern: Box<dyn SdfElement>, slots: Vec<SdfNode>) -> Self {
        SdfNode {
            bbox: intern.get_bbox(
                slots
                    .iter()
                    .map(|node| node.bbox)
                    .collect::<Vec<SdfBoundingBox>>()
                    .as_slice(),
            ),
            intern,
            slots,
        }
    }

    pub fn empty(intern: Box<dyn SdfElement>) -> Self {
        SdfNode {
            slots: Vec::new(),
            bbox: intern.get_bbox(&[]),
            intern,
        }
    }

    pub fn expanded(&self) -> ExpandedSdfNode {
        self.intern.expand(self)
    }

    pub fn is_primitive(&self) -> bool {
        self.intern.get_info().is_primitive
    }

    pub fn is_empty(&self) -> bool {
        self.slots.len() == 0
    }

    pub fn downtree(&self, point: Vec3) -> Vec3 {
        self.intern.downtree_transform(point)
    }

    pub fn full_clone(&self) -> Self {
        SdfNode {
            slots: self.slots.iter().map(|child| child.full_clone()).collect(),
            intern: self.intern.clone(),
            bbox: self.bbox,
        }
    }
}
