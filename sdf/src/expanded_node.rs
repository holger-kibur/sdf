use std::ops::Range;
use std::collections::VecDeque;
use bevy::prelude::*;
use super::{
    obb::*,
    component::*,
    elements::{SdfElement, SdfUnion},
};

pub struct ExpandedSdfNode {
    expanded_slots: Option<[Box<ExpandedSdfNode>; 2]>,
    pub bbox: SdfBoundingBox,
    intern: Option<Box<dyn SdfElement>>,
}

impl ExpandedSdfNode {
    pub fn null() -> Self {
        ExpandedSdfNode {
            expanded_slots: None,
            bbox: SdfBoundingBox::zero(),
            intern: None,
        }
    }

    pub fn primitive(bbox: SdfBoundingBox, intern: Box<dyn SdfElement>) -> Self {
        ExpandedSdfNode {
            expanded_slots: None,
            bbox,
            intern: Some(intern),
        }
    }

    pub fn simple_operation(downtree_union: ExpandedSdfNode, intern: Box<dyn SdfElement>) -> Self {
        ExpandedSdfNode {
            bbox: downtree_union.bbox,
            expanded_slots: Some([Box::new(downtree_union), Box::new(Self::null())]),
            intern: Some(intern),
        }
    }

    pub fn operation(expanded_slots: [Box<ExpandedSdfNode>; 2], bbox: SdfBoundingBox, intern: Box<dyn SdfElement>) -> Self {
        ExpandedSdfNode {
            expanded_slots: Some(expanded_slots),
            bbox,
            intern: Some(intern),
        }
    }

    pub fn is_null(&self) -> bool {
        self.intern.is_none()
    }

    pub fn is_primitive(&self) -> bool {
        self.expanded_slots.is_none() && self.intern.is_some()
    }

    pub fn is_operation(&self) -> bool {
        self.expanded_slots.is_some() && self.intern.is_some()
    }

    pub fn is_union(&self) -> bool {
        self.is_operation() && self.intern.as_ref().unwrap().get_info().is_union
    }

    pub fn make_buffer(&self) -> SdfTreeBuffer {
        let mut buffer = SdfTreeBuffer::make_empty();

        fn recurse(
            buffer: &mut SdfTreeBuffer,
            root: &ExpandedSdfNode,
            other_box: &SdfBoundingBox,
            level: u32,
            parent_is_union: bool,
        ) {
            if root.is_null() {
                return;
            }

            let intern = root.intern.as_ref().unwrap();
            
            let intern_info = intern.get_info();
            let dt_block_spec = intern.get_dt_specific_block();
            let ut_block_spec = intern.get_ut_specific_block();
            let this_ind = buffer.downtree_buffer.len() as usize;


            buffer.downtree_buffer.push(SdfOperationBlock {
                op_code: intern_info.op_id,
                is_primitive: intern_info.is_primitive,
                parent_is_union,
                len: 0,
                level,
                op_specific: dt_block_spec,
                bounding_box: root.bbox.get_bbox_block(),
                other_box: other_box.get_bbox_block(),
            });

            if !root.is_primitive() {
                let exp_slots = root.expanded_slots.as_ref().unwrap();
                recurse(
                    buffer,
                    &exp_slots[0],
                    &exp_slots[0].bbox,
                    level + 1,
                    root.is_union());
                recurse(
                    buffer,
                    &exp_slots[1],
                    &exp_slots[1].bbox,
                    level + 1,
                    root.is_union());
            }

            buffer.uptree_buffer.push(SdfOperationUptreeBlock {
                op_code: intern_info.op_id,
                parent_is_union,
                op_specific: ut_block_spec,
                level,
            });

            buffer.downtree_buffer[this_ind].len = (buffer.downtree_buffer.len() - this_ind - 1) as u32;
        }

        let throwaway_box = SdfBoundingBox::zero();
        recurse(&mut buffer, self, &throwaway_box, 1, false);
        buffer.buffer_len = buffer.downtree_buffer.len() as u32;
        buffer
    }
}