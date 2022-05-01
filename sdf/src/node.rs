use std::ops::Range;
use std::collections::VecDeque;
use bevy::prelude::*;
use super::{
    obb::*,
    component::*,
    elements::{SdfElement, SdfUnion},
};

pub struct NnResult<'a> {
    pub node: &'a SdfNode,
    pub distance: f32,
}

pub struct NodeDistInfo {
    pub min_bound: f32,
    pub max_bound: f32,
}

pub struct SdfNode {
    pub slots: Vec<SdfNode>,
    pub bbox: Option<SdfBoundingBox>,
    intern: Box<dyn SdfElement>,
}

impl SdfNode {
    pub fn new(intern: Box<dyn SdfElement>) -> Self {
        SdfNode {
            slots: Vec::with_capacity(usize::min(
                intern.get_info().num_slots(),
                128
            )),
            intern: intern,
            bbox: None,
        }
    }

    pub fn empty() -> Self {
        SdfNode {
            slots: Vec::with_capacity(0),
            intern: Box::new(SdfUnion {
                smooth_radius: 0.0,
            }),
            bbox: Some(SdfBoundingBox::zero()),
        }
    }

    pub fn set_slot(&mut self, child_node: SdfNode) -> Result<(), &'static str> {
        let intern_info = self.intern.get_info();
        if intern_info.is_primitive {
            Err("Can't set slots on primitive!")
        } else if self.slots.len() >= intern_info.num_slots() {
            Err("Slots already full!")
        } else {
            self.slots.push(child_node);
            Ok(())
        }
    }

    pub fn calc_bbox_assign(&mut self) -> SdfBoundingBox {
        assert!(
            self.intern.get_info().is_union
                || self.slots.len() >= self.intern.get_info().num_slots(),
            "Tried calculating bounding box of SDF node without all required slots filled!"    
        );
        if let Some(bbox) = self.bbox {
            bbox
        } else {
            let bbox = self.intern.get_bbox(
                self.slots.iter_mut()
                    .map(|node| node.calc_bbox_assign())
                    .filter(|bbox| !bbox.is_zero()) // Filter after bbox calculation so entire tree is initialized
                    .collect::<Vec<SdfBoundingBox>>()
                    .as_slice()
            );
            self.bbox = Some(bbox);
            bbox
        }
    }

    pub fn get_sub_boxes(&self) -> Vec<SdfBoundingBox> {
        self.slots.iter()
            .map(|node| node.bbox)
            .filter(|bbox| !bbox.is_zero()) // Filter after bbox calculation so entire tree is initialized
            .collect::<Vec<SdfBoundingBox>>()
    }

    pub fn expanded(&self) -> ExpandedSdfNode {
        self.intern.expand(self)
    }

    pub fn is_finished(&self) -> bool {
        self.bbox.is_some()
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

    pub fn bf_display(&self) {
        let mut disp_queue: VecDeque<(usize, &SdfNode)> = VecDeque::new();
        disp_queue.push_back((0, self));
        while !disp_queue.is_empty() {
            let (level, front) = disp_queue.pop_front().unwrap();
            for child in front.slots.iter() {
                disp_queue.push_back((level + 1, child));
            }
            println!("{}: {:?}, slots: {}, matrix: {}", level, front.intern, front.slots.len(), front.bbox.scale);
            // front.bbox.unwrap_or(SdfBoundingBox::zero()).get_info()
        }
    }

    pub fn could_contain(&self, point: Vec3) -> bool {
        assert!(
            self.is_finished(),
            "Tried testing point-in-bbox on an unfinished SDF node!"
        );
        self.bbox.contains(point)
    }

    pub fn bbox_dist_info(&self, point: Vec3) -> NodeDistInfo {
        assert!(
            self.is_finished(),
            "Tried getting bounding-box distance info on an unfinished SDF node!"
        );
        NodeDistInfo {
            min_bound: self.bbox.distance_to(point),
            max_bound: self.bbox.max_distance(point)
        }
    }

    /*
    GLSL version:

    struct SdfBoundingBoxBlock {
        mat4 matrix;
        vec3 scale;
        mat4 full_inverse;
        mat4 trans_inverse;
    };

    struct SdfChildIndicesBlock {
        uint left_child_index;
        uint right_child_index;
    };

    struct SdfOperationBlock {
        uint                 op_code;
        SdfChildIndicesBlock op_specific;
        SdfBoundingBoxBlock  bbox;
    };

    layout(std430, binding = 2) buffer SdfTree {
        SdfOperationBlock SdfTree[];
    };

    float bbox_minbound(in mat4 full_inverse, in vec4 scale, in vec4 point) {
        vec4 trans = full_inverse * point;
        vec4 q_local = (abs(trans) - vec4(1.0)) * scale;
        return length(max(q_local, 0.0)) + min(max(q_local.x, max(q_local.y, q_local.z)), 0.0);
    }

    float bbox_maxbound(in mat4 matrix, in vec4 point) {
        return max(
            length((matrix * vec4(1.0, 1.0, 1.0, 1.0)) - point),
            length((matrix * vec4(-1.0, 1.0, 1.0, 1.0)) - point),
            length((matrix * vec4(1.0, -1.0, 1.0, 1.0)) - point),
            length((matrix * vec4(-1.0, -1.0, 1.0, 1.0)) - point),
            length((matrix * vec4(1.0, 1.0, -1.0, 1.0)) - point),
            length((matrix * vec4(-1.0, 1.0, -1.0, 1.0)) - point),
            length((matrix * vec4(1.0, -1.0, -1.0, 1.0)) - point),
            length((matrix * vec4(-1.0, -1.0, -1.0, 1.0)) - point)
        );
    }

    float nearest_neighbor(in vec4 point) {
        uint stack[100];
        uint stack_pointer = 0;
        stack[stack_pointer] = 0;
        float current_nearest = 1.0 / 0.0;
        while (stack_pointer >= 0) {
            SdfOperationBlock left_child = 
            SdfOperationBlock operation = SdfTree[stack[stack_pointer]];
            float left_minbound =
            float left_maxbound = bbox_maxbound(operation.matrix, point);
            float right_minbound = bbox_minbound(operation.full_inverse, operation.scale, point);
            if (left_maxbound < right_minbound) {
                
            }
        }
    }
    */

    pub fn nearest_neighbor(&self, point: Vec3) -> NnResult {
        if self.is_primitive() {
            return NnResult {
                node: self,
                distance: self.intern.distance_to(self.bbox.in_box_trans_basis(point.extend(1.0)).truncate()),
            };
        }
        let mut bounds = self.slots.iter()
            .enumerate()
            .map(|(i, node)| (i, node.bbox_dist_info(point)))
            .collect::<Vec<(usize, NodeDistInfo)>>();
        let min_maxdist = bounds.iter()
            .map(|(_, bound)| CmpFloat(bound.max_bound))
            .min().unwrap().0;
        bounds.sort_unstable_by_key(|(_, bound)| CmpFloat(bound.min_bound));
        bounds.iter()
            .take_while(|(_, bound)| bound.min_bound < min_maxdist)
            .map(|(i, bound)| (self.slots.get(*i).unwrap(), bound))
            .fold(
                NnResult {
                    node: self,
                    distance: f32::INFINITY,
                },
                |accum, (node, bound)| {
                    if bound.min_bound > accum.distance {
                        accum
                    } else {
                        let child_nn = node.nearest_neighbor(node.downtree(
                            self.bbox.in_box_trans_basis(point.extend(1.0)).truncate()
                        ));
                        match child_nn.distance < accum.distance {
                            true => child_nn,
                            false => accum,
                        }
                    }
                }
            )
    }
}