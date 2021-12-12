use std::ops::Range;
use std::collections::VecDeque;
use stable_vec::StableVec;
use std::fmt;
use bevy::{
    prelude::*,
};
use super::obb::*;
use super::component::*;

pub struct NnResult<'a> {
    pub node: &'a SdfNode,
    pub distance: f32,
}

pub struct NodeDistInfo {
    pub min_bound: f32,
    pub max_bound: f32,
}

pub struct ExpandedSdfNode {
    root: SdfNode
}

impl ExpandedSdfNode {
    pub fn make_buffer(&self) -> SdfTreeBuffer {
        let mut op_buffer: Vec<SdfOperationBlock> = Vec::new();

        fn recurse(buffer: &mut Vec<SdfOperationBlock>, root: &SdfNode, level: u32) {
            let intern_info = root.intern.get_info();
            let (union_block, op_specific) = match (intern_info.is_union, intern_info.is_primitive) {
                (true, false) => (root.intern.get_specific_block(), SdfOpSpecificBlock::ZERO),
                (false, false) => (root.slots[1].intern.get_specific_block(), root.intern.get_specific_block()),
                (false, true) => (SdfOpSpecificBlock::ZERO, root.intern.get_specific_block()),
            };
            let this_ind = buffer.len();
            let right_child_len = 0;
            let left_child_len = 0;
            buffer.push(SdfOperationBlock {
                op_code: intern_info.op_id,
                is_primitive: intern_info.is_primitive,
                level,
                left_child_len: 0,
                right_child_len: 0,
                union_block,
                op_specific,
                bounding_box: root.bbox.unwrap().get_bbox_block(),
            });
            let rec_root = {
                if intern_info.is_primitive {
                    return;
                } else if !intern_info.is_union {
                    root.slots.get(1).unwrap()
                } else {
                    root
                }
            };
            recurse(buffer, rec_root.slots.get(1).unwrap(), level + 1);
            right_child_len = buffer.len() - this_ind - 1;
            recurse(buffer, rec_root.slots.get(0).unwrap(), level + 1);
            left_child_len = buffer.len() - right_child_len - this_ind - 1;
            buffer[this_ind].right_child_len = right_child_len as u32;
            buffer[this_ind].left_child_len = left_child_len as u32;
        }

        recurse(&mut op_buffer, &self.root, 0);

        SdfTreeBuffer {
            op_buffer
        }
    }

    pub fn nearest_neighbor_naive(&self, point: Vec3) -> NnResult {
        self.root.nearest_neighbor(point)
    }
}

pub struct SdfNode {
    pub slots: StableVec<SdfNode>,
    intern: Box<dyn SdfElement>,
    pub bbox: Option<SdfBoundingBox>,
    expanded: bool,
}

impl SdfNode {
    pub fn new<T: SdfElement + 'static>(intern: T) -> Self {
        SdfNode {
            slots: StableVec::with_capacity(usize::min(
                intern.get_info().num_slots(),
                128
            )),
            intern: Box::new(intern),
            bbox: None,
            expanded: false,
        }
    }

    pub fn empty() -> Self {
        SdfNode {
            slots: StableVec::with_capacity(0),
            intern: Box::new(SdfUnion {
                smooth_radius: 0.0,
            }),
            bbox: Some(SdfBoundingBox::zero()),
            expanded: true,
        }
    }

    pub fn set_slot(&mut self, child_node: SdfNode) -> Result<(), &'static str> {
        let intern_info = self.intern.get_info();
        if intern_info.is_primitive {
            Err("Can't set slots on primitive!")
        } else if self.slots.num_elements() >= intern_info.num_slots() {
            Err("Slots already full!")
        } else {
            self.slots.push(child_node);
            Ok(())
        }
    }

    pub fn calc_bbox_assign(&mut self) -> SdfBoundingBox {
        assert!(
            self.intern.get_info().is_union |
            (0..self.intern.get_info().num_slots()).all(|ind| self.slots.has_element_at(ind)),
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

    pub fn is_expanded(&self) -> bool {
        (self.slots.num_elements() == 0)
        || (
            self.expanded
            && self.slots.get(0).unwrap().is_expanded()
            && self.slots.get(1).unwrap().is_expanded()
        )
    }

    pub fn is_primitive(&self) -> bool {
        self.intern.get_info().is_primitive
    }

    pub fn is_empty(&self) -> bool {
        self.slots.num_elements() == 0
    }

    pub fn get_tree_expansion(&self) -> ExpandedSdfNode {
        ExpandedSdfNode {
            root: self.full_clone().tree_expand(),
        }
    }

    fn tree_expand(&mut self) -> SdfNode {
        assert!(self.is_finished(), "Tried expanding an unfinished SDF node!");
        let intern_info = self.intern.get_info();
        let div_slots = {
            let mut div_slots: StableVec<SdfNode> = StableVec::with_capacity(2);
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
                        div_slots.push(left_children.remove_first().unwrap().tree_expand());
                        div_slots.push(right_children.remove_first().unwrap().tree_expand());
                        div_slots
                    } else {
                        div_slots.push(
                            SdfNode {
                                slots: left_children,
                                intern: self.intern.clone(),
                                bbox: None,
                                expanded: false,
                            }.box_and_expand()
                        );
                        div_slots.push(
                            SdfNode {
                                slots: right_children,
                                intern: self.intern.clone(),
                                bbox: None,
                                expanded: false,
                            }.box_and_expand()
                        );
                        div_slots
                    }
                } else if self.slots.num_elements() == 1 {
                    return self.slots.remove_first().unwrap().tree_expand();
                } else {
                    return SdfNode::empty();
                }
            } else {
                div_slots.push(
                    SdfNode {
                        slots: (0..intern_info.num_acc_slots).map(|i| self.slots.remove(i).unwrap()).collect(),
                        intern: Box::new(SdfUnion {
                            smooth_radius: 0.0,
                        }),
                        bbox: None,
                        expanded: false,
                    }
                );
                div_slots.push(
                    SdfNode {
                        slots: (intern_info.num_acc_slots..intern_info.num_slots()).map(|i| self.slots.remove(i).unwrap()).collect(),
                        intern: Box::new(SdfUnion {
                            smooth_radius: 0.0,
                        }),
                        bbox: None,
                        expanded: false,
                    }.box_and_expand()
                );
                div_slots
            }
        };
        SdfNode {
            slots: div_slots,
            intern: self.intern.clone(),
            bbox: self.bbox,
            expanded: true,
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
            expanded: self.expanded,
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
            println!("{}: {:?}, slots: {}, matrix: {}", level, front.intern, front.slots.num_elements(), front.bbox.unwrap().scale);
            // front.bbox.unwrap_or(SdfBoundingBox::zero()).get_info()
        }
    }

    pub fn could_contain(&self, point: Vec3) -> bool {
        assert!(
            self.is_finished(),
            "Tried testing point-in-bbox on an unfinished SDF node!"
        );
        self.bbox.unwrap().contains(point)
    }

    pub fn bbox_dist_info(&self, point: Vec3) -> NodeDistInfo {
        assert!(
            self.is_finished(),
            "Tried getting bounding-box distance info on an unfinished SDF node!"
        );
        NodeDistInfo {
            min_bound: self.bbox.unwrap().distance_to(point),
            max_bound: self.bbox.unwrap().max_distance(point)
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
            NnResult {
                node: self,
                distance: self.intern.distance_to(
                    self.bbox.unwrap().in_box_trans_basis(point.extend(1.0)).truncate()
                ),
            }
        } else {
            let mut min_maxbounds = self.slots.iter()
                .map(|(i, node)| (i, node.bbox_dist_info(point)))
                .collect::<Vec<(usize, NodeDistInfo)>>();
            let min_maxbound_dist = min_maxbounds.iter()
                .map(|dist_info| CmpFloat(dist_info.1.max_bound))
                .min().unwrap().0;
            min_maxbounds.sort_unstable_by_key(|dist_info| CmpFloat(dist_info.1.min_bound));
            min_maxbounds.iter()
                .take_while(|(_, dist_info)| dist_info.min_bound < min_maxbound_dist)
                .map(|(i, dist_info)| (self.slots.get(*i).unwrap(), dist_info))
                .fold(
                    NnResult {
                        node: self,
                        distance: f32::INFINITY,
                    },
                    |acc, (node, dist_info)| {
                        if dist_info.min_bound >= acc.distance {
                            acc
                        } else {
                            let child_nn = node.nearest_neighbor(node.downtree(point));
                            if child_nn.distance < acc.distance {
                                child_nn
                            } else {
                                acc
                            }
                        }
                    }
                )
        }
    }
}

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
    fn get_bbox_from_slots(&self, slots_bboxes: &[SdfBoundingBox]) -> SdfBoundingBox;
    fn downtree_transform(&self, point: Vec3) -> Vec3 {
        point
    }
    fn distance_to(&self, point: Vec3) -> f32 {
        point.length()
    }
    fn clone(&self) -> Box<dyn SdfElement>;
    fn get_specific_block(&self) -> SdfOpSpecificBlock;
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

    fn distance_to(&self, point: Vec3) -> f32 {
        point.length() - self.radius
    }

    fn get_specific_block(&self) -> SdfOpSpecificBlock {
        SdfOpSpecificBlock {
            vec4s: [Vec4::ZERO, Vec4::ZERO],
            floats: [self.radius, 0.0],
        }
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
    
    fn get_specific_block(&self) -> SdfOpSpecificBlock {
        SdfOpSpecificBlock {
            vec4s: [self.dimension.extend(0.0), Vec4::ZERO],
            floats: [self.thickness, 0.0],
        }
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

    fn get_specific_block(&self) -> SdfOpSpecificBlock {
        SdfOpSpecificBlock {
            vec4s: [Vec4::ZERO, Vec4::ZERO],
            floats: [self.smooth_radius, 0.0],
        }
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
        SdfElementInfo::strict_info(3, 0, 1)
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

    fn get_specific_block(&self) -> SdfOpSpecificBlock {
        SdfOpSpecificBlock {
            vec4s: [self.displacement.extend(0.0), self.bounds.extend(0.0)],
            floats: [0.0, 0.0],
        }
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
        SdfElementInfo::strict_info(4, 0, 1)
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

    fn get_specific_block(&self) -> SdfOpSpecificBlock {
        SdfOpSpecificBlock {
            vec4s: [Vec4::ZERO, Vec4::ZERO],
            floats: [self.period, self.amplitude],
        }
    }
}

#[derive(Debug)]
pub struct SdfLineSegment {
    pub origin: Vec3,
    pub direction: Vec3,
    pub length: f32,
}