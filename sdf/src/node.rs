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
            .map(|node| node.bbox.unwrap())
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
            println!("{}: {:?}, slots: {}, matrix: {}", level, front.intern, front.slots.len(), front.bbox.unwrap().scale);
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
            return NnResult {
                node: self,
                distance: self.intern.distance_to(self.bbox.unwrap().in_box_trans_basis(point.extend(1.0)).truncate()),
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
                            self.bbox.unwrap().in_box_trans_basis(point.extend(1.0)).truncate()
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

pub struct SdfBuilder {
    root: SdfNode,
}

impl SdfBuilder {
    pub fn dyn_primitive(prim: Box<dyn SdfElement>) -> Self {
        assert!(prim.get_info().is_primitive);
        SdfBuilder {
            root: SdfNode::new(prim)
        }
    }

    pub fn primitive<T: SdfElement + 'static>(prim: T) -> Self {
        Self::dyn_primitive(Box::new(prim))
    }

    pub fn dyn_operation(self, op: Box<dyn SdfElement>) -> Self {
        assert!(!op.get_info().is_primitive);
        let mut new_node = SdfNode::new(op);
        new_node.set_slot(self.root).expect("Couldn't set operation child");
        SdfBuilder {
            root: new_node
        }
    }

    pub fn operation<T: SdfElement + 'static>(self, op: T) -> Self {
        self.dyn_operation(Box::new(op))
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

#[cfg(test)]
pub mod tests {
    use rand::prelude::*;
    use std::f32::consts::{PI, FRAC_PI_2, SQRT_2};
    use bevy::prelude::*;
    use crate::{
        node::*,
        elements::*,
    };
    use float_cmp::approx_eq;

    #[derive(Debug)]
    struct TestPrimitive {
        pub scale: f32,
        pub max_dev: f32,
    }

    impl SdfElement for TestPrimitive {
        fn get_info(&self) -> SdfElementInfo {
            SdfElementInfo::primitive_info(1)
        }

        fn get_bbox(&self, _slots_bboxes: &[SdfBoundingBox]) -> SdfBoundingBox {
            SdfBoundingBox::from_transform(Transform::from_scale(Vec3::splat(self.scale)))
        }
        
        fn clone(&self) -> Box<dyn SdfElement> {
            Box::new(TestPrimitive {
                scale: self.scale,
                max_dev: self.max_dev,
            })
        }

        fn distance_to(&self, point: Vec3) -> f32 {
            let sectors = 360.0 / (self.max_dev / SQRT_2);
            let azimuth = f32::atan2(point.z, point.x) + PI;
            let zenith = f32::atan(point.y / f32::sqrt(point.x * point.x + point.y * point.y)) + FRAC_PI_2;
            let azimuth_quant = (azimuth * sectors).trunc();
            let zenith_quant = (zenith * sectors / 2.0).trunc();
            let paired = zenith_quant * sectors + azimuth_quant;
            point.length() - paired / (sectors * sectors / 2.0) * self.scale
        }

        fn get_dt_specific_block(&self) -> SdfOpSpecificBlock {
            let mut ret = SdfOpSpecificBlock::ZERO;
            ret.floats[0] = self.scale;
            ret.floats[1] = self.max_dev;
            ret
        }

        fn expand(&self, this_node: &SdfNode) -> ExpandedSdfNode {
            ExpandedSdfNode::primitive(this_node.bbox.unwrap(), self.clone())
        }
    }

    fn get_random_transforms(count: u32) -> Vec<Transform> {
        let mut rng = thread_rng();
        (0..count)
            .map(|_| {
                if rng.gen::<bool>() {
                    Transform::from_translation(Vec3::new(
                        rng.gen_range(-50.0..50.0),
                        rng.gen_range(-50.0..50.0),
                        rng.gen_range(-50.0..50.0),
                    ))
                } else {
                    Transform::from_rotation(Quat::from_euler(
                        EulerRot::XYZ,
                        rng.gen_range(0.0..(2.0 * PI)),
                        rng.gen_range(0.0..(2.0 * PI)),
                        rng.gen_range(0.0..(2.0 * PI)),
                    ))
                }
            })
            .collect()
    }

    /**
     * Re-used test for checking whether a dense, single-element sdf-tree performs nearest neighbor
     * seach correctly on a given primitive after transformations.
     * 
     * Transformations tested are translation, rotation, translation after rotation, and rotation after
     * translation. For any given primitive, the sdf-tree nearest neighbor results should match the
     * computed ground truth values.
     * 
     * See [`test_dense_nn_single_uni()`] and [`test_dense_nn_single_dir()`] for usage.
     * 
     * Generally, this test is meant to check whether [`SdfBuilder::transform()`] and
     * [`SdfBoundingBox::apply_transform()`] are working as intended.
     */
    fn do_dense_nn_single(prim: Box<dyn SdfElement>) {
        let mut rng = thread_rng();
        let tlate_trans = Transform::from_translation(Vec3::new(
            rng.gen_range(-50.0..50.0),
            rng.gen_range(-50.0..50.0),
            rng.gen_range(-50.0..50.0),
        ));
        let tlate_trans_mat = tlate_trans.compute_matrix();
        let tlate_trans_inv = tlate_trans_mat.inverse();
        let rot_trans = Transform::from_rotation(Quat::from_euler(
            EulerRot::XYZ,
            rng.gen_range(0.0..(2.0 * PI)),
            rng.gen_range(0.0..(2.0 * PI)),
            rng.gen_range(0.0..(2.0 * PI)),
        ));
        let rot_trans_mat = rot_trans.compute_matrix();
        let rot_trans_inv = rot_trans_mat.inverse();
        let point = Vec3::new(
            rng.gen_range(-50.0..50.0),
            rng.gen_range(-50.0..50.0),
            rng.gen_range(-50.0..50.0),
        );

        println!("Simple Transform Test Debug:");
        println!("Sample Point: {}", point);
        println!("Translation Matrix: {}", tlate_trans_mat);
        println!("Rotation Matrix: {}", rot_trans_mat);

        let gt_tlate = prim.distance_to((tlate_trans_inv * point.extend(1.0)).truncate());
        let gt_rot = prim.distance_to((rot_trans_inv * point.extend(1.0)).truncate());
        let gt_tlate_rot = prim.distance_to((tlate_trans_inv * rot_trans_inv * point.extend(1.0)).truncate());
        let gt_rot_tlate = prim.distance_to((rot_trans_inv * tlate_trans_inv * point.extend(1.0)).truncate());

        println!("Translation Ground Truth: {}", gt_tlate);
        println!("Rotation Ground Truth: {}", gt_rot);
        println!("Translation->Rotation Ground Truth: {}", gt_tlate_rot);
        println!("Rotation->Translation Ground Truth: {}", gt_rot_tlate);

        let tree_tlate = SdfBuilder::dyn_primitive(prim.clone())
            .transform(tlate_trans)
            .finalize()
            .nearest_neighbor(point)
            .distance;
        let tree_rot = SdfBuilder::dyn_primitive(prim.clone())
            .transform(rot_trans)
            .finalize()
            .nearest_neighbor(point)
            .distance;
        let tree_tlate_rot = SdfBuilder::dyn_primitive(prim.clone())
            .transform(tlate_trans)
            .transform(rot_trans)
            .finalize()
            .nearest_neighbor(point)
            .distance;
        let tree_rot_tlate = SdfBuilder::dyn_primitive(prim)
            .transform(rot_trans)
            .transform(tlate_trans)
            .finalize()
            .nearest_neighbor(point)
            .distance;

        println!("Translation Tree Result: {}", tree_tlate);
        println!("Rotation Tree Result: {}", tree_rot);
        println!("Translation->Rotation Tree Result: {}", tree_tlate_rot);
        println!("Rotation->Translation Tree Result: {}", tree_rot_tlate);

        assert!(approx_eq!(f32, gt_tlate, tree_tlate),
            "Simple Translation Failed!");
        assert!(approx_eq!(f32, gt_rot, tree_rot),
            "Simple Rotation Failed!");
        assert!(approx_eq!(f32, gt_tlate_rot, tree_tlate_rot),
            "Simple Translation->Rotation Failed!"); 
        assert!(approx_eq!(f32, gt_rot_tlate, tree_rot_tlate),
            "Simple Rotation->Translation Failed!"); 
    }

    fn do_dense_nn_chain(prim: Box<dyn SdfElement>) {
        let mut rng = thread_rng();
        let point = Vec3::new(
            rng.gen_range(-50.0..50.0),
            rng.gen_range(-50.0..50.0),
            rng.gen_range(-50.0..50.0),
        );
        let trans_vec = get_random_transforms(1);
        let final_inv = trans_vec.iter()
            .map(|trans| trans.compute_matrix().inverse())
            .fold(Mat4::IDENTITY, |acc, mat| acc * mat);
        let ground_truth = prim.distance_to((final_inv * point.extend(1.0)).truncate());
        let sdf_tree = trans_vec.iter()
            .fold(
                SdfBuilder::dyn_primitive(prim),
                |acc, trans| acc.transform(*trans).operation(SdfUnion {
                    smooth_radius: 0.0,
                })
            )
            .finalize();
        let nn_result = sdf_tree.nearest_neighbor(point).distance;
        assert!(approx_eq!(f32, ground_truth, nn_result),
            "Transform Chain Failed! Ground Truth: {}, NN Result: {}", ground_truth, nn_result);
    }

    /**
     * Run a sanity test for whether TestPrimitive actually works as intended.
     * 
     * This test is never expected to break or fail; it has no reliance on the rest of the library.
     */
    #[test]
    fn sanity_dir_depend_prim() {
        let prim = TestPrimitive {
            scale: 10.0,
            max_dev: 0.1,
        };
        assert!(!float_cmp::approx_eq!(
            f32,
            prim.distance_to(Vec3::new(50.0, 0.0, 0.0)),
            prim.distance_to(Vec3::new(49.8097349, 4.35778713, 0.0))
        ));
    }

    #[test]
    fn test_dense_nn_single_uni() {
        let prim = SdfSphere {
            radius: 1.0,
        };
        do_dense_nn_single(Box::new(prim));
    }

    #[test]
    fn test_dense_nn_single_dir() {
        let prim = TestPrimitive {
            scale: 1.0,
            max_dev: 0.1,
        };
        do_dense_nn_single(Box::new(prim));
    }

    #[test]
    fn test_dense_nn_chain_uni() {
        let prim = SdfSphere {
            radius: 1.0,
        };
        do_dense_nn_chain(Box::new(prim));
    }

    #[test]
    fn test_dense_nn_chain_dir() {
        let prim = TestPrimitive {
            scale: 1.0,
            max_dev: 0.1,
        };
        do_dense_nn_chain(Box::new(prim));
    }
}
