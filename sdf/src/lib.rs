pub mod builder;
pub mod component;
pub mod dense_node;
pub mod elements;
pub mod expanded_node;
pub mod faux_shader;
pub mod obb;

// #[cfg(test)]
// pub mod tests {
//     use rand::prelude::*;
//     use std::f32::consts::{PI, FRAC_PI_2, SQRT_2};
//     use bevy::prelude::*;
//     use crate::{
//         builder::*,
//         dense_node::*,
//         elements::*,
//         expanded_node::*,
//         obb::*,
//         component::*,
//     };
//     use float_cmp::approx_eq;

//     #[derive(Debug)]
//     struct TestPrimitive {
//         pub scale: f32,
//         pub max_dev: f32,
//     }

//     impl SdfElement for TestPrimitive {
//         fn get_info(&self) -> SdfElementInfo {
//             SdfElementInfo::primitive_info(1)
//         }

//         fn get_bbox(&self, _slots_bboxes: &[SdfBoundingBox]) -> SdfBoundingBox {
//             SdfBoundingBox::from_transform(Transform::from_scale(Vec3::splat(self.scale)))
//         }

//         fn clone(&self) -> Box<dyn SdfElement> {
//             Box::new(TestPrimitive {
//                 scale: self.scale,
//                 max_dev: self.max_dev,
//             })
//         }

//         fn distance_to(&self, point: Vec3) -> f32 {
//             let sectors = 360.0 / (self.max_dev / SQRT_2);
//             let azimuth = f32::atan2(point.z, point.x) + PI;
//             let zenith = f32::atan(point.y / f32::sqrt(point.x * point.x + point.y * point.y)) + FRAC_PI_2;
//             let azimuth_quant = (azimuth * sectors).trunc();
//             let zenith_quant = (zenith * sectors / 2.0).trunc();
//             let paired = zenith_quant * sectors + azimuth_quant;
//             point.length() - paired / (sectors * sectors / 2.0) * self.scale
//         }

//         fn get_dt_specific_block(&self) -> SdfOpSpecificBlock {
//             let mut ret = SdfOpSpecificBlock::ZERO;
//             ret.floats[0] = self.scale;
//             ret.floats[1] = self.max_dev;
//             ret
//         }

//         fn expand(&self, this_node: &SdfNode) -> ExpandedSdfNode {
//             ExpandedSdfNode::primitive(this_node.bbox, self.clone())
//         }
//     }

//     fn get_random_transforms(count: u32) -> Vec<Transform> {
//         let mut rng = thread_rng();
//         (0..count)
//             .map(|_| {
//                 if rng.gen::<bool>() {
//                     Transform::from_translation(Vec3::new(
//                         rng.gen_range(-50.0..50.0),
//                         rng.gen_range(-50.0..50.0),
//                         rng.gen_range(-50.0..50.0),
//                     ))
//                 } else {
//                     Transform::from_rotation(Quat::from_euler(
//                         EulerRot::XYZ,
//                         rng.gen_range(0.0..(2.0 * PI)),
//                         rng.gen_range(0.0..(2.0 * PI)),
//                         rng.gen_range(0.0..(2.0 * PI)),
//                     ))
//                 }
//             })
//             .collect()
//     }

//     /**
//      * Re-used test for checking whether a dense, single-element sdf-tree performs nearest neighbor
//      * seach correctly on a given primitive after transformations.
//      *
//      * Transformations tested are translation, rotation, translation after rotation, and rotation after
//      * translation. For any given primitive, the sdf-tree nearest neighbor results should match the
//      * computed ground truth values.
//      *
//      * See [`test_dense_nn_single_uni()`] and [`test_dense_nn_single_dir()`] for usage.
//      *
//      * Generally, this test is meant to check whether [`BuildingSdfNode::transform()`] and
//      * [`SdfBoundingBox::apply_transform()`] are working as intended.
//      */
//     fn do_dense_nn_single(prim: Box<dyn SdfElement>) {
//         let mut rng = thread_rng();
//         let tlate_trans = Transform::from_translation(Vec3::new(
//             rng.gen_range(-50.0..50.0),
//             rng.gen_range(-50.0..50.0),
//             rng.gen_range(-50.0..50.0),
//         ));
//         let tlate_trans_mat = tlate_trans.compute_matrix();
//         let tlate_trans_inv = tlate_trans_mat.inverse();
//         let rot_trans = Transform::from_rotation(Quat::from_euler(
//             EulerRot::XYZ,
//             rng.gen_range(0.0..(2.0 * PI)),
//             rng.gen_range(0.0..(2.0 * PI)),
//             rng.gen_range(0.0..(2.0 * PI)),
//         ));
//         let rot_trans_mat = rot_trans.compute_matrix();
//         let rot_trans_inv = rot_trans_mat.inverse();
//         let point = Vec3::new(
//             rng.gen_range(-50.0..50.0),
//             rng.gen_range(-50.0..50.0),
//             rng.gen_range(-50.0..50.0),
//         );

//         println!("Simple Transform Test Debug:");
//         println!("Sample Point: {}", point);
//         println!("Translation Matrix: {}", tlate_trans_mat);
//         println!("Rotation Matrix: {}", rot_trans_mat);

//         let gt_tlate = prim.distance_to((tlate_trans_inv * point.extend(1.0)).truncate());
//         let gt_rot = prim.distance_to((rot_trans_inv * point.extend(1.0)).truncate());
//         let gt_tlate_rot = prim.distance_to((tlate_trans_inv * rot_trans_inv * point.extend(1.0)).truncate());
//         let gt_rot_tlate = prim.distance_to((rot_trans_inv * tlate_trans_inv * point.extend(1.0)).truncate());

//         println!("Translation Ground Truth: {}", gt_tlate);
//         println!("Rotation Ground Truth: {}", gt_rot);
//         println!("Translation->Rotation Ground Truth: {}", gt_tlate_rot);
//         println!("Rotation->Translation Ground Truth: {}", gt_rot_tlate);

//         let tree_tlate = BuildingSdfNode::dyn_primitive(prim.clone())
//             .transform(tlate_trans)
//             .finalize()
//             .nearest_neighbor(point)
//             .distance;
//         let tree_rot = BuildingSdfNode::dyn_primitive(prim.clone())
//             .transform(rot_trans)
//             .finalize()
//             .nearest_neighbor(point)
//             .distance;
//         let tree_tlate_rot = BuildingSdfNode::dyn_primitive(prim.clone())
//             .transform(tlate_trans)
//             .transform(rot_trans)
//             .finalize()
//             .nearest_neighbor(point)
//             .distance;
//         let tree_rot_tlate = BuildingSdfNode::dyn_primitive(prim)
//             .transform(rot_trans)
//             .transform(tlate_trans)
//             .finalize()
//             .nearest_neighbor(point)
//             .distance;

//         println!("Translation Tree Result: {}", tree_tlate);
//         println!("Rotation Tree Result: {}", tree_rot);
//         println!("Translation->Rotation Tree Result: {}", tree_tlate_rot);
//         println!("Rotation->Translation Tree Result: {}", tree_rot_tlate);

//         assert!(approx_eq!(f32, gt_tlate, tree_tlate),
//             "Simple Translation Failed!");
//         assert!(approx_eq!(f32, gt_rot, tree_rot),
//             "Simple Rotation Failed!");
//         assert!(approx_eq!(f32, gt_tlate_rot, tree_tlate_rot),
//             "Simple Translation->Rotation Failed!");
//         assert!(approx_eq!(f32, gt_rot_tlate, tree_rot_tlate),
//             "Simple Rotation->Translation Failed!");
//     }

//     fn do_dense_nn_chain(prim: Box<dyn SdfElement>) {
//         let mut rng = thread_rng();
//         let point = Vec3::new(
//             rng.gen_range(-50.0..50.0),
//             rng.gen_range(-50.0..50.0),
//             rng.gen_range(-50.0..50.0),
//         );
//         let trans_vec = get_random_transforms(1);
//         println!("{}", trans_vec[0].translation);
//         let final_inv = trans_vec.iter()
//             .map(|trans| trans.compute_matrix().inverse())
//             .fold(Mat4::IDENTITY, |acc, mat| acc * mat);
//         let ground_truth = prim.distance_to((final_inv * point.extend(1.0)).truncate());
//         let sdf_tree = trans_vec.iter()
//             .fold(
//                 BuildingSdfNode::dyn_primitive(prim),
//                 |acc, trans| acc.transform(*trans).operation(SdfUnion {
//                     smooth_radius: 0.0,
//                 })
//             )
//             .finalize();
//         let nn_result = sdf_tree.nearest_neighbor(point).distance;
//         assert!(approx_eq!(f32, ground_truth, nn_result),
//             "Transform Chain Failed! Ground Truth: {}, NN Result: {}", ground_truth, nn_result);
//     }

//     /**
//      * Run a sanity test for whether TestPrimitive actually works as intended.
//      *
//      * This test is never expected to break or fail; it has no reliance on the rest of the library.
//      */
//     #[test]
//     fn sanity_dir_depend_prim() {
//         let prim = TestPrimitive {
//             scale: 10.0,
//             max_dev: 0.1,
//         };
//         assert!(!float_cmp::approx_eq!(
//             f32,
//             prim.distance_to(Vec3::new(50.0, 0.0, 0.0)),
//             prim.distance_to(Vec3::new(49.8097349, 4.35778713, 0.0))
//         ));
//     }

//     #[test]
//     fn test_dense_nn_single_uni() {
//         let prim = SdfSphere {
//             radius: 1.0,
//         };
//         do_dense_nn_single(Box::new(prim));
//     }

//     #[test]
//     fn test_dense_nn_single_dir() {
//         let prim = TestPrimitive {
//             scale: 1.0,
//             max_dev: 0.1,
//         };
//         do_dense_nn_single(Box::new(prim));
//     }

//     #[test]
//     fn test_dense_nn_chain_uni() {
//         let prim = SdfSphere {
//             radius: 1.0,
//         };
//         let x = Box::new(prim);
//         do_dense_nn_chain(x);
//     }

//     #[test]
//     fn test_dense_nn_chain_dir() {
//         let prim = TestPrimitive {
//             scale: 1.0,
//             max_dev: 0.1,
//         };
//         do_dense_nn_chain(Box::new(prim));
//     }
// }
