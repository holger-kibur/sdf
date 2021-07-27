use nalgebra::{Matrix4, Vector4, Vector3, Matrix4xX};
use std::cmp::{Ordering};
use bevy::prelude::*;

const VERT_LIST: [Vector4<f32>; 8] = [
    Vector4::new(1.0, 1.0, 1.0, 1.0),
    Vector4::new(-1.0, 1.0, 1.0, 1.0),
    Vector4::new(1.0, -1.0, 1.0, 1.0),
    Vector4::new(-1.0, -1.0, 1.0, 1.0),
    Vector4::new(1.0, 1.0, -1.0, 1.0),
    Vector4::new(-1.0, 1.0, -1.0, 1.0),
    Vector4::new(1.0, -1.0, -1.0, 1.0),
    Vector4::new(-1.0, -1.0, -1.0, 1.0),
];

#[derive(Copy, Clone)]
pub struct SdfBoundingBox {
    matrix: Matrix4<f32>,
}

// I'm so done with this floats-can't-be-compared bullshit; I don't care 
// if it isn't right. 
#[derive(PartialEq)]
struct CmpFloat(f32);

impl Eq for CmpFloat {}

impl PartialOrd for CmpFloat {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        self.0.partial_cmp(&other.0)
    }
}

impl Ord for CmpFloat {
    fn cmp(&self, other: &Self) -> Ordering {
        self.0.partial_cmp(&other.0).unwrap_or(Ordering::Equal)
    }
}

impl SdfBoundingBox {
    #[allow(clippy::map_clone)]
    pub fn merge(sub_boxes: &[Self]) -> Self {
        // Build a matrix from all box vertices
        let mut vert_mat = Matrix4xX::from_iterator(
            sub_boxes.len() * 8,
            sub_boxes.iter()
                .map(|sub_box| VERT_LIST.iter()
                    .map(move |vert| (sub_box.matrix * vert).iter()
                        .map(|x| *x).collect::<Vec<f32>>())
                    .flatten())
                .flatten()
        );
        // Precalculate transpose
        let vert_mat_trans = vert_mat.transpose();
        // Get mean of vertices
        let vert_mean = vert_mat.column_mean();
        // Subtract mean from vert columns
        for mut col in vert_mat.column_iter_mut() {
            col -= vert_mean;
        }
        // Get covariance matrix by right-multiply with transpose
        let covar_mat = (vert_mat * &vert_mat_trans) / ((sub_boxes.len() * 8 - 1) as f32);
        // Get eigenstuff of covariance matrix. Covariance is symmetric so we gucci.
        let eigen_info = covar_mat.symmetric_eigen();
        // Precalculate normalized eigenvector basis
        let eigenbasis_norm = eigen_info.eigenvectors.normalize();
        // Get projections of box verts on normalized eigenvector basis
        let vert_proj_mat = vert_mat_trans * eigenbasis_norm;
        // Get minimums and maximums of verts along eigenvector basis
        let mut box_min = Vector4::zeros();
        let mut box_max = Vector4::zeros();
        for proj_vert in vert_proj_mat.row_iter() {
            // proj_vert is a row vector, so transpose it to compare with box_min, box_max
            let proj_vert_trans = proj_vert.transpose();
            box_min = box_min.inf(&proj_vert_trans);
            box_max = box_max.sup(&proj_vert_trans);
        }
        // Get centroid of new bounding box
        let centroid = (box_max + box_min) / 2.0;
        // Get scale of new bounding box using centroid
        let scale = box_max - centroid;
        // Create new bounding box matrix in eigenbasis of vertices, then change to standard basis
        let new_bbox_mat = (Matrix4::new_translation(&centroid.xyz()) * eigenbasis_norm)
            .append_translation(&vert_mean.xyz())
            .append_nonuniform_scaling(&scale.xyz());
        SdfBoundingBox {
            matrix: new_bbox_mat
        }
    }

    pub fn split(&self, sub_boxes: &[Self]) -> (Vec<usize>, Vec<usize>) {
        let mut inds = (0..sub_boxes.len()).collect::<Vec<usize>>();
        // Get axes of sides of box
        let x_axis = self.matrix.column(0);
        let y_axis = self.matrix.column(1);
        let z_axis = self.matrix.column(2);
        // Get bounding box scale in local coordinates
        let x_scale: f32 = x_axis.iter().map(|x| *x * *x).sum();
        let y_scale: f32 = x_axis.iter().map(|x| *x * *x).sum();
        let z_scale: f32 = x_axis.iter().map(|x| *x * *x).sum();
        // Get normalized axis of bound boxes longest side
        let split_axis = {
            if x_scale >= y_scale && x_scale >= z_scale {
                x_axis.normalize()
            } else if y_scale >= z_scale {
                y_axis.normalize()
            } else {
                z_axis.normalize()
            }
        };
        // Sort by value of projection along splitting axis
        inds.sort_unstable_by_key(|sub_box_ind| CmpFloat(sub_boxes[*sub_box_ind].matrix.column(3).dot(&split_axis)));
        // Split at median
        let left_side = inds.split_off(inds.len() / 2);
        (left_side, inds)
    }

    pub fn from_srt(scale: Vec3, rotation: Vec3, translation: Vec3) -> Self {
        // SRT is Scaling, Rotation, Tramslation e.g. the order in which operations are performed
        // Note that because we work with column vectors, the order is reversed in multiplication.
        SdfBoundingBox {
            matrix: Matrix4::new_translation(&Vector3::new(
                translation.x,
                translation.y,
                translation.z,
            ))
            * Matrix4::from_euler_angles(rotation.x, rotation.y, rotation.z)
            * Matrix4::new_nonuniform_scaling(&Vector3::new(
                scale.x,
                scale.y,
                scale.z
            ))
        }
    }

    pub fn apply_scale(&self, scale: Vec3) -> Self {
        SdfBoundingBox {
            matrix: Matrix4::new_nonuniform_scaling(&Vector3::new(
                scale.x,
                scale.y,
                scale.z
            ))
            * self.matrix
        }
    }

    pub fn apply_bevy_transform(self, trans: Transform) -> Self {
        SdfBoundingBox {
            matrix: Matrix4::from_column_slice(
                &trans.compute_matrix().to_cols_array()
            ) * self.matrix
        }
    }
}