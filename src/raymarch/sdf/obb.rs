use nalgebra::{Matrix4, Vector4, matrix, Matrix4xX, Point};
use std::{
    convert::TryInto,
    cmp::Ordering,
};
use float_cmp::approx_eq;
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

// I'm so done with this floats-can't-be-compared bullshit; I don't care 
// if it isn't right. 
#[derive(PartialEq)]
pub struct CmpFloat(pub f32);

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

#[derive(Copy, Clone)]
pub struct SdfBoundingBox {
    matrix: Matrix4<f32>,
    inverse: Matrix4<f32>,
}

impl SdfBoundingBox {
    pub fn unit() -> Self {
        SdfBoundingBox {
            matrix: Matrix4::identity(),
            inverse: Matrix4::identity(),
        }
    }

    pub fn zero() -> Self {
        SdfBoundingBox {
            matrix: Matrix4::zeros(),
            inverse: matrix![
                f32::INFINITY, 0.0, 0.0, 0.0;
                0.0, f32::INFINITY, 0.0, 0.0;
                0.0, 0.0, f32::INFINITY, 0.0;
                0.0, 0.0, 0.0,           1.0;
            ],
        }
    }

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
        // Get mean of vertices
        let vert_mean = vert_mat.column_mean();
        // Subtract mean from vert columns
        for mut col in vert_mat.column_iter_mut() {
            col -= vert_mean;
        }
        // Get transpose
        let vert_mat_mean_trans = vert_mat.transpose();
        // Get covariance matrix by right-multiply with transpose
        let covar_mat = (vert_mat * &vert_mat_mean_trans) / ((sub_boxes.len() * 8 - 1) as f32);
        // Get eigenstuff of covariance matrix. Covariance is symmetric so we gucci.
        let eigen_info = covar_mat.symmetric_eigen();
        // For some reason, symmetric eigen loves to make the W-vector negative sometimes, so we have to set it 
        let mut eigen_basis = eigen_info.eigenvectors;
        eigen_basis.set_column(3, &matrix![0.0; 0.0; 0.0; 1.0;]);
        // Get projections of box verts on normalized eigenvector basis
        let vert_proj_mat = vert_mat_mean_trans * eigen_basis;
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
        let new_bbox_mat = Matrix4::new_translation(&vert_mean.xyz())
            * eigen_basis
            * Matrix4::new_nonuniform_scaling(&scale.xyz()).append_translation(&centroid.xyz());
        SdfBoundingBox {
            matrix: new_bbox_mat,
            inverse: new_bbox_mat.try_inverse().unwrap(),
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
        let y_scale: f32 = y_axis.iter().map(|x| *x * *x).sum();
        let z_scale: f32 = z_axis.iter().map(|x| *x * *x).sum();
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

    pub fn from_transform(trans: Transform) -> Self {
        Self::unit().apply_transform(trans)
    }

    pub fn apply_transform(self, trans: Transform) -> Self {
        let mat = Matrix4::from_column_slice(
            &trans.compute_matrix().to_cols_array()
        ) * self.matrix;
        let inv = Matrix4::from_column_slice(
            &Transform {
                translation: -trans.translation,
                rotation: trans.rotation.inverse(),
                scale: trans.scale.recip(),
            }.compute_matrix().to_cols_array()
        ) * self.inverse;
        SdfBoundingBox {
            matrix: mat,
            inverse: inv,
        }
    }

    pub fn get_transform(&self) -> Transform {
        Transform::from_matrix(Mat4::from_cols_array(
            &self.matrix.iter()
                .copied()
                .collect::<Vec<f32>>()
                .try_into().unwrap()
        ))
    }

    pub fn in_box_basis(&self, point: Vec3) -> Vec3 {
        let nalgebra_point = self.inverse.transform_point(&Point::from_slice(point.as_ref())).coords;
        let mut vec_iter = nalgebra_point.into_iter();
        Vec3::new(
            *vec_iter.next().unwrap(),
            *vec_iter.next().unwrap(),
            *vec_iter.next().unwrap(),
        )
    }

    pub fn is_zero(&self) -> bool {
        self.get_transform().scale.as_ref().iter()
            .all(|comp| approx_eq!(f32, *comp, 0.0, ulps = 2))
    }

    pub fn distance_to(&self, point: Vec3) -> f32 {
        let trans = self.in_box_basis(point);
        let q = trans.abs() - Vec3::splat(1.0);
        q.max(Vec3::ZERO).length() + q.y.max(q.z).max(q.x).min(0.0)
    }

    pub fn centroid(&self) -> Vec3 {
        self.get_transform().translation
    }

    pub fn contains(&self, point: Vec3) -> bool {
        self.inverse.transform_point(&Point::from_slice(point.as_ref())).coords.amax() <= 1.0
    }
}