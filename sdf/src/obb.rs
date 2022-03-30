use nalgebra::{
    Matrix4, Vector4, matrix, Matrix, Matrix4xX, Point, U1, U4, Scalar, Const,
    storage::{Storage}
};
use std::{
    convert::TryInto,
    cmp::Ordering,
};
use float_cmp::approx_eq;
use bevy::prelude::*;
use super::component::*;

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

fn vec_bevy_to_nalgebra(bevy_vec: Vec4) -> Vector4<f32> {
    Vector4::from_row_slice(bevy_vec.as_ref())
}

fn mat_bevy_to_nalgebra(bevy_mat: Mat4) -> Matrix4<f32> {
    Matrix4::from_row_slice(bevy_mat.as_ref())
}

fn vec_nalgebra_to_bevy(nalgebra_vec: Vector4<f32>) -> Vec4 {
    // Fully qualified syntax because I am fully qualified to write
    // this kind of code B)
    <Vec4 as From<[f32; 4]>>::from(
        nalgebra_vec.iter()
            .copied()
            .collect::<Vec<f32>>()
            .try_into().unwrap()
    )
}

fn mat_nalgebra_to_bevy(nalgebra_mat: Matrix4<f32>) -> Mat4 {
    Mat4::from_cols_array(
        &nalgebra_mat.iter()
            .copied()
            .collect::<Vec<f32>>()
            .try_into().unwrap()
    )
}

pub fn vec_nalgebra_minmax<T>(nalgebra_vec: Matrix<f32, U4, U1, T>) -> Vector4<f32> where 
    T: Storage<f32, Const<4_usize>>
{
    let mut onehot = [0.0, 0.0, 0.0, 0.0];
    onehot[nalgebra_vec.iamax()] = 1.0;
    Vector4::from_row_slice(&onehot)
}

#[derive(Copy, Clone)]
pub struct SdfBoundingBox {
    pub matrix: Matrix4<f32>,
    pub scale: Vector4<f32>,
    pub full_inverse: Matrix4<f32>,
    pub trans_inverse: Matrix4<f32>,
}

impl SdfBoundingBox {
    pub fn unit() -> Self {
        SdfBoundingBox {
            matrix: Matrix4::identity(),
            scale: matrix![
                1.0;
                1.0;
                1.0;
                0.0;
            ],
            full_inverse: Matrix4::identity(),
            trans_inverse: Matrix4::identity(),
        }
    }

    pub fn zero() -> Self {
        SdfBoundingBox {
            matrix: Matrix4::zeros(),
            scale: Vector4::zeros(),
            full_inverse: matrix![
                f32::INFINITY, 0.0, 0.0, 0.0;
                0.0, f32::INFINITY, 0.0, 0.0;
                0.0, 0.0, f32::INFINITY, 0.0;
                0.0, 0.0, 0.0,           1.0;
            ],
            trans_inverse: Matrix4::identity(),
        }
    }

    #[allow(clippy::map_clone)]
    pub fn merge(sub_boxes: &[Self]) -> Self {
        if sub_boxes.len() == 0 {
            return SdfBoundingBox::zero();
        }
        // Build a matrix from all box vertices
        let mut vert_mat = Matrix4xX::from_iterator(
            sub_boxes.len() * 8,
            sub_boxes.iter()
                .map(|sub_box| VERT_LIST.iter()
                    .map(|vert| (sub_box.matrix * vert).iter()
                        .copied().collect::<Vec<f32>>())
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
        // let sort_transform = Matrix4::from_columns(
        //     eigen_info.eigenvectors.column_iter()
        //         .map(vec_nalgebra_minmax)
        //         // .inspect(|x| println!("{}", x))
        //         .collect::<Vec<Vector4<f32>>>()
        //         .as_slice()
        // );
        let mut eigen_basis = eigen_info.eigenvectors;
        // println!("eigen: {}", eigen_info.eigenvectors);
        eigen_basis.set_column(3, &matrix![0.0; 0.0; 0.0; 1.0;]);
        eigen_basis.set_row(3, &matrix![0.0, 0.0, 0.0, 1.0;]);
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
        let scale_recip = scale.map(|x| 1.0 / x);
        // Create new bounding box matrix in eigenbasis of vertices, then change to standard basis
        let new_bbox_mat = Matrix4::new_translation(&vert_mean.xyz())
            * eigen_basis
            * Matrix4::new_nonuniform_scaling(&scale.xyz()).append_translation(&centroid.xyz());
        let mut scale_iter = scale.iter();
        // println!("eigen full_inverse: {}", (eigen_basis * Matrix4::new_nonuniform_scaling(&scale_recip.xyz())).transpose());
        SdfBoundingBox {
            matrix: new_bbox_mat,
            scale,
            full_inverse: new_bbox_mat
                .try_inverse()
                .unwrap(),
            trans_inverse: (Matrix4::new_translation(&vert_mean.xyz()) * eigen_basis)
                .try_inverse()
                .unwrap(),
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

    pub fn verts(&self) -> Vec<Vec4> {
        VERT_LIST.iter()
            .map(|vert| vec_nalgebra_to_bevy(self.matrix * vert))
            .collect()
    }

    pub fn from_transform(trans: Transform) -> Self {
        Self::unit().apply_transform(trans)
    }

    pub fn apply_transform(self, trans: Transform) -> Self {
        let mut no_scale = trans.clone();
        no_scale.scale = Vec3::new(1.0, 1.0, 1.0);
        let mat = Matrix4::from_column_slice(
            &trans.compute_matrix().to_cols_array()
        ) * self.matrix;
        let full_inv = self.full_inverse * Matrix4::from_column_slice(
            &trans.compute_matrix().inverse().to_cols_array()
        );
        let trans_inv = self.trans_inverse * Matrix4::from_column_slice(
            &no_scale.compute_matrix().inverse().to_cols_array()
        );
        SdfBoundingBox {
            matrix: mat,
            scale: self.scale.component_mul(&vec_bevy_to_nalgebra(trans.scale.extend(0.0))),
            full_inverse: full_inv,
            trans_inverse: trans_inv,
        }
    }

    pub fn get_transform(&self) -> Transform {
        Transform::from_matrix(mat_nalgebra_to_bevy(self.matrix))
    }

    pub fn in_box_basis(&self, point: Vec4) -> Vec4 {
        vec_nalgebra_to_bevy(
            self.full_inverse * vec_bevy_to_nalgebra(point)
        )
    }

    pub fn in_box_trans_basis(&self, point: Vec4) -> Vec4 {
        vec_nalgebra_to_bevy(
            self.trans_inverse * vec_bevy_to_nalgebra(point)
        )
    }

    pub fn in_parent_basis(&self, point: Vec4) -> Vec4 {
        vec_nalgebra_to_bevy(
            self.matrix * vec_bevy_to_nalgebra(point)
        )
    }

    pub fn mat_in_box_basis(&self, mat: Mat4) -> Mat4 {
        mat_nalgebra_to_bevy(
            self.full_inverse * mat_bevy_to_nalgebra(mat)
        )
    }

    pub fn mat_in_box_trans_basis(&self, mat: Mat4) -> Mat4 {
        mat_nalgebra_to_bevy(
            self.trans_inverse * mat_bevy_to_nalgebra(mat)
        )
    }

    pub fn box_in_box_trans_basis(&self, other: &SdfBoundingBox) -> SdfBoundingBox {
        SdfBoundingBox {
            matrix: self.trans_inverse * other.matrix,
            scale: other.scale,
            full_inverse: other.full_inverse * self.trans_inverse,
            trans_inverse: other.trans_inverse * self.trans_inverse,
        }
    }

    pub fn is_zero(&self) -> bool {
        self.get_transform().scale.as_ref().iter()
            .all(|comp| approx_eq!(f32, *comp, 0.0, ulps = 2))
    }

    pub fn distance_to(&self, point: Vec3) -> f32 {
        let trans = self.in_box_basis(point.extend(1.0));
        // println!("scale: {}", self.scale);
        let q_local = (trans.abs() - Vec4::splat(1.0)) * vec_nalgebra_to_bevy(self.scale);
        // println!("q_local: {}", q_local);
        q_local.max(Vec4::ZERO).length() + q_local.y.max(q_local.z).max(q_local.x).min(0.0)
    }

    pub fn max_distance(&self, point: Vec3) -> f32 {
        let nalgebra_point = Point::from_slice(point.extend(1.0).as_ref()).coords;
        VERT_LIST.iter()
            .map(|vert| CmpFloat(((self.matrix * vert) - nalgebra_point).magnitude()))
            .max().unwrap().0
    }

    pub fn centroid(&self) -> Vec3 {
        self.get_transform().translation
    }

    pub fn contains(&self, point: Vec3) -> bool {
        self.full_inverse.transform_point(&Point::from_slice(point.as_ref())).coords.amax() <= 1.0
    }

    pub fn get_bbox_block(&self) -> SdfBoundingBoxBlock {
        SdfBoundingBoxBlock {
            matrix: mat_nalgebra_to_bevy(self.matrix),
            scale: vec_nalgebra_to_bevy(self.scale),
            full_inverse: mat_nalgebra_to_bevy(self.full_inverse),
            trans_inverse: mat_nalgebra_to_bevy(self.trans_inverse),
        }
    }
}