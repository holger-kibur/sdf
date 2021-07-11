use super::sdf::*;
use std::{
    cmp::Ordering,
    rc::{Rc, Weak},
};
use bevy::prelude::*;

const NODE_SIZE: usize = 10;
const NODE_SIZE_DIV_2: usize = NODE_SIZE / 2;

#[derive(Clone)]
struct SdfObjectHandle {
    handle: Handle<SdfObject>,
    bounding_box: SdfBb,
}

struct RTreeNode {
    bounding_box: SdfBb,
    child_nodes: Vec<BoundedNode>,
}

struct RTreeLeaf {
    bounding_box: SdfBb,
    objects: Vec<SdfObjectHandle>,
}

impl RTreeNode {
    pub fn get_subslice_bb(&self, indices: &[usize]) -> SdfBb {
        SdfBb::merge_slice(
            indices.iter()
                .map(|i| self.child_nodes[*i].get_bounding_box())
                .collect::<Vec<&SdfBb>>()
                .as_slice()
        )
    }

    pub fn insert(&mut self, object: SdfObjectHandle) {
        self.bounding_box = self.bounding_box.get_merged(&object.bounding_box);
        let optimal_child = self.child_nodes.iter_mut().min_by_key(
            |node| node.get_bounding_box().get_merged(&object.bounding_box).get_surface_area()
        ).unwrap();
        optimal_child.insert(object);
        if optimal_child.len() > NODE_SIZE {
            let new_nodes = optimal_child.split();
            self.child_nodes.push(new_nodes.0);
            self.child_nodes.push(new_nodes.1);
        }
    }
}

impl RTreeLeaf {
    pub fn get_subslice_bb(&self, indices: &[usize]) -> SdfBb {
        SdfBb::merge_slice(
            indices.iter()
                .map(|i| &self.objects[*i].bounding_box)
                .collect::<Vec<&SdfBb>>()
                .as_slice()
        )
    }

    pub fn insert(&mut self, object: SdfObjectHandle) {
        self.bounding_box = self.bounding_box.get_merged(&object.bounding_box);
        self.objects.push(object);
    }
}

enum BoundedNode {
    Parent(RTreeNode),
    Leaf(RTreeLeaf)
}

impl BoundedNode {
    pub fn get_bounding_box(&self) -> &SdfBb {
        match self {
            BoundedNode::Parent(node) => &node.bounding_box,
            BoundedNode::Leaf(node) => &node.bounding_box
        }
    }

    pub fn len(&self) -> usize {
        match self {
            BoundedNode::Parent(node) => node.child_nodes.len(),
            BoundedNode::Leaf(node) => node.objects.len()
        }
    }

    fn get_subslice_bb(&self, indices: &[usize]) -> SdfBb {
        match self {
            BoundedNode::Parent(node) => node.get_subslice_bb(indices),
            BoundedNode::Leaf(node) => node.get_subslice_bb(indices)
        }
    }

    pub fn split(self) -> (BoundedNode, BoundedNode) {
        let bbox_centers: Vec<Vec3> = {
            match self {
                BoundedNode::Parent(node) => {
                    node.child_nodes.iter().map(|node| node.get_bounding_box().get_center()).collect()
                },
                BoundedNode::Leaf(node) => {
                    node.objects.iter().map(|node| node.bounding_box.get_center()).collect()
                }
            }
        };
        let x_sorted: Vec<usize> = (0..bbox_centers.len()).collect();
        let y_sorted: Vec<usize> = (0..bbox_centers.len()).collect();
        let z_sorted: Vec<usize> = (0..bbox_centers.len()).collect();

        x_sorted.sort_unstable_by(|ai, bi| bbox_centers[*ai].x.partial_cmp(&bbox_centers[*bi].x).unwrap());
        y_sorted.sort_unstable_by(|ai, bi| bbox_centers[*ai].y.partial_cmp(&bbox_centers[*bi].y).unwrap());
        z_sorted.sort_unstable_by(|ai, bi| bbox_centers[*ai].z.partial_cmp(&bbox_centers[*bi].z).unwrap());

        let x_pair = (self.get_subslice_bb(&x_sorted[..NODE_SIZE_DIV_2]), self.get_subslice_bb(&x_sorted[NODE_SIZE_DIV_2..]));
        let y_pair = (self.get_subslice_bb(&x_sorted[..NODE_SIZE_DIV_2]), self.get_subslice_bb(&x_sorted[NODE_SIZE_DIV_2..]));
        let z_pair = (self.get_subslice_bb(&x_sorted[..NODE_SIZE_DIV_2]), self.get_subslice_bb(&x_sorted[NODE_SIZE_DIV_2..]));

        let x_sa = *x_pair.0.get_surface_area() + *x_pair.1.get_surface_area();
        let y_sa = *y_pair.0.get_surface_area() + *y_pair.1.get_surface_area();
        let z_sa = *z_pair.0.get_surface_area() + *z_pair.1.get_surface_area();

        let (a_inds, a_bb, b_bb) = {
            if x_sa < y_sa && x_sa < z_sa {
                (&x_sorted[..NODE_SIZE_DIV_2], x_pair.0, x_pair.1)
            } else if y_sa < z_sa {
                (&y_sorted[..NODE_SIZE_DIV_2], y_pair.0, y_pair.1)
            } else {
                (&z_sorted[..NODE_SIZE_DIV_2], z_pair.0, z_pair.1)
            }
        };

        a_inds.sort_unstable();
        a_inds.reverse();

        match self {
            BoundedNode::Parent(node) => {
                (
                    BoundedNode::Parent(RTreeNode{
                        bounding_box: a_bb,
                        child_nodes: a_inds.iter().map(|rem_ind| node.child_nodes.remove(*rem_ind)).collect(),
                    }),
                    BoundedNode::Parent(RTreeNode{
                        bounding_box: b_bb,
                        child_nodes: node.child_nodes
                    })
                )
            },
            BoundedNode::Leaf(node) => {
                (
                    BoundedNode::Leaf(RTreeLeaf{
                        bounding_box: a_bb,
                        objects: a_inds.iter().map(|rem_ind| node.objects.remove(*rem_ind)).collect(),
                    }),
                    BoundedNode::Leaf(RTreeLeaf{
                        bounding_box: b_bb,
                        objects: node.objects
                    })
                )
            }
        }
    }

    pub fn insert(&mut self, object: SdfObjectHandle) {
        match self {
            BoundedNode::Parent(node) => node.insert(object),
            BoundedNode::Leaf(node) => node.insert(object)
        }
    }
}

pub struct SdfRTree {
    root: BoundedNode,
}

impl SdfRTree {

    fn bulk_load_recurse(objects: &mut [SdfObjectHandle]) -> BoundedNode {
        if objects.len() <= NODE_SIZE {
            BoundedNode::Leaf(RTreeLeaf {
                bounding_box: SdfBb::merge_slice(
                    objects.iter().map(|handle| &handle.bounding_box).collect::<Vec<&SdfBb>>().as_slice()
                ),
                objects: objects.to_owned()
            })
        } else {
            let height = (objects.len() as f32).log(NODE_SIZE as f32).ceil();
            let sub_size = (NODE_SIZE as f32).powf(height - 1.0);
            let parts_per_dim = ((objects.len() as f32) / sub_size).ceil().cbrt().floor().to_int_unchecked();
            let 
        }
    }

    pub fn bulk_load(objects: Vec<SdfObjectHandle>) {
        
    }

    pub fn insert(&mut self, object: SdfObjectHandle) {
        self.root.insert(object);
        if self.root.len() > NODE_SIZE {
            let split_root = self.root.split();
            self.root = BoundedNode::Parent(RTreeNode {
                bounding_box: split_root.0.get_bounding_box().get_merged(split_root.1.get_bounding_box()),
                child_nodes: vec![split_root.0, split_root.1],
            });
        }
    }
}