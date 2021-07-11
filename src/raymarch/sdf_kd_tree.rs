use std::{
    cmp::Ordering,
    iter
};
use bevy::{
    prelude::*,
    core::{Byteable, AsBytes, Bytes},
    render::{
        renderer::{RenderResources, RenderResource, RenderResourceIterator, RenderResourceHints},
    },
};

pub trait Compare3D {
    fn xyz_lt(a: &Self, b: &Self) -> Ordering;
    fn yzx_lt(a: &Self, b: &Self) -> Ordering;
    fn zxy_lt(a: &Self, b: &Self) -> Ordering;
    fn get_null() -> Self;
}

#[derive(Debug)]
pub struct KdTree3D<T: Compare3D + PartialEq> {
    element_buffer: Vec<T>,
    order_buffer: Vec<usize>,
    num_empty: usize,
}

pub trait BkdTree3DObject: Compare3D + PartialEq + Clone {}

impl<T: BkdTree3DObject> KdTree3D<T> {
    const CMP_FUNCS: [fn(&T, &T) -> Ordering; 3] = [T::xyz_lt, T::yzx_lt, T::zxy_lt];

    fn build(mut in_list: Vec<T>) -> Self {
        let mut len = in_list.len();

        // Pad length to make it proper size for algorithm
        let num_empty = {
            if !(len + 1).is_power_of_two() {
                len = len.next_power_of_two() - 1;
                in_list.append(vec![T::get_null(); len - in_list.len()]);
                len - in_list.len()
            } else {
                0
            }
        };

        // xyz index, yzx index, zxy index
        let mut index_buffer: Vec<usize> = (0_usize..(len * 3)).map(|v| v % len).collect();
        let mut temp_buffer = vec![0_usize; len];
        let mut dest_buffer: Vec<T> = Vec::with_capacity(len);
        let max_level: usize = (len.count_ones() as usize) - 2; // Floor of log2 len minus two
        let mut level: usize = 0;

        // Initial index buffer sorts
        for i in 0..3 {
            index_buffer[(i * len)..((i + 1) * len)].sort_unstable_by(|&a, &b| {
                Self::CMP_FUNCS[i](&in_list[a], &in_list[b])
            });
        }

        while level < max_level {
            let ref_index: usize = level % 3;
            let range_size: usize = len / (1 << level);
            let ref_offset = ref_index * len;
            for part_index in [(ref_index + 1) % 3, (ref_index + 2) % 3].iter() {
                let mut range_offset = 0;
                let part_offset = (*part_index as usize) * len;
                while range_offset < len {
                    let part_offset = (*part_index as usize) * len;
                    let pivot_index = index_buffer[ref_offset + range_offset + range_size / 2];
                    let pivot_elm = &in_list[pivot_index];
                    let mut less_idx = range_offset;
                    let mut more_idx = range_offset + range_size / 2 + 1;
                    for item_index in index_buffer[(part_offset + range_offset)..(part_offset + range_offset + range_size)].iter(){
                        let item = &in_list[*item_index];
                        match Self::CMP_FUNCS[ref_index](item, pivot_elm) {
                            Ordering::Less => {
                                temp_buffer[less_idx] = *item_index;
                                less_idx += 1;
                            },
                            Ordering::Greater => {
                                temp_buffer[more_idx] = *item_index;
                                more_idx += 1;
                            },
                            Ordering::Equal => {
                                temp_buffer[range_offset + range_size / 2] = *item_index;
                            }
                        }
                    }
                    range_offset += range_size + 1
                }
                for i in 0..len {
                    index_buffer[part_offset + i] = temp_buffer[i];
                }
            }
            level += 1;
        }

        let correct_index_offset = len * (level % 3);
        Self {
            element_buffer: in_list,
            order_buffer: index_buffer.drain(correct_index_offset..(correct_index_offset + len)).collect(),
            num_empty,
        }
    }

    fn null_index(&mut self, index: usize) {
        self.element_buffer[self.order_buffer[index]] = T::get_null();
        self.num_empty += 1;
    }

    pub fn try_remove(&mut self, elm: &T) -> Result<(), ()> {
        let mut level = 0;
        let mut jump_amount = (self.element_buffer.len() + 1) / 4;
        let mut index = jump_amount * 2 - 1;
        while jump_amount > 0 {
            let indexed_elm = &self.element_buffer[self.order_buffer[index]];
            index = match Self::CMP_FUNCS[(level as usize) % 3](elm, indexed_elm) {
                Ordering::Less => index - jump_amount,
                Ordering::Greater => index + jump_amount,
                Ordering::Equal => {
                    self.null_index(index);
                    return Ok(());
                }
            };
            level += 1;
            jump_amount /= 2;
        }
        if let Ordering::Equal = Self::CMP_FUNCS[(level as usize) % 3](
            elm,
            &self.element_buffer[self.order_buffer[index]]
        ) {
            self.null_index(index);
            Ok(())
        } else {
            Err(())
        }
    }

    pub fn len(&self) -> usize {
        self.element_buffer.len() - self.num_empty
    }
}

impl<T: BkdTree3DObject> Into<Vec<T>> for KdTree3D<T> {
    fn into(self) -> Vec<T> {
        let null = T::get_null();
        self.element_buffer.retain(|x| *x != null);
        self.element_buffer
    }
}

struct BkdTree3D<T: BkdTree3DObject> {
    stage_size: usize,
    staging_buffer: Vec<T>,
    sub_trees: Vec<Option<KdTree3D<T>>>,
}

impl<T: BkdTree3DObject> BkdTree3D<T> {
    pub fn new(staging_size: usize) -> Self {
        Self {
            stage_size: staging_size,
            staging_buffer: Vec::with_capacity(staging_size),
            sub_trees: Vec::new(),
        }
    }

    fn get_next_empty_tree(&mut self) -> usize {
        for (i, sub_tree) in self.sub_trees.iter().enumerate() {
            if sub_tree.is_none() {
                return i;
            }
        }
        self.sub_trees.push(None);
        self.sub_trees.len() - 1
    }

    fn collect_trees(&mut self, low_tree: usize, high_tree: usize) -> Vec<T> {
        if high_tree > 0 {
            // There must be a full tree at the previous index
            let collect_buf: Vec<T> = self.sub_trees[high_tree].take().unwrap().into();
            collect_buf.reserve_exact(collect_buf.len() + 1);
            for i in low_tree..high_tree {
                collect_buf.append(&mut self.sub_trees[i].take().unwrap().into());
            }
            return collect_buf;
        } else {
            // There are no filled trees
            return Vec::with_capacity(self.stage_size + 1);
        }
    }

    pub fn insert(&mut self, element_handle: &T) {
        if self.staging_buffer.len() == self.stage_size {
            let next_tree_index = self.get_next_empty_tree();
            let collect_buffer = self.collect_trees(0, next_tree_index);
            collect_buffer.append(&mut self.staging_buffer);
            collect_buffer.push(element_handle.clone());
            self.sub_trees[next_tree_index] = Some(KdTree3D::build(collect_buffer));
        } else {
            self.staging_buffer.push(element_handle.clone());
        }
    }

    fn try_tree_reduce(&mut self, frag_tree_index: usize) -> Result<(), ()> {
        if frag_tree_index == 0 {
            // Can't reduce into staging buffer, it's only for staging
            return Err(());
        }
        let prev_tree_len = {
            if let Some(prev_tree) = self.sub_trees[frag_tree_index - 1] {
                prev_tree.len()
            } else {
                0
            }
        };
        let this_tree_len = self.sub_trees[frag_tree_index].unwrap().len();
        if prev_tree_len + this_tree_len < self.stage_size * (1 << (frag_tree_index - 1)) {
            self.sub_trees[frag_tree_index - 1] = Some(KdTree3D::build(
                self.collect_trees(frag_tree_index - 1, frag_tree_index)
            ));
            Ok(())
        } else {
            Err(())
        }
    }

    pub fn remove(&mut self, element: &T) -> Result<(), ()> {
        for stage_index in 0..self.staging_buffer.len() {
            if self.staging_buffer[stage_index] == *element {
                self.staging_buffer.remove(stage_index);
                return Ok(());
            }
        }
        for (i, sub_tree_opt) in self.sub_trees.iter_mut().enumerate() {
            if let Some(mut sub_tree) = sub_tree_opt {
                if sub_tree.try_remove(element).is_ok() {
                    self.try_tree_reduce(i);
                    return Ok(());
                }
            }
        }
        Err(())
    }
}

impl<T: BkdTree3DObject + Byteable> BkdTree3D<T> {
    pub fn create_resource_buffers(&self) -> BkdTree3DTreeBuffers<T> {
        let tree_descriptors: Vec<Bkd3DSubTreeDescriptor> = Vec::new();
        let element_buffer: Vec<T> = Vec::new();
        let order_buffer: Vec<usize> = Vec::new();
        let staging_tree = KdTree3D::build(self.staging_buffer.clone());
        let elm_length = T::get_null().byte_len();
        let mut length_accum = 0_usize;
        
        for sub_tree in iter::once(staging_tree).chain(
            self.sub_trees.iter()
                .filter(|tree_opt| tree_opt.is_some())
                .map(|tree_some| tree_some.unwrap())
        ) {
            tree_descriptors.push(Bkd3DSubTreeDescriptor {
                tree_offset: length_accum,
                tree_size: sub_tree.element_buffer.len(),
            });
            element_buffer.extend(sub_tree.element_buffer);
            order_buffer.extend(sub_tree.order_buffer.iter().map(|index| *index + length_accum));
            length_accum += sub_tree.element_buffer.len();
        }

        BkdTree3DTreeBuffers {
            tree_descriptors,
            element_buffer,
            order_buffer
        }
    }
}

#[derive(Clone, Copy)]
#[repr(C)]
struct Bkd3DSubTreeDescriptor {
    tree_offset: usize,
    tree_size: usize,
}

unsafe impl Byteable for Bkd3DSubTreeDescriptor {}

struct BkdTree3DTreeBuffers<T: BkdTree3DObject + Byteable> {
    tree_descriptors: Vec<Bkd3DSubTreeDescriptor>,
    element_buffer: Vec<T>,
    order_buffer: Vec<usize>,
}

impl<T: BkdTree3DObject + Byteable + Send + Sync + 'static> RenderResources for BkdTree3DTreeBuffers<T> {
    fn render_resources_len(&self) -> usize {
        let usize_byte_len = 0_usize.byte_len();
        4 + // num trees u32
        self.tree_descriptors.len() * 2 * usize_byte_len + // tree descriptors
        self.element_buffer.len() * T::get_null().byte_len() + // elements
        self.order_buffer.len() * usize_byte_len // indexes
    }

    fn get_render_resource(&self, index: usize) -> Option<&dyn RenderResource> {
        match index {
            0 => Some(&(self.tree_descriptors.len() as u32)),
            1 => Some(&self.tree_descriptors),
            2 => Some(&self.element_buffer),
            3 => Some(&self.order_buffer),
            _ => None
        }
    }

    fn get_render_resource_name(&self, index: usize) -> Option<&str> {
        match index {
            0 => Some("num_trees"),
            1 => Some("tree_desc_buffer"),
            2 => Some("element_buffer"),
            3 => Some("index_buffer"),
            _ => None
        }
    }
    
    fn iter(&self) -> RenderResourceIterator {
        RenderResourceIterator::new(self)
    }

    fn get_render_resource_hints(&self, _index: usize) -> Option<RenderResourceHints> {
        match _index {
            0 => None,
            1..=3 => Some(RenderResourceHints::BUFFER),
            _ => None
        }
    }
}