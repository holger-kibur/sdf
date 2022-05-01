use super::{dense_node::SdfNode, elements::SdfElement};
use bevy::prelude::*;

pub struct BuildingSdfNode {
    slots: Vec<BuildingSdfNode>,
    intern: Box<dyn SdfElement>,
    transform: Transform,
}

impl BuildingSdfNode {
    fn empty(prim: Box<dyn SdfElement>) -> Self {
        BuildingSdfNode {
            slots: Vec::new(),
            intern: prim,
            transform: Transform::identity(),
        }
    }

    fn set_slot(&mut self, child_node: BuildingSdfNode) -> Result<(), &'static str> {
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

    fn apply_transform(&mut self, trans: Transform) -> Result<(), ()> {
        // Intentional floating point equality test
        if trans.scale.x == 1_f32 && trans.scale.y == 1_f32 && trans.scale.z == 1_f32 {
            self.transform = self.transform * trans;
            Ok(())
        } else {
            Err(())
        }
    }

    pub fn dyn_primitive(prim: Box<dyn SdfElement>) -> Self {
        assert!(prim.get_info().is_primitive);
        Self::empty(prim)
    }

    pub fn primitive<T: SdfElement + 'static>(prim: T) -> Self {
        Self::dyn_primitive(Box::new(prim))
    }

    pub fn dyn_operation(self, op: Box<dyn SdfElement>) -> Self {
        assert!(!op.get_info().is_primitive);
        let mut new_node = Self::empty(op);
        new_node
            .set_slot(self)
            .expect("Couldn't set operation child");
        new_node
    }

    pub fn operation<T: SdfElement + 'static>(self, op: T) -> Self {
        self.dyn_operation(Box::new(op))
    }

    pub fn with(mut self, node: BuildingSdfNode) -> Self {
        self.set_slot(node).expect("Couldn't set operation slot!");
        self
    }

    pub fn transform(mut self, trans: Transform) -> Self {
        self.apply_transform(trans)
            .expect("Transformation scale not identity!");
        self
    }

    pub fn finalize(self) -> SdfNode {
        assert!(
            self.intern.get_info().is_union
                || self.slots.len() >= self.intern.get_info().num_slots(),
            "Tried finalizing SDF node without all required slots filled!"
        );
        if self.intern.get_info().is_primitive {
            SdfNode::empty(self.intern)
        } else {
            SdfNode::from_slots(
                self.intern,
                self.slots.into_iter().map(|node| node.finalize()).collect(),
            )
        }
    }
}

#[cfg(test)]
pub mod tests {
    use crate::builder::*;
    use crate::elements::*;
    use bevy::prelude::*;

    fn do_sdf_build_flat(prim: Box<dyn SdfElement>, transforms: Vec<Transform>) -> SdfNode {
        let mut root_union = BuildingSdfNode {
            slots: Vec::new(),
            intern: Box::new(SdfUnion { smooth_radius: 0.0 }),
            transform: Transform::identity(),
        };
        for trans in transforms.iter() {
            root_union =
                root_union.with(BuildingSdfNode::dyn_primitive(prim.clone()).transform(*trans));
        }
        return root_union.finalize();
    }

    fn do_sdf_build_tall(prim: Box<dyn SdfElement>, transforms: Vec<Transform>) -> SdfNode {
        let mut cur_root = BuildingSdfNode::dyn_primitive(prim);
        for trans in transforms.iter() {
            cur_root = cur_root
                .transform(*trans)
                .operation(SdfUnion { smooth_radius: 0.0 });
        }
        return cur_root.finalize();
    }

    fn do_sdf_build_single(prim: Box<dyn SdfElement>, trans: Transform) -> SdfNode {
        return BuildingSdfNode::dyn_primitive(prim)
            .transform(trans)
            .finalize();
    }

    #[test]
    fn test_builder_single() {
        let prim = Box::new(SdfSphere { radius: 1.0 });
        let root_node = do_sdf_build_single(prim.clone(), Transform::identity());
        assert!(
            root_node.intern == prim,
            "Expected node internal element isn't correct!"
        );
    }

    #[test]
    fn test_builder_flat() {
        const TEST_NUM_PRIMS: usize = 16;
        let prim = Box::new(SdfSphere { radius: 1.0 });
        let root_node = do_sdf_build_flat(
            prim.clone(),
            (0..TEST_NUM_PRIMS).map(|_| Transform::identity()).collect(),
        );
        assert!(
            root_node.slots.len() == TEST_NUM_PRIMS,
            "Number of slots not correct! Expected: {}, Found: {}!",
            TEST_NUM_PRIMS,
            root_node.slots.len()
        );
        for (i, child_prim) in root_node.slots.iter().enumerate() {
            assert!(
                child_prim.intern == prim.clone(),
                "Primitive at slot {} isn't correct!",
                i,
            );
        }
    }

    #[test]
    fn test_builder_tall() {
        const TEST_HEIGHT: usize = 16;
        let prim = Box::new(SdfSphere { radius: 1.0 });
        let mut root_node = do_sdf_build_tall(
            prim.clone(),
            (0..TEST_HEIGHT).map(|_| Transform::identity()).collect(),
        );
        for i in 0..TEST_HEIGHT {
            assert!(
                root_node.slots.len() == 1,
                "Number of slots for depth {} isn't correct! Expected 1, Found: {}!",
                i,
                root_node.slots.len()
            );
            root_node = root_node.slots.remove(0);
        }
        assert!(
            root_node.intern == prim,
            "Expected bottom node internal element isn't correct!"
        );
    }
}
