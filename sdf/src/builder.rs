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
