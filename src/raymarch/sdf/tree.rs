use super::obb::SdfBoundingBox;

pub struct SdfTreeNode {
    bbox: SdfBoundingBox,
    left_child: Box<SdfTreeNode>,
    right_child: Box<SdfTreeNode>,
}

pub struct SdfTree {
    root_node: SdfTreeNode
}