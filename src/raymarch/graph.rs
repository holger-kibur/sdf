use bevy::{
    prelude::*,
    render::render_graph::{
        RenderGraph,
        AssetRenderResourcesNode
    }
};

pub mod node {
    pub const SDF_OBJECT_BUFFER: &str = "sdf_object_buffer";
    pub const SDF_UNIFORM_BUFFER: &str = "sdf_uniform_buffer";
}

pub fn add_sdf_nodes(
    glob_sdf_render: Res<>
    mut render_graph: ResMut<RenderGraph>
) {
    render_graph.add_system_node(
        node::SDF_OBJECT_BUFFER,
        AssetRenderResourcesNode::<false>
    );
}