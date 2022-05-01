use bevy::{
    core::Byteable,
    input::mouse::{MouseScrollUnit, MouseWheel},
    prelude::*,
    reflect::TypeUuid,
    render::{
        camera::Camera,
        mesh::shape,
        pipeline::{PipelineDescriptor, RenderPipeline},
        render_graph::{base, AssetRenderResourcesNode, RenderGraph, RenderResourcesNode},
        renderer::RenderResources,
        shader::{ShaderStage, ShaderStages},
    },
};
use rand::Rng;

/// This example illustrates how to create a custom material asset and a shader that uses that
/// material
fn main() {
    App::build()
        .add_plugins(DefaultPlugins)
        .insert_resource(CameraDist(20.0))
        .add_asset::<SdfBuffer>()
        .add_asset::<SdfIndex>()
        .add_startup_system(setup.system())
        .add_system(cam_rotator.system())
        .add_system(cam_mover.system())
        .run();
}

struct CameraDist(f32);

#[derive(Default, Clone, Copy, Debug)]
#[repr(C)]
struct SdfSphere {
    pub color: Vec4,
    pub center: Vec3,
    pub radius: f32,
}

unsafe impl Byteable for SdfSphere {}

#[derive(RenderResources, TypeUuid)]
#[uuid = "6aec7d3e-63ac-4abe-9d52-b66347d5a577"]
struct SdfBuffer {
    pub num_spheres: u32,
    #[render_resources(buffer)]
    pub sdfs: Vec<SdfSphere>,
}

impl SdfBuffer {
    pub fn insert_sphere(&mut self, sphere: SdfSphere) -> SdfIndex {
        self.sdfs.push(sphere);
        self.num_spheres += 1;
        SdfIndex {
            index: self.num_spheres - 1,
        }
    }
}

impl Default for SdfBuffer {
    fn default() -> Self {
        SdfBuffer {
            num_spheres: 0,
            sdfs: Vec::new(),
        }
    }
}

#[derive(RenderResources, Default, TypeUuid)]
#[uuid = "1e08866c-0b8a-437e-8bce-37733b25127e"]
struct SdfIndex {
    pub index: u32,
}

const VERTEX_SHADER: &str = r#"
#version 450
layout(location = 0) in vec3 Vertex_Position;
layout(set = 0, binding = 0) uniform CameraViewProj {
    mat4 ViewProj;
};
layout(set = 0, binding = 2) uniform CameraPosition {
    vec4 Position;
};
layout(set = 1, binding = 1) uniform Transform {
    mat4 Model;
};

layout(location = 0) out vec3 Initial_Ray_Position;
layout(location = 1) out vec3 Initial_Ray_Direction;

void main() {
    vec4 world_position = Model * vec4(Vertex_Position, 1.0);
    Initial_Ray_Position = world_position.xyz;
    Initial_Ray_Direction = normalize(world_position.xyz - Position.xyz);
    gl_Position = ViewProj * world_position;
}
"#;

const FRAGMENT_SHADER: &str = r#"
#version 450

struct SdfSphere {
    vec4 color;
    vec3 center;
    float radius;
};

layout(location = 0) in vec3 Initial_Ray_Position;
layout(location = 1) in vec3 Initial_Ray_Direction;

layout(location = 0) out vec4 o_Target;
layout(set = 2, binding = 0) uniform SdfIndex_index {
    uint index;
};
layout(set = 3, binding = 0) uniform SdfBuffer_num_spheres {
    uint num_sdf_spheres;
};
layout(set = 3, binding = 1) buffer SdfBuffer_sdfs {
    SdfSphere[] sdfs;
};

float sdSphere(vec3 point, float radius) {
    return length(point) - radius;
}

float map(vec3 point, out uint closest) {
    float min_dist = 10000.0;
    for (uint i = 0; i < num_sdf_spheres; i++) {
        SdfSphere cur_sphere = sdfs[i];
        float cur_dist = sdSphere(point - cur_sphere.center, cur_sphere.radius);
        if (cur_dist < min_dist) {
            min_dist = cur_dist;
            closest = i;
        }
    }
    return min_dist;
}

float map(vec3 point) {
    float min_dist = 10000.0;
    for (uint i = 0; i < num_sdf_spheres; i++) {
        SdfSphere cur_sphere = sdfs[i];
        min_dist = max(min_dist, sdSphere(point - cur_sphere.center, cur_sphere.radius));
    }
    return min_dist;
}

vec4 march(vec3 pos, vec3 dir, out uint closest) {
    int i;
    for (i = 0; i < 20; i++) {
        float dist = map(pos, closest);
        if (dist < 0.00001) {
            break;
        }
        pos += dir * dist * 0.95;
    }
    if (i == 20) {
        i = -1;
    }
    return vec4(pos, i);
}

vec3 calcSphereNormal(vec3 point, uint sdf_index) {
    const float h = 0.0001;
    const vec2 k = vec2(1, -1);
    SdfSphere cur_sphere = sdfs[sdf_index];
    return normalize(
        k.xyy * sdSphere((point + k.xyy * h) - cur_sphere.center, cur_sphere.radius) +
        k.yyx * sdSphere((point + k.yyx * h) - cur_sphere.center, cur_sphere.radius) +
        k.yxy * sdSphere((point + k.yxy * h) - cur_sphere.center, cur_sphere.radius) +
        k.xxx * sdSphere((point + k.xxx * h) - cur_sphere.center, cur_sphere.radius)
    );
}

void main() {
    uint thing;
    vec4 final = march(Initial_Ray_Position, Initial_Ray_Direction, thing);
    if (final.w >= 0) {
        vec3 norm = calcSphereNormal(final.xyz, thing);
        float intensity = max(0, dot(normalize(-final.xyz), norm));
        o_Target = vec4(sdfs[thing].color.xyz * intensity, 1.0);
    } else {
        o_Target = vec4(0.678431373, 0.84375, 0.90625, 1.0);
    }
}
"#;

fn cam_mover(mut cam_dist: ResMut<CameraDist>, mut scroll_evt: EventReader<MouseWheel>) {
    for evt in scroll_evt.iter() {
        if let MouseScrollUnit::Line = evt.unit {
            cam_dist.0 -= evt.y;
        }
    }
}

fn cam_rotator(
    time: Res<Time>,
    cam_dist: Res<CameraDist>,
    mut query: Query<&mut Transform, With<Camera>>,
) {
    let mut cam_transform = query.single_mut().unwrap();
    let z = (cam_dist.0 as f64);
    let x = (cam_dist.0 as f64);
    // let z = (time.seconds_since_startup() / 10.0).cos() * (cam_dist.0 as f64);
    // let x = (time.seconds_since_startup() / 10.0).sin() * (cam_dist.0 as f64);
    *cam_transform = Transform::from_xyz(x as f32, 0.0, z as f32).looking_at(Vec3::ZERO, Vec3::Y);
}

fn setup(
    mut commands: Commands,
    mut pipelines: ResMut<Assets<PipelineDescriptor>>,
    mut shaders: ResMut<Assets<Shader>>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut indexes: ResMut<Assets<SdfIndex>>,
    mut buffers: ResMut<Assets<SdfBuffer>>,
    mut render_graph: ResMut<RenderGraph>,
) {
    // Create a new shader pipeline
    let pipeline_handle = pipelines.add(PipelineDescriptor::default_config(ShaderStages {
        vertex: shaders.add(Shader::from_glsl(ShaderStage::Vertex, VERTEX_SHADER)),
        fragment: Some(shaders.add(Shader::from_glsl(ShaderStage::Fragment, FRAGMENT_SHADER))),
    }));

    // Add an AssetRenderResourcesNode to our Render Graph. This will bind MyMaterial resources to
    // our shader
    render_graph.add_system_node("indexes", AssetRenderResourcesNode::<SdfIndex>::new(true));

    render_graph.add_system_node(
        "sdf_buffer",
        AssetRenderResourcesNode::<SdfBuffer>::new(false),
    );

    // Add a Render Graph edge connecting our new "my_material" node to the main pass node. This
    // ensures "my_material" runs before the main pass
    render_graph
        .add_node_edge("indexes", base::node::MAIN_PASS)
        .unwrap();

    render_graph
        .add_node_edge("sdf_buffer", base::node::MAIN_PASS)
        .unwrap();

    let new_sdf_handle = buffers.add(SdfBuffer::default());
    let sdf_mut = buffers.get_mut(new_sdf_handle.clone()).unwrap();

    // println!("{:?}", sdf_mut.get_render_resource_hints(0));

    // spheres
    for _ in 0..16 {
        let x = rand::thread_rng().gen_range(-20.0..20.0);
        let y = rand::thread_rng().gen_range(-20.0..20.0);
        let z = rand::thread_rng().gen_range(-20.0..20.0);
        let radius = rand::thread_rng().gen_range(1.0..4.0);
        let r = rand::thread_rng().gen_range(0.0..1.0);
        let g = rand::thread_rng().gen_range(0.0..1.0);
        let b = rand::thread_rng().gen_range(0.0..1.0);
        let new_sdf = SdfSphere {
            color: Color::rgb(r, g, b).into(),
            center: Vec3::new(x, y, z),
            radius: radius,
        };
        commands
            .spawn_bundle(MeshBundle {
                mesh: meshes.add(Mesh::from(shape::Box::new(
                    radius * 0.5_f32,
                    radius * 3_f32,
                    radius * 3_f32,
                ))),
                render_pipelines: RenderPipelines::from_pipelines(vec![RenderPipeline::new(
                    pipeline_handle.clone(),
                )]),
                transform: Transform::from_xyz(x, y, z),
                ..Default::default()
            })
            .insert(indexes.add(sdf_mut.insert_sphere(new_sdf)))
            .insert(new_sdf_handle.clone());
    }
    // camera
    commands.spawn_bundle(PerspectiveCameraBundle {
        transform: Transform::from_xyz(0.0, 0.0, -10.0).looking_at(Vec3::ZERO, Vec3::Y),
        ..Default::default()
    });
}
