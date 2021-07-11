

let const VERTEX_SHADER_WIREFRAME = r#"
#version 450
#define MAX_DEPTH 11

layout(location = 0) in vec3 Vertex_Position;
layout(location = 1) in uint Fractal_Depth;

layout(set = 0, binding = 0) uniform DepthColorMap {
    vec3 color_map[MAX_DEPTH];
};
layout(set = 1, binding = 0) uniform CameraViewProj {
    mat4 ViewProj;
}
layout(set = 2, binding = 0) uniform Transform {
    mat4 Model;
}

layout(location = 0) out vec4 Fragment_Color;

void main() {
    Fragment_Color = vec4(color_map[Fractal_Depth], 1.0);
    gl_Position = ViewProj * Model * vec4(Vertex_Position, 1.0);
}
"#;

let const FRAGMENT_SHADER_WIREFRAME = r#"
#version 450

layout(location = 0) in vec4 Fragment_Color;

layout(location = 0) out vec4 gl_FragColor;

void main() {
    gl_FragColor = Fragment_Color;
}
"#;