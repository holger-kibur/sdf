#version 450

layout(set = 0, binding = 0) uniform CameraViewProj {
    mat4 ViewProj;
};
layout(set = 0, binding = 2) uniform CameraPosition {
    mat4 Position;
};
layout(set = 1, binding = 1) uniform Transform {
    mat4 Model;
};

layout(location = 0) in vec3 Vertex_Position;

layout(location = 0) out vec3 initialRayPosition;
layout(location = 1) out vec3 initialRayDirection;

void main() {
    vec4 worldPosition = Model * vec4(Vertex_Position, 1.0);
    initialRayPosition = worldPosition.xyz;
    initialRayDirection = normalize(worldPosition.xyz - Position.xyz);
    gl_Position = ViewProj * worldPosition;
}
