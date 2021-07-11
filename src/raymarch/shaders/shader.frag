#version 450

#define SDF_TYPE_SPHERE 0
#define SDF_TYPE_BOX 1

#define MAX_INTERSECT_ITER 64
#define DIST_EPSILON 0.0001

struct IntersectInfo {
    float min_dist;
    float ray_length;
    bool hit;
    uint intersect_info_index;
}

struct MapInfo {
    float closest_dist;
    uint closest_info_index;
}

struct SdfInfo {
    // Buffer info
    uint sdf_type;
    uint sdf_index;

    // Object info
    vec3 position;

    // Material info
    vec4 color;
};

struct DfSphere {
    float radius;
};

struct DfBox {
    vec3 dimension;
};

layout(location = 0) in vec3 initialRayPosition;
layout(location = 1) in vec3 initialRayDirection;

layout(set = 2, binding = 0) uniform SdfInfoIndex {
    uint info_index;
};
layout(set = 3, binding = 0) uniform SdfInfoBuffer_buffer_length {
    uint info_buffer_length;
};
layout(set = 3, binding = 1) buffer SdfInfoBuffer_info_buffer {
    SdfInfo[] info_buffer;
};
layout(set = 4, binding = 0) buffer SdfObjectBuffers_buffer_lengths {
    uint[] buffer_lengths;
};
layout(set = 4, binding = 1) buffer SdfObjectBuffers_sphere_buffer {
    DfSphere[] sphere_buffer;
};
layout(set = 4, binding = 2) buffer SdfObjectBuffers_box_buffer {
    DfBox[] box_buffer;
};

layout(location = 0) out vec4 colorOut;

float sdSphere(in vec3 point, in DfSphere sphere) {
    return length(point) - sphere.radius;
}

float sdBox(in vec3 point, in FfBox box) {
    vec3 q = abs(point) - box.dimension;
    return length(max(q, 0.0)) + min(max(q.x, max(q.y, q.z)), 0.0);
}

MapInfo map(in vec3 point) {
    float min_dist = 1e10;
    uint i;
    for (i = 0; i < info_buffer_length; i++) {
        SdfInfo cur_info = info_buffer[i];
        vec3 displace = point - cur_info.position;
        float cur_dist;
        switch (cur_info.sdf_type) {
            case SDF_TYPE_SPHERE:
                cur_dist = sdSphere(displace, sphere_buffer[cur_info.sdf_index]);
                break;
            case SDF_TYPE_BOX:
                cur_dist = sdBox(displace, box_buffer[cur_info.sdf_index]);
                break;
        }
        if (cur_dist < min_dist) {
            min_dist = cur_dist;
            if (cur_dist < 0) {
                break;
            }
        }
    }
    return MapInfo(min_dist, i);
}

vec3 mapNormal(in vec3 point) {
    const float h = DIST_EPSILON;
    const vec2 k = vec2(-1.0, 1.0);
    return normalize(
        k.xyy * map(point + k.xyy * h) +
        k.yyx * map(point + k.yyx * h) +
        k.yxy * map(point + k.yxy * h) +
        k.xxx * map(point + k.xxx * h)
    )
}

IntersectInfo mapIntersect(in vec3 ray_origin, in vec3 ray_direction, in float max_length) {
    float cur_length = 0.0;
    float min_dist = 1e10;
    uint closest_info_index;
    bool hit = false; 
    for (uint i = 0; i < MAX_INTERSECT_ITER; i++) {
        MapInfo minfo = map(ray_origin + cur_length * ray_direction);
        min_dist = min(min_dist, minfo.closest_dist);
        closest_info_index = minfo.closest_info_index;
        if (minfo.closest_dist < DIST_EPSILON || cur_length > max_length) {
            hit = true;
            break;
        }
        cur_length += minfo.closest_dist;
    }
    return IntersectInfo(min_dist, cur_length, hit, closest_info_index);
}

void main() {
    IntersectInfo inter = mapIntersect(initialRayPosition, initialRayDirection, 1000.0);
    colorOut = info_buffer[inter.intersect_info_index].color;
}