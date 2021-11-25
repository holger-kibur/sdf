#version 450

struct SdfOpSpecificBlock {
    vec4 vec4s[2];
    float floats[2];
};

struct SdfBoundingBoxBlock {
    mat4 matrix;
    vec4 scale;
    mat4 full_inverse;
    mat4 trans_inverse;
};

struct SdfOperationBlock {
    uint                op_code;
    bool                is_primitive;
    bool                is_union;
    SdfOpSpecificBlock  op_specific;
    SdfBoundingBoxBlock bounding_box;
    uint                left_child;
    uint                right_child;
};

struct NnInfo {
    uint  prim_index;
    float prim_dist;
}

struct NnStackFrame {
    SdfOperationBlock op;
    vec4              point;
    uint              index;
    float             left_ret;
    float             right_ret;
    uint              ret_branch;
};

NnStackFrame nn_stack[128];
uint         nn_stack_ptr;

#define NN_REC_CALL(BRANCH) \
    nn_stack[nn_stack_ptr++] = NnStackFrame {
        op,
        point,
        index,
    }

NnInfo _nearest_neighbor(uint index, vec4 point) {
    SdfOperationBlock op = SdfTree[index];
    if (op.is_primitive) {
        return NnInfo {
            index,
            prim_dispatch(op.bounding_box.trans_inverse * point)
        }
    } else {
        SdfOperationBlock left_child = SdfTree[top_block.op.left_child];
        float left_mindist = mindist(left_child.bounding_box, point);
        float left_maxdist = maxdist(left_child.bounding_box, point);
        SdfOperationBlock right_child = SdfTree[top_block.op.right_child];
        float right_mindist = mindist(right_child.bounding_box, point);
        float right_maxdist = maxdist(right_child.bounding_box, point);
        if (left_maxdist < right_mindist) {
            stack[stack_ptr] = NnStackFrame {
                left_child,
                downtree_dispatch(point),
                top_block.op.left_child,
            };
        } else if (right_maxdist < left_mindist) {
            stack[stack_ptr] = NnStackFrame {
                right_child,
                downtree_dispatch(point),
                top_block.op.right_child,
            };
        } else {
            stack[stack_ptr] = NnStackFrame {
                right_child,
                downtree_dispatch(point),
                top_block.op.right_child,
            };
            stack[stack_ptr + 1] = NnStackFrame {
                left_child,
                downtree_dispatch(point),
                top_block.op.left_child,
            };
            stack_ptr += 1;
        }
    }
}

float prim_dispatch(uint op_code, vec4 point) {
    // Fill out
}

float downtree_dispatch(uint op_code, vec4 point) {
    // Fill out
}

float mindist(SdfBoundingBoxBlock bbox, vec4 point) {
    vec4 trans = bbox.full_inverse * point;
    vec4 q_local = (abs(trans) - vec4(1.0)) * bbox.scale;
    return length(max(q_local, vec4(0.0))) + min(max(max(q_local.x, q_local.y), q_local.z), 0.0);
}

float maxdist(SdfBoundingBoxBlock bbox, vec4 point) {
    return max(
        length((bbox.matrix * vec4(1.0, 1.0, 1.0, 1.0)) - point),
        length((bbox.matrix * vec4(-1.0, 1.0, 1.0, 1.0)) - point),
        length((bbox.matrix * vec4(1.0, -1.0, 1.0, 1.0)) - point),
        length((bbox.matrix * vec4(-1.0, -1.0, 1.0, 1.0)) - point),
        length((bbox.matrix * vec4(1.0, 1.0, -1.0, 1.0)) - point),
        length((bbox.matrix * vec4(-1.0, 1.0, -1.0, 1.0)) - point),
        length((bbox.matrix * vec4(1.0, -1.0, -1.0, 1.0)) - point),
        length((bbox.matrix * vec4(-1.0, -1.0, -1.0, 1.0)) - point)
    );
}

NnInfo nearest_neighbor(vec4 point) {
    NnStackFrame stack[64];
    uint stack_ptr = 0;
    stack[stack_ptr] = NnStackFrame {
        SdfTree[0],
        point,
        0,
    };
    float cur_nearest = 1.0 / 0.0;
    uint cur_nearest_idx = 0;
    while (stack_ptr >= 0) {
        NnStackFrame top_block = stack[stack_ptr];
        if (top_block.op.is_union) {
            if (mindist(top_block.op.bounding_box, top_block.point) > cur_nearest) {
                stack_ptr -= 1;
            } else {
                SdfOperationBlock left_child = SdfTree[top_block.op.left_child];
                float left_mindist = mindist(left_child.bounding_box, top_block.point);
                float left_maxdist = maxdist(left_child.bounding_box, top_block.point);
                SdfOperationBlock right_child = SdfTree[top_block.op.right_child];
                float right_mindist = mindist(right_child.bounding_box, top_block.point);
                float right_maxdist = maxdist(right_child.bounding_box, top_block.point);
                if (left_maxdist < right_mindist) {
                    stack[stack_ptr] = NnStackFrame {
                        left_child,
                        downtree_dispatch(top_block.point),
                        top_block.op.left_child,
                    };
                } else if (right_maxdist < left_mindist) {
                    stack[stack_ptr] = NnStackFrame {
                        right_child,
                        downtree_dispatch(top_block.point),
                        top_block.op.right_child,
                    };
                } else {
                    stack[stack_ptr] = NnStackFrame {
                        right_child,
                        downtree_dispatch(top_block.point),
                        top_block.op.right_child,
                    };
                    stack[stack_ptr + 1] = NnStackFrame {
                        left_child,
                        downtree_dispatch(top_block.point),
                        top_block.op.left_child,
                    };
                    stack_ptr += 1;
                }
            }
        } else if (top_block.op.is_primitive) {
            float dist = prim_dispatch(top_block.op.op_code, top_block.point);
            if (dist < cur_nearest) {
                cur_nearest = dist;
                cur_nearest_idx = top_block.index;
            }
        } else {

        }
    }
}