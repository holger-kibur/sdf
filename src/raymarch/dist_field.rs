use bevy::{
    core::Byteable,
    prelude::*
};

// From https://stackoverflow.com/questions/32817193/how-to-get-index-of-macro-repetition-single-element
// Crackhead tier macro
macro_rules! gen_sdf_ids {
    (@step $size:expr,) => {
        const NUM_OBJECTS: usize = $size;
    };

    (@step $idx:expr, $head:ident, $($tail:ident,)*) => {
        impl SignedDistanceFieldId for $head {
            const SDF_ID: usize = $idx;
        }
        gen_sdf_ids!(@step $idx + 1usize, $($tail,)*);
    };

    ($($sdfs:ident),*) => {
        pub 
        gen_sdf_ids!(@step 0usize, $($sdfs,)*);
    };
}

macro_rules! gen_sdf_buffers {
    ($($sdfs:ident),*) => {
        impl SdfObjectBuffers {
            pub fn new() -> Self {
                SdfObjectBuffers {
                    buffers: vec![Box::new(Vec::<SdfSphere>::new())]
                }
            }
        }
    };
}

pub trait SignedDistanceField {
    const GLSL_DIST_STRUCT: &'static str;
    const GLSL_DIST_FUNC: &'static str;

    fn signed_dist(&self, point: &Vec3) -> f32;
    fn maximum_dist(&self) -> f32;
}

pub trait SignedDistanceFieldId: SignedDistanceField{
    const SDF_ID: usize;
}

pub struct SdfObjectBuffers {
    buffers: [Vec<dyn SignedDistanceFieldId>; NUM_OBJECTS as usize]
}

gen_sdf_ids![
    SdfSphere,
    SdfBox
];

impl SdfObjectBuffers {
    pub fn new() -> Self {
        SdfObjectBuffers {
            buffers: [Box::new(Vec::<SdfSphere>::new())]
        }
    }
}

// ==============
// = SDFS BELOW =
// ==============

pub struct SdfSphere {
    pub radius: f32,
}

impl SignedDistanceField for SdfSphere {
    const GLSL_DIST_STRUCT: &'static str =
    r#"
    struct dfSphere {
        float radius;
    };
    "#;
    const GLSL_DIST_FUNC: &'static str = 
    r#"
    float sdSphere(in vec3 point, in dfSphere sphere) {
        return length(point) - sphere.radius;
    }
    "#;

    fn signed_dist(&self, point: &Vec3) -> f32 {
        point.length() - self.radius
    }
}

pub struct SdfBox {
    pub dimension: Vec3,
}

impl SignedDistanceField for SdfBox {
    const GLSL_DIST_STRUCT: &'static str = 
    r#"
    struct DfBox {
        vec3 dimension;
    };
    "#;
    const GLSL_DIST_FUNC: &'static str = 
    r#"
    float sdBox(in vec3 point, in DfBox box) {
        vec3 q = abs(point) - box.dimension;
        return length(max(q, 0.0)) + min(max(q.x, max(q.y, q.z)), 0.0);
    }
    "#;

    fn signed_dist(&self, point: &Vec3) -> f32 {
        let q = point.abs() - self.dimension;
        let zero_max = q.max(Vec3::new(0.0, 0.0, 0.0));
        let dim_max = q.x.max(q.y.max(q.z)).min(0.0);
        (zero_max + Vec3::splat(dim_max)).length()
    }
}