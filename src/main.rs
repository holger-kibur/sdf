mod raymarch;

use raymarch::sdf::fields::*;
use bevy::prelude::*;
use rand::prelude::*;
use std::f32::consts::PI;

fn get_rand_transform() -> Transform {
    let mut rng = thread_rng();
    let mut trans = Transform::from_translation(Vec3::new(
        rng.gen_range(-20.0..20.0),
        rng.gen_range(-20.0..20.0),
        rng.gen_range(-20.0..20.0),
    ));
    trans.rotate(Quat::from_rotation_ypr(
        rng.gen_range(0.0..(2.0 * PI)),
        rng.gen_range(0.0..(2.0 * PI)),
        rng.gen_range(0.0..(2.0 * PI)),
    ));
    trans.apply_non_uniform_scale(Vec3::new(
        rng.gen_range(0.5..5.0),
        rng.gen_range(0.5..5.0),
        rng.gen_range(0.5..5.0),
    ));
    trans
}

fn main() {
    let mut sdf = SdfBuilder::primitive(
        SdfSphere {
            radius: 1.0,
        }
    )
    .transform(get_rand_transform())
    .operation(
        SdfUnion {
            smooth_radius: 0.0,
        }
    );
    for _ in 0..15 {
        sdf = sdf.with(
            SdfBuilder::primitive(
                SdfSphere {
                    radius: 1.0,
                }
            )
            .transform(get_rand_transform())
        );
    }
    let sdf_comp = sdf.finalize();
    sdf_comp.bf_display();
    sdf_comp.get_tree_expansion().bf_display();
}