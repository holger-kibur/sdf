mod raymarch;

use nalgebra::{
    Vector4, matrix
};
use raymarch::sdf::fields::*;
use raymarch::sdf::component::*;
use std::time::{Duration, Instant};
use bevy::prelude::*;
use rand::prelude::*;
use std::f32::consts::PI;

fn get_rand_transform() -> Transform {
    let mut rng = thread_rng();
    let mut trans = Transform::from_translation(Vec3::new(
        rng.gen_range(-50.0..50.0),
        rng.gen_range(-50.0..50.0),
        rng.gen_range(-50.0..50.0),
    ));
    trans.rotate(Quat::from_rotation_ypr(
        rng.gen_range(0.0..(2.0 * PI)),
        rng.gen_range(0.0..(2.0 * PI)),
        rng.gen_range(0.0..(2.0 * PI)),
    ));
    // trans.apply_non_uniform_scale(Vec3::new(
    //     rng.gen_range(0.5..10.0),
    //     rng.gen_range(0.5..10.0),
    //     rng.gen_range(0.5..10.0),
    // ));
    trans
}

fn main() {
    let mut sdf = SdfBuilder::primitive(
        SdfSphere {
            radius: 5.0,
        }
    )
    .transform(get_rand_transform())
    .operation(
        SdfUnion {
            smooth_radius: 0.0,
        }
    );
    for _ in 0..2 {
        sdf = sdf.with(
            SdfBuilder::primitive(
                SdfSphere {
                    radius: 5.0,
                }
            )
            .transform(get_rand_transform())
        );
    }
    let sdf_comp = sdf.finalize();
    let create_start = Instant::now();
    let sdf_comp_expand = sdf_comp.get_tree_expansion();
    println!("create: {:?}", create_start.elapsed());
    // sdf_comp.bf_display();
    let first_start = Instant::now();
    println!("{}", sdf_comp.nearest_neighbor(Vec3::new(0.0, 0.0, 0.0)).distance);
    println!("first: {:?}", first_start.elapsed());
    let second_start = Instant::now();
    println!("{}", sdf_comp_expand.nearest_neighbor(Vec3::new(0.0, 0.0, 0.0)).distance);
    println!("second: {:?}", second_start.elapsed());
    let buf: SdfTreeBuffer = (&sdf_comp_expand).into();
    for x in buf.op_buffer.iter() {
        print!("{:#?}", x);
    }
}