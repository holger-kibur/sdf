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
    for _ in 0..7 {
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
    // sdf_comp.bf_display();
    println!("{}", sdf_comp.nearest_neighbor(Vec3::new(10.0, 10.0, 40.0)).distance);
    println!("{}", sdf_comp.get_tree_expansion().nearest_neighbor(Vec3::new(10.0, 10.0, 40.0)).distance);
    // let samp_sphere = SdfBuilder::primitive(
    //     SdfSphere {
    //         radius: 5.0,
    //     }
    // )
    // .transform(Transform::from_xyz(3.0, 0.0, 3.0))
    // .operation(
    //     SdfUnion {
    //         smooth_radius: 0.0,
    //     }
    // )
    // .with(
    //     SdfBuilder::primitive(
    //         SdfSphere {
    //             radius: 3.0,
    //         }
    //     )
    //     .transform(Transform::from_xyz(-3.0, 0.0, -3.0))
    // )
    // .finalize();
    // // println!("matrix: {}", samp_sphere.bbox.unwrap().matrix);
    // // println!("inverse: {}", samp_sphere.bbox.unwrap().inverse);
    // println!("{}", samp_sphere.bbox.unwrap().distance_to(Vec3::new(-3.0, 0.0, 6.0)));
    // // let dist_info = samp_sphere.bbox_dist_info(Vec3::new(0.0, 0.0, 0.0));
    // // println!("minbound: {}, maxbound: {}", dist_info.min_bound, dist_info.max_bound);
}