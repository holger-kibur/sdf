mod raymarch;

use raymarch::sdf::fields::*;
use bevy::prelude::*;

#[derive(Debug, Copy, Clone)]
struct KdTup(u32, u32, u32);

fn main() {
    let mut sdf = SdfBuilder::primitive(
        SdfSphere {
            radius: 5.0,
        }
    )
    .operation(
        SdfCaaClone {
            displacement: Vec3::new(10.0, 5.0, 0.0),
            bounds: Vec3::new(100.0, 100.0, 100.0),
        }
    )
    .operation(
        SdfSurfaceSin {
            period: 1.0,
            amplitude: 0.5,
        }
    )
    .operation(
        SdfUnion {
            smooth_radius: 0.0,
        }
    )
    .with(
        SdfBuilder::primitive(
            SdfSphere {
                radius: 20.0
            }
        )
    )
    .finalize();
    sdf.bf_display();
    println!();
    sdf.get_tree_expansion().bf_display();
}