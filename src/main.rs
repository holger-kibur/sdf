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
        SdfUnion {
            smooth_radius: 0.0,
        }
    );
    for _ in 0..16 {
        sdf = sdf.with(
            SdfBuilder::primitive(
                SdfSphere {
                    radius: 20.0
                }
            )
        )
    }
    let sdf_comp = sdf.finalize();
    sdf_comp.bf_display();
    println!();
    sdf_comp.get_tree_expansion().bf_display();
}