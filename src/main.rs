mod raymarch;

use raymarch::sdf::fields::*;
use bevy::prelude::*;
use rand::prelude::*;

#[derive(Debug, Copy, Clone)]
struct KdTup(u32, u32, u32);

fn main() {
    let sdf = SdfBuilder::primitive(
        SdfSphere {
            radius: 5.0,
        }
    )
    .operation(
        SdfUnion {
            smooth_radius: 0.0,
        }
    );
    let sdf_comp = sdf.finalize();
    sdf_comp.bf_display();
}