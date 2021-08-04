mod raymarch;

use raymarch::sdf::fields::*;
use bevy::prelude::*;
use rand::prelude::*;

fn main() {
    let sdf = SdfBuilder::primitive(
        SdfSphere {
            radius: 5.0,
        }
    )
    .transform(Transform::from_rotation(Quat::from_rotation_z(0.785)))
    .operation(
        SdfUnion {
            smooth_radius: 0.0,
        }
    )
    .with(
        SdfBuilder::primitive(
            SdfSphere {
                radius: 5.0,
            }
        )
    );
    let sdf_comp = sdf.finalize();
    sdf_comp.bf_display();
    sdf_comp.get_tree_expansion().bf_display();
}