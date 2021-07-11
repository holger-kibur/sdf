// mod dist_field;
// mod graph;
// mod ecs;
pub mod sdf;

// use bevy::prelude::*;

// Classes of objects within the raytracing pipeline can be broken down into six catagories:
// - emitters
// - receivers
// - occluders
// - neutrals
// - priority emitters
// - priority receivers

// Emitters can "emit" rays e.g. change the color and increase the luminance of a ray.
// Emitters have their diffuse and emissive textures stored.

// Receivers can "receive" rays e.g. redirect and reduce the luminance of a ray. Receivers
// have their metallic and reflectance textures stored.

// Occluders act like pure receivers that set the luminance of a ray to 0. They are
// effectively stopping points for all rays, since no possible redirected ray could have
// a positive luminance when bouncing off this object. No textures are stored for this
// object type.

// Neutrals are completely transparent to rays. All scene objects are neutral by default.
// No textures are stored.

// Emitters/occluders with priority are considered in the priority raytracing pass in
// conjunction with the PBR pass. Note that in the priority pass, priority receivers can
// only receive light from priority emitters.

