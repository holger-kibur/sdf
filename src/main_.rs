mod raymarch;

use std::cmp::Ordering;
use raymarch::sdf_kd_tree::*;
use rand::prelude::*;
use std::time::{Instant};

#[derive(Debug, Copy, Clone)]
struct KdTup(u32, u32, u32);

impl Compare3D<KdTup> for KdTup {
    fn xyz_lt(a: &KdTup, b: &KdTup) -> Ordering {
        (a.0 * 32 * 32 + a.1 * 32 + a.2).cmp(&(b.0 * 32 * 32 + b.1 * 32 + b.2))
    }

    fn yzx_lt(a: &KdTup, b: &KdTup) -> Ordering {
        (a.1 * 32 * 32 + a.2 * 32 + a.0).cmp(&(b.1 * 32 * 32 + b.2 * 32 + b.0))
    }

    fn zxy_lt(a: &KdTup, b: &KdTup) -> Ordering {
        (a.2 * 32 * 32 + a.0 * 32 + a.1).cmp(&(b.2 * 32 * 32 + b.0 * 32 + b.1))
    }

    fn get_null() -> Self {
        KdTup(0, 0, 0)
    }
}

fn main() {
    let mut rng = thread_rng();

    let mut x_samp: Vec<u32> = (0..32).collect();
    x_samp.shuffle(&mut rng);

    let mut y_samp: Vec<u32> = (0..32).collect();
    y_samp.shuffle(&mut rng);

    let mut z_samp: Vec<u32> = (0..32).collect();
    z_samp.shuffle(&mut rng);
    
    let mut samp_vec = Vec::with_capacity(32 * 32 * 32);

    for x in x_samp {
        for y in &y_samp {
            for z in &z_samp {
                samp_vec.push(KdTup(x, *y, *z));
            }
        }
    }
    samp_vec.remove(0);
    
    let start = Instant::now();
    let tree = KdTree3D::build(samp_vec.as_slice());
    let duration = start.elapsed();
    println!("Elapsed time: {:?}", duration);
}