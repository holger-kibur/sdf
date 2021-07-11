use crate::terrain::graph::FractalContext;
use std::collections::VecDeque;

struct FractalizeJob {

}

pub struct TerrainBuilder {
    parent_frac_ctx: FractalContext,
    fractal_queue: VecDeque<FractalizeJob>,
}