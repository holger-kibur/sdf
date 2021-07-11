use std::rc::{Rc, Weak};
use std::slice::Iter;
use std::iter::Map;

macro_rules! upgrade_if_one_count {
    ($x:expr) => {
        $x.iter()
        .filter(|edge_ref| edge_ref.strong_count() == 1)
        .map(|edge_ref| edge_ref.upgrade().unwrap())
    };
}

#[derive(Debug)]
pub struct BarycentricPoint {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub w: f32,
}

impl BarycentricPoint {
    pub fn globalize(&mut self, a: &BarycentricPoint, b: &BarycentricPoint, c: &BarycentricPoint) {
        let new_x = a.x * self.x + b.x * self.y + c.x * self.z;
        let new_y = a.y * self.x + b.y * self.y + c.y * self.z;
        let new_z = a.z * self.x + b.z * self.y + c.z * self.z;
        self.x = new_x;
        self.y = new_y;
        self.z = new_z;
    }

    pub fn distance_to_sq(&self, other_point: &BarycentricPoint) -> f32 {
        let dx = other_point.x - self.x;
        let dy = other_point.y - self.y;
        let dz = other_point.z - self.z;
        dx * dx + dy * dy + dz * dz
    }
}

enum PointStoreType {
    Owns(Vec<BarycentricPoint>),
    Indexes(Vec<usize>),
}

#[derive(Debug)]
pub struct BarycentricEdge {
    pub from_index: usize,
    pub to_index: usize,
    pub length_sq: f32,
}

impl BarycentricEdge {
    pub fn new(points: &Vec<BarycentricPoint>, from_index: usize, to_index: usize) -> Self {
        BarycentricEdge {
            from_index: from_index,
            to_index: to_index,
            length_sq: points.get(from_index).unwrap().distance_to_sq(points.get(to_index).unwrap()),
        }
    }
}

#[derive(Debug)]
pub struct CircumCircle {
    pub center: BarycentricPoint,
    pub radius_sq: f32,
}

impl CircumCircle {
    pub fn new(point_list: &Vec<BarycentricPoint>, ab: &BarycentricEdge, bc: &BarycentricEdge, ca: &BarycentricEdge) -> Self {
        let mut local_circum_x = bc.length_sq * (ca.length_sq + ab.length_sq - bc.length_sq);
        let mut local_circum_y = ca.length_sq * (ab.length_sq + bc.length_sq - ca.length_sq);
        let mut local_circum_z = ab.length_sq * (bc.length_sq + ca.length_sq - ab.length_sq);
        let local_mag = local_circum_x + local_circum_y + local_circum_z;
        local_circum_x /= local_mag;
        local_circum_y /= local_mag;
        local_circum_z /= local_mag;
        let mut circum_point = BarycentricPoint {
            x: local_circum_x,
            y: local_circum_y,
            z: local_circum_z,
            w: 0.0,
        };
        let super_a_ref = point_list.get(ab.from_index).unwrap();
        circum_point.globalize(
            super_a_ref,
            point_list.get(bc.from_index).unwrap(),
            point_list.get(ca.from_index).unwrap(),
        );
        CircumCircle {
            radius_sq: circum_point.distance_to_sq(super_a_ref),
            center: circum_point,
        }
    }
}

#[derive(Debug)]
pub struct FractalTriangle {
    edge_ab: Rc<BarycentricEdge>,
    edge_bc: Rc<BarycentricEdge>,
    edge_ca: Rc<BarycentricEdge>,
    circum: CircumCircle,
}

impl FractalTriangle {
    pub fn new(
        point_list: &Vec<BarycentricPoint>,
        edge_ab: Rc<BarycentricEdge>,
        edge_bc: Rc<BarycentricEdge>,
        edge_ca: Rc<BarycentricEdge>,
    ) -> Self {
        let circum = CircumCircle::new(
            point_list,
            &*edge_ab,
            &*edge_bc,
            &*edge_ca,
        );
        FractalTriangle {
            edge_ab: edge_ab,
            edge_bc: edge_bc,
            edge_ca: edge_ca,
            circum: circum
        }
    }
}

pub struct FractalContext {
    // We need to store RC pointers to the super edges because otherwise they would
    // be dropped in the point insertion algorithm.
    super_ab: Rc<BarycentricEdge>,
    super_bc: Rc<BarycentricEdge>,
    super_ca: Rc<BarycentricEdge>,
    // Points are owned and referenced by index, because they will never be deleted while fractalizing
    // a triangle.
    pub points: PointStoreType,
    // Each edge will be owned by two triangles plus some intermediary steps in point insertion,
    // so these have to be reference counted. However, when both trianges that the edge is part of are
    // deleted, then the edge will never have to be accessed again. Therefore, Weak pointers are stored
    // so the dropping is automatic.
    pub edges: Vec<Weak<BarycentricEdge>>,
    // Triangles aren't referenced or compared, so they can be owned easily.
    pub triangles: Vec<FractalTriangle>,
}

impl FractalContext {
    pub fn make_global() -> Self {
        /// Make a global fractal context e.g. it's not inside of any parent context.
        let mut points = Vec::new();
        points.push(BarycentricPoint {
            x: 1.0,
            y: 0.0,
            z: 0.0,
            w: 0.0,
        });
        points.push(BarycentricPoint {
            x: 0.0,
            y: 1.0,
            z: 0.0,
            w: 0.0,
        });
        points.push(BarycentricPoint {
            x: 0.0,
            y: 0.0,
            z: 1.0,
            w: 0.0,
        });
        let edge_ab = Rc::new(BarycentricEdge::new(&points, 0, 1));
        let edge_bc = Rc::new(BarycentricEdge::new(&points, 1, 2));
        let edge_ca = Rc::new(BarycentricEdge::new(&points, 2, 0));
        let mut edges = Vec::new();
        edges.push(Rc::downgrade(&edge_ab));
        edges.push(Rc::downgrade(&edge_bc));
        edges.push(Rc::downgrade(&edge_ca));
        let mut triangles = Vec::new();
        triangles.push(FractalTriangle::new(&points, Rc::clone(&edge_ab), Rc::clone(&edge_bc), Rc::clone(&edge_ca)));
        FractalContext {
            super_ab: edge_ab,
            super_bc: edge_bc,
            super_ca: edge_ca,
            points: PointStoreType::Owns(points),
            edges: edges,
            triangles: triangles,
        }
    }

    pub fn make_fractal(triangle: FractalTriangle) -> Self {
        let mut points = Vec::new();
        points.push(triangle.edge_ab.from_index);
        points.push(triangle.edge_bc.from_index);
        points.push(triangle.edge_ca.from_index);
        let mut edges = Vec::new();
        edges.push(Rc::downgrade(&triangle.edge_ab));
        edges.push(Rc::downgrade(&triangle.edge_bc));
        edges.push(Rc::downgrade(&triangle.edge_ca));
        FractalContext {
            super_ab: Rc::clone(&triangle.edge_ab),
            super_bc: Rc::clone(&triangle.edge_bc),
            super_ca: Rc::clone(&triangle.edge_ca),
            points: PointStoreType::Indexes(points),
            edges: edges,
            triangles: vec![triangle],
        }
    }

    pub fn insert_point<'a>(&'a mut self, new_point: BarycentricPoint, sub_inst: &'a mut FractalContext) -> usize {

    }

    pub fn insert_point(&mut self, new_point: BarycentricPoint) {
        // Drop triangles where point is in circumcircle
        self.triangles.retain(|tri| {
            new_point.distance_to_sq(&tri.circum.center) > tri.circum.radius_sq
        });

        // Now all shared edges should have a ref-count of 0, and be dropped.
        // New triangle base edges have a count of 1. We can go through the Weak
        // pointer list to process.

        // Clean up dropped edges
        self.edges.retain(|edge_weak_ref| edge_weak_ref.strong_count() > 0);

        // List for storing crossing edges.
        let mut cross_edges: Vec<Weak<BarycentricEdge>> = Vec::new();

        // Add point to points, store index
        let new_point_index = self.points.len();
        self.points.push(new_point);

        // Create triangles from base edge list
        for base_edge in upgrade_if_one_count!(self.edges) {
            // Base edge will be the BC edge of the triangle e.g. point B -> point C
            // This means base_edge.from_point = B and base_edge.to_point = C

            // Get new edge RC pointer for the CA edge to make new triangle with.
            // Edges that are counted once should be used again because every crossing
            // edge will be used in exactly two triangles.
            let cross_ca = match upgrade_if_one_count!(cross_edges) // Upgrade temporarily for comparison.
                // Is this a CA cross edge for this base edge?
                .filter(|cross_edge| base_edge.to_index == cross_edge.from_index)
                // There should only be one, so get one
                .next()
            {
                // Edge already exists, so increase it's reference count and use it
                Some(cross_edge) => Rc::clone(&cross_edge),
                // Edge doesn't exist, so make a new one, put it in an RC, and add it's weak pointer
                // to the cross edge list for future usage.
                None => {
                    let new_cross = Rc::new(BarycentricEdge::new(&self.points, base_edge.to_index, new_point_index));
                    cross_edges.push(Rc::downgrade(&new_cross));
                    new_cross
                }
            };
            // Same except comparison and creation logic is for the AB edge
            let cross_ab = match upgrade_if_one_count!(cross_edges)
                .filter(|cross_edge| base_edge.from_index == cross_edge.to_index)
                .next()
            {
                Some(cross_edge) => Rc::clone(&cross_edge),
                None => {
                    let new_cross = Rc::new(BarycentricEdge::new(&self.points, new_point_index, base_edge.from_index));
                    cross_edges.push(Rc::downgrade(&new_cross));
                    new_cross
                }
            };
            // Create new triangle with created RC pointers.
            self.triangles.push(FractalTriangle::new(&self.points, cross_ab, base_edge, cross_ca));
        }
        // Add all new edge weak pointers to global edge weak pointer list.
        self.edges.extend(cross_edges);
    }

    pub fn iter_insert<T>(&mut self, generator: &mut T) where
        T: Iterator,
        T::Item: Clone + Into<BarycentricPoint>,
    {
        for new_point_raw in generator {
            self.insert_point(new_point_raw.clone().into());
        }
    }

    pub fn iter_points(&self) -> Iter<BarycentricPoint> {
        self.points.iter()
    }

    pub fn iter_edges<'a>(&'a self) -> impl Iterator<Item = Rc<BarycentricEdge>> + 'a {
        self.edges.iter()
            .map(|edge_weak_ref| {
                debug_assert!(edge_weak_ref.strong_count() > 0);
                edge_weak_ref.upgrade().unwrap()
            })
    }
}