use nalgebra::{Vector3, DMatrix};
use kiss3d::scene::SceneNode;

static G : f64  = 0.00000000006674301515151515;

pub struct Body {
    pub mass: f64,
    pub coordinates: Vector3<f64>,
    pub velocity: Vector3<f64>,
    pub node: SceneNode
}

pub fn gravitational_force(A: &Body, B: &Body) -> Vector3<f64> {
    let f = G*A.mass*B.mass / (A.coordinates - B.coordinates).norm_squared();
    f * (A.coordinates - B.coordinates).normalize()
}

pub fn update_position(A: &mut Body, forces: &DMatrix<f64>) {
    // update position of body using a column slice, tbd
}