use nalgebra::{Vector3};
use kiss3d::scene::SceneNode;

pub struct Body {
    pub mass: f64,
    pub coordinates: Vector3<f64>,
    pub velocity: Vector3<f64>,
    pub node: SceneNode
}

pub fn gravitational_force(A: &Body, B: &Body) -> f64 {
    let G : f64  = 6.674301515151515 * 10f64.powf(-11f64);
    let f = G*A.mass*B.mass / (A.coordinates - B.coordinates).norm_squared();
    f
}