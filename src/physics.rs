use nalgebra::{Vector3};

static G : f64  = 0.00000000006674301515151515;

pub struct Body {
    pub mass: f64,
    pub coordinates: Vector3<f64>,
    pub velocity: Vector3<f64>
}

pub fn gravitational_force(A: &Body, B: &Body) -> Vector3<f64> {
    let f = G*A.mass*B.mass / (A.coordinates - B.coordinates).norm_squared();
    f * (A.coordinates - B.coordinates).normalize()
}