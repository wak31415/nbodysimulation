use nalgebra::{Vector3, DMatrix};
use kiss3d::scene::SceneNode;

static G : f32  = 0.00000000006674301515151515;

pub struct Body {
    pub mass: f32,
    pub coordinates: Vector3<f32>,
    pub velocity: Vector3<f32>,
    pub node: SceneNode
}

pub fn gravitational_force(A: &Body, B: &Body) -> Vector3<f32> {
    let f = G*A.mass*B.mass / (A.coordinates - B.coordinates).norm_squared();
    f * (A.coordinates - B.coordinates).normalize()
}