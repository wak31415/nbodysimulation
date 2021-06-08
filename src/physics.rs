use nalgebra::{Vector3, DMatrix};
use serde::{Deserialize, Serialize};

static G : f64  = 0.00000000006674301515151515;

#[derive(Serialize, Deserialize, Debug)]
pub struct tmpBody {
    pub mass: f64,
    pub coordinates: [f64; 3],
    pub velocity: [f64; 3]
}

#[derive(Debug)]
pub struct Body {
    pub mass: f64,
    pub coordinates: Vector3<f64>,
    pub velocity: Vector3<f64>
}

impl tmpBody {
    pub fn convert(&self) -> Body {
        Body {
            mass: self.mass,
            coordinates: Vector3::new(self.coordinates[0], self.coordinates[1], self.coordinates[2]),
            velocity: Vector3::new(self.velocity[0], self.velocity[1], self.velocity[2])
        }
    }
}

impl std::fmt::Display for Body {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "mass: {}, coordinates: {}, velocity: {}", self.mass, self.coordinates, self.velocity)
    }
}

pub fn gravitational_force(A: &Body, B: &Body) -> Vector3<f64> {
    let f = G*A.mass*B.mass / (A.coordinates - B.coordinates).norm_squared();
    f * (A.coordinates - B.coordinates).normalize()
}

pub fn update_position(A: &mut Body, forces: &DMatrix<f64>) {
    // update position of body using a column slice, tbd
}