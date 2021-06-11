use nalgebra::{Vector3, DMatrix};
use serde::{Deserialize, Serialize};

static G : f32  = 0.00000000006674301515151515;
static DIST_THRESHOLD : f32 = 0.001;

#[derive(Serialize, Deserialize, Debug)]
pub struct tmpBody {
    pub mass: f32,
    pub coordinates: [f32; 3],
    pub velocity: [f32; 3]
}

#[derive(Debug)]
pub struct Body {
    pub mass: f32,
    pub coordinates: Vector3<f32>,
    pub velocity: Vector3<f32>
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

pub fn gravitational_force(A: &Body, B: &Body) -> Vector3<f32> {
    if A.mass == 0f32 || B.mass == 0f32 { return Vector3::new(0f32, 0f32, 0f32); }

    let dist = (A.coordinates - B.coordinates).norm_squared();
    if dist < DIST_THRESHOLD {
        // A.velocity = (A.mass * A.velocity + B.mass * B.velocity) / (A.mass + B.mass);
        // A.mass += B.mass;
        // B.mass = 0f32;
        return Vector3::new(0f32, 0f32, 0f32);
    }

    let f = G*A.mass*B.mass / dist;
    f * (A.coordinates - B.coordinates).normalize()
}