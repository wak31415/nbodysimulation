extern crate kiss3d;
extern crate nalgebra as na;

use kiss3d::light::Light;
use kiss3d::scene::SceneNode;
use kiss3d::window::{State, Window};
use std::sync::atomic::AtomicBool;
use na::{UnitQuaternion, Vector3, Translation3};
use std::time::{Duration, Instant};

// mod thread_manager;
mod physics;

static N_THREADS: u32 = 1;
static FPS: f32 = 30f32;

fn main() {
    let mut window = Window::new("Kiss3d: wasm example");
    window.set_light(Light::StickToCamera);

    let A = physics::Body {mass: 50f64, coordinates: Vector3::new(0f64, 0f64, 0f64), velocity: Vector3::new(1f64, 1f64, 1f64), node: window.add_cube(1.0, 1.0, 1.0) };
    let B = physics::Body {mass: 100f64, coordinates: Vector3::new(10f64, 10f64, 10f64), velocity: Vector3::new(-1f64, -1f64, -1f64), node: window.add_cube(1.0, 1.0, 1.0) };
    let f = physics::gravitational_force(&A, &B);

    println!("Force between A and B = {}", f);
    let draw_interval = Duration::from_millis(((1f32/FPS)*1000f32) as u64);
    let mut t_0 = Instant::now();

    loop {
        //If its time to draw, then block all the threads
        //at this point and draw
        let t_1 = Instant::now();
        if t_1.duration_since(t_0) > draw_interval {
            t_0 = t_1;
            window.render();
        }
        //update forces
        //compute new positions
    }
}
