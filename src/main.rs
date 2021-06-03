extern crate kiss3d;
extern crate nalgebra as na;

use kiss3d::light::Light;
use kiss3d::scene::SceneNode;
use kiss3d::window::{State, Window};
use std::sync::atomic::AtomicBool;
use na::{UnitQuaternion, Vector3, Translation3};
use std::time::SystemTime;

// mod thread_manager;
mod physics;

static N_BODIES: u64 = 2;
static N_THREADS: u32 = 1;
static FPS: f32 = 30f32;

fn help() {
    println!("usage:
nbodysimulation
    Run the n-body simulation with the default number of bodies (2)
nbodysimulation <usize>
    Run the n-body simulation with <usize> bodies");
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let num_objects: usize = match args.len() {
        1 => {
            2
        },
        2 => {
            let arg = &args[1];
            match arg.parse() {
                Ok(n) => { n },
                Err(_) => { 
                    eprintln!("error: second argument not an integer");
                    help();
                    return;
                },
            }
        }
        _ => {
            // show a help message
            help();
            return;
        }
    };

    println!("Creating {} objects", num_objects);

    let mut window = Window::new("Kiss3d: wasm example");
    window.set_light(Light::StickToCamera);

    let A = physics::Body {mass: 50f64, coordinates: Vector3::new(0f64, 0f64, 0f64), velocity: Vector3::new(1f64, 1f64, 1f64), node: window.add_cube(1.0, 1.0, 1.0) };
    let B = physics::Body {mass: 100f64, coordinates: Vector3::new(10f64, 10f64, 10f64), velocity: Vector3::new(-1f64, -1f64, -1f64), node: window.add_cube(1.0, 1.0, 1.0) };
    let f = physics::gravitational_force(&A, &B);

    println!("Force between A and B = {}", f);
    let draw_interval = 1f32/FPS;

    loop {
        //If its time to draw, then block all the threads
        //at this point and draw
        window.render();
        //update forces
        //compute new positions
    }
}
