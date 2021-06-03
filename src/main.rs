extern crate kiss3d;
extern crate nalgebra as na;
extern crate rand;

use kiss3d::light::Light;
use kiss3d::scene::SceneNode;
use kiss3d::window::{State, Window};
use std::sync::atomic::AtomicBool;
use na::{UnitQuaternion, Vector3, Translation3};
use std::time::{Duration, Instant};

// mod thread_manager;
mod physics;

use rand::thread_rng;
use rand::Rng;

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

    let mut objects: Vec<physics::Body> = Vec::new();
    let mut window = Window::new("Kiss3d: wasm example");

    for _i in 0..num_objects {
        let coord = Vector3::new(rand::random::<f64>(), rand::random::<f64>(), rand::random::<f64>());
        let v = Vector3::new(rand::random::<f64>(), rand::random::<f64>(), rand::random::<f64>());
        let m = 1000f64 + 1000f64*rand::random::<f64>();
        objects.push(physics::Body { mass: m, coordinates: coord, velocity: v, node: window.add_sphere((m/1000f64) as f32) });
    }
    
    window.set_light(Light::StickToCamera);

    let f = physics::gravitational_force(&objects[0], &objects[1]);

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
