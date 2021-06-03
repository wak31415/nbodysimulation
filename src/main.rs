extern crate kiss3d;
extern crate nalgebra as na;

use kiss3d::light::Light;
use kiss3d::scene::SceneNode;
use kiss3d::window::{State, Window};
use na::{UnitQuaternion, Vector3, Translation3};

// mod thread_manager;
mod physics;

struct AppState {
    c: SceneNode,
    trans: Translation3<f32>,
}

impl State for AppState {
    fn step(&mut self, _: &mut Window) {
        self.c.prepend_to_local_translation(&self.trans);
    }
}

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
    let A = physics::Body {mass: 5000f64, coordinates: Vector3::new(0f64, 0f64, 0f64), velocity: Vector3::new(1f64, 1f64, 1f64), node: window.add_cube(1.0, 1.0, 1.0) };
    let B = physics::Body {mass: 100000f64, coordinates: Vector3::new(2f64, 10f64, 10f64), velocity: Vector3::new(-1f64, -1f64, -1f64), node: window.add_cube(1.0, 1.0, 1.0) };
    let f = physics::gravitational_force(&A, &B);

    println!("Force between A and B = {}", f);
    
    let mut c = window.add_cube(1.0, 1.0, 1.0);
    c.set_color(1.0, 0.0, 0.0);

    // some kind of fixed reference    
    let mut s = window.add_cube(10.0, 10.0, 10.0);
    s.set_color(0.0, 0.0, 1.0);
    /* try to find how to make the walls of an object
     * double-sided, this was supposed to be "the universe" */ 

    window.set_light(Light::StickToCamera);

    //let rot = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.001);
    let trans = Translation3::new(-0.02f32, 0f32, 0f32);
    let state = AppState { c, trans };

    window.render_loop(state)
}
