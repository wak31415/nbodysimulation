extern crate kiss3d;
extern crate nalgebra as na;
extern crate rand;

use kiss3d::light::Light;
use kiss3d::scene::SceneNode;
use kiss3d::window::{State, Window};
use std::sync::{Arc, Barrier};
use na::{UnitQuaternion, Vector3, Isometry3, DMatrix};
use std::time::{Duration, Instant};

mod thread_manager;
mod physics;

use rand::thread_rng;
use rand::Rng;

static N_THREADS: usize = 1;
static FPS: f32 = 30f32;
static TIME_STEP: f32 = 0.01;

fn help() {
    println!("usage:
nbodysimulation
    Run the n-body simulation with the default number of bodies (2)
nbodysimulation <usize>
    Run the n-body simulation with <usize> bodies");
}

pub fn thread_loop(begin: usize, end: usize, bodies: &mut Vec<physics::Body>, forces: &mut DMatrix<Vector3<f32>>, f_worklist: &Vec<(usize, usize)>, bar: Arc<Barrier>) {
    assert_eq!(bodies.len(), forces.shape().0);
    assert_eq!(forces.shape().0, forces.shape().1);
    let n_bodies = bodies.len();
    let n_forces = forces.len();
    loop {
        //Update forces
        for f in f_worklist.iter() {
            let force = physics::gravitational_force(&bodies[f.0], &bodies[f.1]);
            forces[(f.0, f.1)] = force;
            forces[(f.1, f.0)] = -force;
        }
        bar.wait();
        // Update positions
        for i in begin..end {
            let mut net_force = Vector3::new(0f32,0f32,0f32);
            for j in 0..n_bodies {
                net_force += forces[(i, j)];
            }
            let acceleration = net_force/bodies[i].mass;
            bodies[i].velocity += acceleration*TIME_STEP;
            bodies[i].coordinates += bodies[i].velocity*TIME_STEP;
        }
        bar.wait();
    }
}

pub fn thread_loop_main(window: &mut Window, body_nodes: &mut Vec<SceneNode>, bodies: &mut Vec<physics::Body>, forces: &mut DMatrix<Vector3<f32>>, f_worklist: &Vec<(usize, usize)>, bar: Arc<Barrier>) {
    // Main thread by convention does the 1st block because we don't want it to
    // Be handling the extra at the tail, since its already drawing alone
    // compute all n^2 forces
    assert_eq!(bodies.len(), forces.shape().0);
    assert_eq!(forces.shape().0, forces.shape().1);
    let n_bodies = bodies.len();
    let block_size = n_bodies/N_THREADS;
    let n_forces = forces.len();
    let draw_interval = Duration::from_millis(((1f32/FPS)*1000f32) as u64);
    let mut t_0 = Instant::now();
    loop {
        //As you can see, the difference is that only the main thread draws
        //Draw if its time to do so
        let t_1 = Instant::now();
        if t_1.duration_since(t_0) > draw_interval {
            t_0 = t_1;
            for i in 0..n_bodies {
                body_nodes[i].set_local_transformation(Isometry3::new(bodies[i].coordinates, Vector3::new(0f32,0f32,0f32)));
            }
            window.render();
        }
        //Update forces
        for f in f_worklist.iter() {
            let force = physics::gravitational_force(&bodies[f.0], &bodies[f.1]);
            forces[(f.0, f.1)] = force;
            forces[(f.1, f.0)] = -force;
        }
        bar.wait();
        // Update positions
        for i in 0..block_size {
            let mut net_force = Vector3::new(0f32,0f32,0f32);
            for j in 0..n_bodies {
                net_force += forces[(i, j)];
            }
            let acceleration = net_force/bodies[i].mass;
            bodies[i].velocity += acceleration*TIME_STEP;
            bodies[i].coordinates += bodies[i].velocity*TIME_STEP;
        }
        bar.wait();
    }
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

    let mut objects: Vec<physics::Body> = Vec::with_capacity(num_objects);
    let mut window = Window::new("Kiss3d: wasm example");

    for _i in 0..num_objects {
        let coord = Vector3::new(rand::random::<f64>(), rand::random::<f64>(), rand::random::<f64>());
        let v = Vector3::new(rand::random::<f64>(), rand::random::<f64>(), rand::random::<f64>());
        let m = 1000f64 + 1000f64*rand::random::<f64>();
        objects.push(physics::Body { mass: m, coordinates: coord, velocity: v, node: window.add_sphere((m/100000f64) as f32) });
        match objects.last() {
            Some(o) => println!("{}", o.coordinates),
            None => {},
        }
    }
    
    window.set_light(Light::StickToCamera);

    let f = physics::gravitational_force(&objects[0], &objects[1]);
    println!("Force between A and B = {}", f);

    let draw_interval = Duration::from_millis(((1f32/FPS)*1000f32) as u64);
    let mut t_0 = Instant::now();

    loop {
        //If its time to draw, then draw
        let t_1 = Instant::now();
        if t_1.duration_since(t_0) > draw_interval {
            t_0 = t_1;
            window.render();
        }
        //update forces
        //compute new positions
    }
}
