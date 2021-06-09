extern crate kiss3d;
extern crate nalgebra as na;
extern crate rand;

use kiss3d::light::Light;
use kiss3d::scene::SceneNode;
use kiss3d::window::{State, Window};
use na::{DMatrix, Isometry3, UnitQuaternion, Vector3};
use std::sync::{Arc, Barrier};
use std::time::{Duration, Instant};
use std::thread;
use closure::closure;
use crossbeam;

use serde_json::{Result, Value};
use std::fs::File;
use std::io::Read;

// mod thread_manager;
mod physics;

use rand::thread_rng;
use rand::Rng;

static N_THREADS: usize = 1;
static FPS: f32 = 30f32;
static TIME_STEP: f32 = 0.01;

fn help() {
    println!(
        "usage:
nbodysimulation
    Run the n-body simulation with the default number of bodies (2)
nbodysimulation <usize>
    Run the n-body simulation with <usize> bodies"
    );
}

pub fn set(data: &*mut f32, size: usize, i: usize, j: usize, val: Vector3<f32>) {
    unsafe {
        for i in 0..3 {
            *data.offset((3*(i + j*size)+i) as isize) = val[i];
        }
    }
}

pub fn get(data: &*mut f32, size: usize, i: usize, j: usize) -> Vector3<f32> {
    let mut val: Vector3<f32> = Vector3::new(0f32,0f32,0f32);
    unsafe {
        for i in 0..3 {
            val[i] = *data.offset((3*(i + j*size)+i) as isize);
        }
    }
    val
}

pub fn thread_loop(
    begin: usize,
    end: usize,
    bodies: *mut physics::Body,
    forces: &*mut f32,
    f_worklist: &Vec<(usize, usize)>,
    bar: &Arc<Barrier>,
    n_bodies: usize,
) {
    loop {
        //Update forces
        unsafe {
            for f in f_worklist.iter() {
                let force = physics::gravitational_force(&(*(bodies.offset(f.0 as isize))), &(*(bodies.offset(f.1 as isize))));
                set(forces, n_bodies, f.0, f.1, force);
                set(forces, n_bodies, f.1, f.0, -force);
            }
        }
        bar.wait();
        // Update positions
        for i in begin..end {
            let mut net_force = Vector3::new(0f32, 0f32, 0f32);
            for j in 0..n_bodies {
                net_force += get(forces, n_bodies, i, j);
            }
            unsafe {
                let mut b = bodies.offset(i as isize);
                let acceleration = net_force / (*b).mass;
                let v: Vector3<f32> = acceleration * TIME_STEP;
                (*b).velocity = v;
                (*b).coordinates += v * TIME_STEP;
            }
        }
        bar.wait();
    }
}

pub fn thread_loop_main(
    window: &mut Window,
    body_nodes: &mut Vec<SceneNode>,
    bodies: *mut physics::Body,
    forces: &*mut f32,
    f_worklist: &Vec<(usize, usize)>,
    bar: Arc<Barrier>,
    n_bodies: usize,
) {
    // Main thread by convention does the 1st block because we don't want it to
    // Be handling the extra at the tail, since its already drawing alone
    // compute all n^2 forces
    let block_size = n_bodies / N_THREADS;
    let draw_interval = Duration::from_millis(((1f32 / FPS) * 1000f32) as u64);
    let mut t_0 = Instant::now();
    loop {
        //As you can see, the difference is that only the main thread draws
        //Draw if its time to do so
        let t_1 = Instant::now();
        if t_1.duration_since(t_0) > draw_interval {
            t_0 = t_1;
            for i in 0..n_bodies {
                unsafe {
                    body_nodes[i].set_local_transformation(Isometry3::new(
                        (*bodies.offset(i as isize)).coordinates,
                        Vector3::new(0f32, 0f32, 0f32),
                    ));
                }
            }
            window.render();
        }
        //Update forces
        unsafe {
            for f in f_worklist.iter() {
                let force = physics::gravitational_force(&(*(bodies.offset(f.0 as isize))), &(*(bodies.offset(f.1 as isize))));
                set(forces, n_bodies, f.0, f.1, force);
                set(forces, n_bodies, f.1, f.0, -force);
            }
        }
        bar.wait();
        // Update positions
        for i in 0..block_size {
            let mut net_force = Vector3::new(0f32, 0f32, 0f32);
            for j in 0..n_bodies {
                net_force += get(forces, n_bodies, i, j);
            }
            unsafe {
                let mut b = bodies.offset(i as isize);
                let acceleration = net_force / (*b).mass;
                let v: Vector3<f32> = acceleration * TIME_STEP;
                (*b).velocity = v;
                (*b).coordinates += v * TIME_STEP;
            }
        }
        bar.wait();
    }
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let rand_num_obj: usize = match args.len() {
        1 => 0,
        2 => {
            let arg = &args[1];
            match arg.parse() {
                Ok(n) => n,
                Err(_) => {
                    eprintln!("error: second argument not an integer");
                    help();
                    return;
                }
            }
        }
        _ => {
            // show a help message
            help();
            return;
        }
    };
    let mut objects: Vec<physics::Body> = Vec::new();

    if rand_num_obj == 0 {
        let mut file = File::open("starting_configuration.json").unwrap();
        let mut data = String::new();
        file.read_to_string(&mut data).unwrap();
        let json_objects: Vec<physics::tmpBody> = serde_json::from_str(&data).unwrap();
        for obj in &json_objects {
            objects.push(obj.convert());
        }
    } else {
        for _i in 0..rand_num_obj {
            let coord = Vector3::new(
                rand::random::<f32>(),
                rand::random::<f32>(),
                rand::random::<f32>(),
            );
            let v = Vector3::new(
                rand::random::<f32>(),
                rand::random::<f32>(),
                rand::random::<f32>(),
            );
            let m = 1000f32 + 1000f32 * rand::random::<f32>();
            objects.push(physics::Body {
                mass: m,
                coordinates: coord,
                velocity: v,
            });
        }
    }
    let num_objects : usize = objects.len();

    println!("Created {} objects", num_objects);
    let mut window = Window::new("Kiss3d: wasm example");
    window.set_light(Light::StickToCamera);

    let mut bodies: Vec<SceneNode> = Vec::with_capacity(num_objects);
    for obj in &objects {
        let mut s = window.add_sphere((obj.mass.powf(1f32/3f32) / 50f32) as f32);
        s.set_local_transformation(Isometry3::new(
            obj.coordinates,
            Vector3::new(0f32, 0f32, 0f32),
        ));
        bodies.push(s);
    }

    let f = physics::gravitational_force(&objects[0], &objects[1]);
    println!("Force between A and B = {}", f);

    let mut forces = vec![0f32; num_objects*num_objects*3];

    let mut f_worklists: Vec::<Vec::<(usize, usize)>> = Vec::with_capacity(N_THREADS);
    for _ in 0..N_THREADS {
        f_worklists.push(Vec::<(usize, usize)>::new());
    }
    let mut curr = 0usize;
    for i in 0..num_objects {
        for j in (i + 1)..num_objects {
            f_worklists[curr].push((i, j));
            curr = (curr + 1) % N_THREADS;
        }
    }
    let barrier = Arc::new(Barrier::new(N_THREADS));

    let block_size: usize = num_objects/N_THREADS;
    let mut begin = block_size;
    let mut end = begin + block_size;
    // let mut threads = vec![];
    crossbeam::scope(|scope| {
        let objects = &objects;
        let forces = &forces;
        let f_worklists = &f_worklists;
        let barrier = &barrier;

        for i in 1..N_THREADS {
            if i == N_THREADS - 1 {
                scope.spawn(move |_| {
                    thread_loop(begin, num_objects, objects.as_ptr() as *mut physics::Body, &(forces.as_ptr() as *mut f32), &f_worklists[i], barrier, num_objects);
                });
                break;
            }
            scope.spawn(move |_| {
                thread_loop(begin, end, objects.as_ptr() as *mut physics::Body, &(forces.as_ptr() as *mut f32), &f_worklists[i], barrier, num_objects);
            });
            begin += block_size;
            end += block_size;
        }
    })
    .unwrap();
    thread_loop_main(&mut window, &mut bodies, objects.as_ptr() as *mut physics::Body, &(forces.as_ptr() as *mut f32), &f_worklists[0], barrier, num_objects);
}
