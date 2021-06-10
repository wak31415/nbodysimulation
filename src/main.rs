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

static N_THREADS: usize = 8;
static FPS: f32 = 30f32;
static TIME_STEP: f32 = 1.0;

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
    // println!("size: {}, accessing: {}", size, 3*(i + j*size)+2);
    unsafe {
        for k in 0..3 {
            *data.offset((3*(i + j*size)+k) as isize) = val[k];
        }
    }
}

pub fn get(data: &*mut f32, size: usize, i: usize, j: usize) -> Vector3<f32> {
    let mut val: Vector3<f32> = Vector3::new(0f32,0f32,0f32);
    unsafe {
        for k in 0..3 {
            val[k] = *data.offset((3*(i + j*size)+k) as isize);
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
    bar: Arc<Barrier>,
    n_bodies: usize,
) {
    loop {
        //Update forces
        unsafe {
            for f in f_worklist.iter() {
                let force = physics::gravitational_force(&mut (*bodies.offset(f.0 as isize)), &mut (*bodies.offset(f.1 as isize)));
                set(forces, n_bodies, f.0, f.1, force);
                set(forces, n_bodies, f.1, f.0, -force);
            }
        }
        bar.wait();
        // Update positions
        for i in begin..end {
            unsafe {
                if (*bodies.offset(i as isize)).mass == 0f32 {
                    continue;
                }
            }
            let mut net_force = Vector3::new(0f32, 0f32, 0f32);
            for j in 0..n_bodies {
                net_force += get(forces, n_bodies, j, i);
            }
            unsafe {
                let acceleration = net_force / (*bodies.offset(i as isize)).mass;
                let v: Vector3<f32> = (*bodies.offset(i as isize)).velocity + acceleration * TIME_STEP;
                (*bodies.offset(i as isize)).velocity = v;
                // println!("Coordinates {} before: {}", i, (*bodies.offset(i as isize)).coordinates);
                (*bodies.offset(i as isize)).coordinates += v * TIME_STEP;
                // println!("Coordinates {} after: {}", i, (*bodies.offset(i as isize)).coordinates);
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
    let mut block_1 = Duration::from_secs(0u64);
    let mut block_2 = Duration::from_secs(0u64);
    let mut total = Duration::from_secs(0u64);
    let mut counter : usize = 1;

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
                    let r = (*bodies.offset(i as isize)).mass.powf(1f32/3f32) / 300f32;
                    body_nodes[i].set_local_scale(r, r, r);
                }
            }
            window.render();
        }
        //Update forces
        unsafe {
            for f in f_worklist.iter() {
                let force = physics::gravitational_force(&mut (*bodies.offset(f.0 as isize)), &mut (*bodies.offset(f.1 as isize)));
                set(forces, n_bodies, f.0, f.1, force);
                set(forces, n_bodies, f.1, f.0, -force);
            }
        }
        bar.wait();
        let t_2 = Instant::now();
        // Update positions
        for i in 0..block_size {
            unsafe {
                if (*bodies.offset(i as isize)).mass == 0f32 {
                    continue;
                }
            }
            let mut net_force = Vector3::new(0f32, 0f32, 0f32);
            for j in 0..n_bodies {
                net_force += get(forces, n_bodies, j, i);
            }
            unsafe {
                let acceleration = net_force / (*bodies.offset(i as isize)).mass;
                let v: Vector3<f32> = (*bodies.offset(i as isize)).velocity + acceleration * TIME_STEP;
                (*bodies.offset(i as isize)).velocity = v;
                // println!("Coordinates {} before: {}", i, (*bodies.offset(i as isize)).coordinates);
                (*bodies.offset(i as isize)).coordinates += v * TIME_STEP;
                // println!("Coordinates {} after: {}", i, (*bodies.offset(i as isize)).coordinates);
            }
        }
        bar.wait();
        let t_3 = Instant::now();

        if counter < 100 {
            block_1 += t_2.duration_since(t_1);
            block_2 += t_3.duration_since(t_2);
            total += t_3.duration_since(t_1);
            counter += 1;
        } else {
            println!("Block 1: {} ms \t Block 2: {} ms \t Total: {} ms \t FPS: {}", block_1.as_millis(), block_2.as_millis(), total.as_millis(), 100f32 / total.as_secs_f32());
            block_1 = Duration::from_millis(0u64);
            block_2 = Duration::from_millis(0u64);
            total = Duration::from_millis(0u64);
            counter = 0;
        }
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
                2f32*rand::random::<f32>() - 1f32,
                2f32*rand::random::<f32>() - 1f32,
                2f32*rand::random::<f32>() - 1f32,
            );
            let v = Vector3::new(
                2f32*rand::random::<f32>() - 1f32,
                2f32*rand::random::<f32>() - 1f32,
                2f32*rand::random::<f32>() - 1f32,
            );
            let m = 100f32 + 5000f32 * rand::random::<f32>();
            objects.push(physics::Body {
                mass: m,
                coordinates: 2f32*coord,
                velocity: 0.00001f32*v,
            });
        }
    }
    let num_objects : usize = objects.len();

    println!("Created {} objects.", num_objects);
    println!("Using {} threads.", N_THREADS);

    let mut window = Window::new("Concurrent N-Body Simulation");
    window.set_light(Light::StickToCamera);

    let mut bodies: Vec<SceneNode> = Vec::with_capacity(num_objects);
    for obj in &objects {
        let mut s = window.add_sphere((obj.mass.powf(1f32/3f32) / 300f32) as f32);
        s.set_local_transformation(Isometry3::new(
            obj.coordinates,
            Vector3::new(0f32, 0f32, 0f32),
        ));
        bodies.push(s);
    }

    // let f = physics::gravitational_force(&objects[0], &objects[1]);
    // println!("Force between A and B = {}", f);

    let forces = vec![0f32; num_objects*num_objects*3];

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

        for i in 1..N_THREADS {
            let c = Arc::clone(&barrier);
            if i == N_THREADS - 1 {
                scope.spawn(move |_| {
                    thread_loop(begin, end, objects.as_ptr() as *mut physics::Body, &(forces.as_ptr() as *mut f32), &f_worklists[i], c, num_objects);
                });
                break;
            }
            scope.spawn(move |_| {
                thread_loop(begin, end, objects.as_ptr() as *mut physics::Body, &(forces.as_ptr() as *mut f32), &f_worklists[i], c, num_objects);
            });
            begin += block_size;
            end += block_size;
        }
        let c = Arc::clone(&barrier);
        thread_loop_main(&mut window, &mut bodies, objects.as_ptr() as *mut physics::Body, &(forces.as_ptr() as *mut f32), &f_worklists[0], c, num_objects);
    }).unwrap();
}
