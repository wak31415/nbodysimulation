extern crate kiss3d;
extern crate nalgebra as na;
extern crate rand;

use kiss3d::light::Light;
use kiss3d::scene::SceneNode;
use kiss3d::window::Window;
use na::{DMatrix, Isometry3, Vector3};
use std::sync::{Arc, Barrier, mpsc, RwLock};
use std::time::{Duration, Instant};
use std::thread;

use serde_json::{Result, Value};
use std::fs::File;
use std::io::Read;

// mod thread_manager;
mod physics;

use rand::thread_rng;
use rand::Rng;

//Number of WORKER threads: Must be more than 0
static N_THREADS: usize = 7;
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

//Message is either a Force(i,j,v) of magnitude v between objects i and j
//or Pos(i, nv, np), where nv is the new velocity of object i and np the position
//or the Stop signal which tells main that the thread is done for this stage
pub enum Msg {
    Force(usize, usize, Vector3<f32>),
    Pos(usize, Vector3<f32>, Vector3<f32>),
    Stop
}

pub fn thread_loop(
    begin: usize,
    end: usize,
    tx: mpsc::Sender<Msg>,
    bodies: Arc<RwLock<Vec<physics::Body>>>,
    forces: Arc<RwLock<DMatrix<Vector3<f32>>>>,
    f_worklist: Vec<(usize, usize)>,
    bar: Arc<Barrier>,
    n_bodies: usize
) {
    let mut local_body_copy: Vec<physics::Body> = Vec::with_capacity(n_bodies);
    // println!("Hello from worker!");
    // println!("Begin..end = {}..{}", begin, end);
    {
        let bodies_vec = bodies.read().unwrap();
        for i in 0..n_bodies {
        local_body_copy.push(physics::Body{mass: bodies_vec[i].mass, coordinates: bodies_vec[i].coordinates, velocity: bodies_vec[i].velocity});
        }
    }
    loop {
        //Update forces
        {
            let bodies_vec = bodies.read().unwrap(); //Get read lock on bodies
            for f in &f_worklist {
                let force = physics::gravitational_force(&bodies_vec[f.0], &bodies_vec[f.1]);
                tx.send(Msg::Force(f.0, f.1, force)).unwrap();
            }
            tx.send(Msg::Stop).unwrap();
        } //The read lock on bodies is dropped here
        bar.wait(); //Wait for others to finish computing forces
        // Update positions
        {
            let forces_mat = forces.read().unwrap(); //Get read lock on forces
            for i in begin..end {
                let mut net_force = Vector3::new(0f32, 0f32, 0f32);
                for j in 0..n_bodies {
                    net_force += forces_mat[(j, i)];
                }
                let acceleration = net_force / local_body_copy[i].mass;
                local_body_copy[i].velocity += acceleration*TIME_STEP;
                let nv = local_body_copy[i].velocity;
                local_body_copy[i].coordinates += nv*TIME_STEP;
                tx.send(Msg::Pos(i, nv, local_body_copy[i].coordinates)).unwrap();

            }
            tx.send(Msg::Stop).unwrap();
        } // forces lock ends here
        bar.wait();
    }
}

pub fn thread_loop_main(
    rx: mpsc::Receiver<Msg>,
    window: &mut Window,
    body_nodes: &mut Vec<SceneNode>,
    bodies: Arc<RwLock<Vec<physics::Body>>>,
    forces: Arc<RwLock<DMatrix<Vector3<f32>>>>,
    n_bodies: usize
) {
    // Main thread by convention does the 1st block because we don't want it to
    // Be handling the extra at the tail, since its already drawing alone
    // compute all n^2 forces
    //let draw_interval = Duration::from_millis(((1f32 / FPS) * 1000f32) as u64);
    //let mut t_0 = Instant::now();
    let mut stage = false; //false = writing forces, true = writing positions
    let mut stop_count = 0;
    let mut t0 = Instant::now();
    let frame_interval = Duration::from_millis(((1.0/FPS) * 1000f32) as u64);
    let mut total = Duration::from_secs(0u64);
    let mut counter : usize = 0;

    loop {
        let t_1 = Instant::now();
        if !stage {
            let mut force_mat = forces.write().unwrap();
            let t1 = Instant::now();
            if t1 - t0 > frame_interval {
                window.render();
                t0 = t1;
            }
            loop {
                if stop_count == N_THREADS {
                    stop_count = 0;
                    stage = true;
                    break;
                }
                //Receive force messages until stop_count hits N_THREADS
                let new_msg = rx.recv().unwrap();
                match new_msg {
                    Msg::Force(i, j, f) => { force_mat[(i, j)] = f; force_mat[(j, i)] = -f; },
                    Msg::Pos(_, _, _) => println!("Oh no, got pos while waiting for force!"),
                    Msg::Stop => stop_count += 1
                };
            }
        }
        if stage {
            let mut bodies_vec = bodies.write().unwrap();
            loop {
                if stop_count == N_THREADS {
                    stop_count = 0;
                    stage = false;
                    break;
                }
                //Receive body updates until stop count hits N_THREADS
                let new_msg = rx.recv().unwrap();
                match new_msg {
                    Msg::Force(_, _, _) => println!("Oh no, got force while waiting for pos"),
                    Msg::Pos(i, nv, np) => {
                        bodies_vec[i].velocity = nv;
                        bodies_vec[i].coordinates = np;
                    },
                    Msg::Stop => stop_count += 1
                }
            }
            for i in 0..n_bodies {
                body_nodes[i].set_local_transformation(Isometry3::new(bodies_vec[i].coordinates, Vector3::<f32>::new(0f32,0f32,0f32)));
            }
            // println!("Main says b2coors = {}", bodies_vec[2].coordinates);
        }
        let t_3 = Instant::now();

        if counter < 100 {
            // block_1 += t_2.duration_since(t_1);
            // block_2 += t_3.duration_since(t_2);
            total += t_3.duration_since(t_1);
            counter += 1;
        } else {
            println!("Total: {} ms \t FPS: {}", total.as_millis(), 100f32 / total.as_secs_f32());
            // block_1 = Duration::from_millis(0u64);
            // block_2 = Duration::from_millis(0u64);
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
        let mut s = window.add_sphere((obj.mass.powf(1f32/3f32) / 50f32) as f32);
        s.set_local_transformation(Isometry3::new(
            obj.coordinates,
            Vector3::new(0f32, 0f32, 0f32),
        ));
        bodies.push(s);
    }

    let forces: DMatrix<Vector3<f32>> = DMatrix::from_element(num_objects, num_objects, Vector3::new(0f32, 0f32, 0f32));

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

    let (tx, rx) = mpsc::channel::<Msg>();

    let block_size: usize = num_objects/N_THREADS;
    let n_objects = num_objects;
    let mut begin = 0;
    let mut end = begin + block_size;
    let mut threads = vec![];
    let forces_lock = Arc::new(RwLock::new(forces));
    let objects_lock = Arc::new(RwLock::new(objects));
    for i in 0..N_THREADS {
        let ntx = tx.clone();
        if i == N_THREADS - 1 {
            let worklist = f_worklists[i].clone();
            let objects_ref = objects_lock.clone();
            let forces_ref = forces_lock.clone();
            let bar_ref = barrier.clone();
            let m_begin = begin.clone();
            threads.push(thread::spawn(move || {
                thread_loop(m_begin, n_objects, ntx, objects_ref, forces_ref, worklist, bar_ref, n_objects);
            }));
            break;
        }
        let worklist = f_worklists[i].clone();
        let objects_ref = objects_lock.clone();
        let forces_ref = forces_lock.clone();
        let bar_ref = barrier.clone();
        let m_begin = begin.clone();
        let m_end = end.clone();
        threads.push(thread::spawn(move || {
            thread_loop(m_begin, m_end, ntx, objects_ref, forces_ref, worklist, bar_ref, n_objects);
        }));
        begin += block_size;
        end += block_size;
    }

    thread_loop_main(rx, &mut window, &mut bodies, objects_lock, forces_lock, n_objects);
}
