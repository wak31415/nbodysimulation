use nalgebra::{DMatrix, Vector3};
use kiss3d::scene::SceneNode;
use std::thread;
use std::sync::{Arc, Barrier};
// use super::dense_symmetric_matrix::SymmetricMatrix;

use super::physics;

/* Some links to some very useful ressources about
 * std::thread:
 * * [1] https://doc.rust-lang.org/book/ch16-01-threads.html
 * 
 * Some other references:
 * * [2] https://smallcultfollowing.com/babysteps/blog/2015/12/18/rayon-data-parallelism-in-rust/
 * 
 * Full spec in [3] https://moodle.polytechnique.fr/pluginfile.php/314212/mod_resource/content/2/CSE305_Project__n_Body_simulation.pdf
 */


// To be discussed later...
// We need lifetime information that can be supplied as a type argument
pub struct One_force<'a> {
    object_1: &'a physics::Body,
    object_2: &'a physics::Body,
    force_1_on_2: &'a mut Vector3<f64>,
    force_2_on_1: &'a mut Vector3<f64>
}

pub struct Assigned_work {
    forces: Vec<One_force>,
    positions: Vec<&mut physics::Body>
}

pub fn thread(barrier: Barrier,
              is_buffer_a: bool,
              buffer_a_work: Assigned_work,
              buffer_b_work: Assigned_work) {
    // compute the forces you're assigned;
    // wait to sync;
    // compute the updated positions you're assigned;
    // wait to sync;
    // loop to the top;
}

pub fn parallel_loop(bodies: &mut Vec<physics::Body>, forces: &mut DMatrix<f64>) {
    // work split?
}

pub fn sequential_loop(bodies: &mut Vec<physics::Body>, forces: &mut DMatrix<f64>) {
    // compute all n^2 forces
    assert_eq!(bodies.len(), forces.shape().0);
    assert_eq!(forces.shape().0, forces.shape().1);
    let n_bodies = bodies.len();
    let mut pos = (0, 0);
    for body_a in bodies.iter() {
        if pos.0 == n_bodies - 1 {
            break
        } 
        for body_b in bodies.iter() {
            if pos.1 == n_bodies - pos.0 - 1 {
                break
            } 
            
            let force = physics::gravitational_force(body_a, body_b);
            forces.index_mut((pos.0, pos.1)) = force;
            forces.index_mut((pos.1, pos.0)) = force;

            pos.1 += 1;
        }
        pos.0 += 1;
    }
    // compute all n positions
    for body in bodies.iter_mut() {
        physics::update_position(body, forces);
    }
}