use nalgebra::{MatrixN};
use kiss3d::scene::SceneNode;
use std::thread;
use std::sync::{Arc, Barrier};
use super::dense_symmetric_matrix::SymmetricMatrix;

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
pub struct One_force {
    object_1: &physics::Body,
    object_2: &physics::Body,
    force_1_on_2: &mut f64,
    force_2_on_1: &mut f64
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

pub fn parallel_loop<D>(bodies: &mut Vec<physics::Body>, forces: &mut MatrixN<f64, D>) {
    // work split?
}

pub fn sequential_loop() {
    // compute all n^2 forces
    // compute all n positions
}