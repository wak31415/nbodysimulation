use nalgebra::{UnitQuaternion, Vector3, Translation3};
use kiss3d::scene::SceneNode;

/* Some links to some very useful ressources about
 * rayon the library:
 * * [1] https://smallcultfollowing.com/babysteps/blog/2015/12/18/rayon-data-parallelism-in-rust/
 * * [2] https://docs.rs/rayon/1.5.1/rayon/index.html
 * * [3] https://www.youtube.com/watch?v=gof_OEv71Aw
 * * [4] https://docs.rs/nalgebra/0.27.0/nalgebra/index.html
 * 
 * A library is required in Rust to do any kind of concurrency,
 * and rayon appears to allow us to express our ideas with ease.
 * Some notable competition is:
 * * fork-join (not updated) ( [5] https://publications.lib.chalmers.se/records/fulltext/219016/219016.pdf)
 * * futures ( [6] https://medium.com/@robertgrosse/parallelizing-enjarify-in-go-and-rust-21055d64af7e)
 * 
 * Full spec in [7] https://moodle.polytechnique.fr/pluginfile.php/314212/mod_resource/content/2/CSE305_Project__n_body_simulation.pdf
 */

struct body {
    mass: f64,
    coordinates: mut Vector3<f64>,
    velocity: mut Vector3<f64>,
    node: SceneNode
}

/* We start with a sequential algorithm.
 * To make it directly parallelizable, we take the approach
 * of [1] employing an abstraction over the split. We will write 
 * the parallel version later, maybe with a different crate,
 * but hopefully this will still be recyclable. */

trait TaskManager {
    fn is_parallel() -> bool;
    fn<T: Fn> run_tasks(vec: Vec<T>) -> ();
}

