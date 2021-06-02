extern crate kiss3d;
extern crate nalgebra as na;

use kiss3d::light::Light;
use kiss3d::scene::SceneNode;
use kiss3d::window::{State, Window};
use na::{UnitQuaternion, Vector3, Translation3};

struct AppState {
    c: SceneNode,
    trans: Translation3<f32>,
}

impl State for AppState {
    fn step(&mut self, _: &mut Window) {
        self.c.prepend_to_local_translation(&self.trans);
    }
}

fn main() {
    let mut window = Window::new("Kiss3d: wasm example");
    let mut c = window.add_cube(1.0, 1.0, 1.0);
    c.set_color(1.0, 0.0, 0.0);

    let mut s = window.add_cube(10.0, 10.0, 10.0);
    s.set_color(0.0, 0.0, 1.0);

    window.set_light(Light::StickToCamera);

    //let rot = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.001);
    let trans = Translation3::new(-0.02f32, 0f32, 0f32);
    let state = AppState { c, trans };

    window.render_loop(state)
}
