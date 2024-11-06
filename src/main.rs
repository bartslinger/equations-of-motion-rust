use crate::rigid_body::RigidBody;

mod gravity_model;
mod rigid_body;

fn main() {
    let gravity_model = gravity_model::GravityModel {};
    let mass = 1.0;
    let inertia = nalgebra::Matrix3::new(0.0135, 0.0, 0.0, 0.0, 0.0010, 0.0, 0.0, 0.0, 0.0142);
    let mut body = RigidBody::new(gravity_model, mass, inertia);
    let next_state = body.step(&(), 0.01);
    println!("{:?}", next_state);
}
