use crate::rigid_body::RigidBody;

mod gravity_model;
mod rigid_body;

fn main() {
    let gravity_model = gravity_model::GravityModel {};
    let mut body = RigidBody::new(1.0, gravity_model);
    let next_state = body.step(&9.80665, 0.01);
    println!("{:?}", next_state);
}
