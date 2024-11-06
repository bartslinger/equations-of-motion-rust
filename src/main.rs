use crate::rigid_body::RigidBody;

mod gravity_model;
mod rigid_body;

fn main() {
    let gravity_model = gravity_model::GravityModel {};
    let mass = 1.0;
    let inertia = nalgebra::Matrix3::new(0.0135, 0.0, 0.0, 0.0, 0.0010, 0.0, 0.0, 0.0, 0.0142);
    let mut body = RigidBody::new(gravity_model, mass, inertia);
    body.set_state(3, 30.0 * std::f64::consts::PI / 180.0);
    body.set_output_file("output.csv");

    body.simulate(
        std::time::Duration::from_secs(10),
        std::time::Duration::from_millis(10),
        |time| {
            if time < 5000 {
                9.80665
            } else {
                0.0
            }
        },
    );
}
