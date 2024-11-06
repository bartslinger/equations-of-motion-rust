use crate::rigid_body::RigidBody;

mod gravity_model;
mod rigid_body;

fn main() {
    let gravity_model = gravity_model::GravityModel {};
    let mass = 1.0;
    let inertia = nalgebra::Matrix3::new(0.0135, 0.0, 0.0, 0.0, 0.0010, 0.0, 0.0, 0.0, 0.0142);
    let mut body = RigidBody::new(gravity_model, mass, inertia);

    body.simulate(
        std::time::Duration::from_secs(10),
        std::time::Duration::from_millis(10),
        nalgebra::Vector3::new(0.0, 0.0, 0.0),
        nalgebra::Vector3::new(15.0 * std::f64::consts::PI / 180.0, 0.0, 0.0),
        nalgebra::Vector3::new(0.0, 0.0, 0.0),
        nalgebra::Vector3::new(0.0, 0.0, 0.0),
        |time| {
            if time < 5000 {
                0.5 * 9.80665
            } else {
                0.0
            }
        },
        Some("output.csv"),
    );
}
