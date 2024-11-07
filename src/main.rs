use crate::rigid_body::{rotation_matrix_from_quaternion, RigidBody, State};
use std::f64::consts::PI;

mod gravity_model;
mod rigid_body;

const STEP_INPUT: fn(u128) -> f64 = |time| {
    if time < 5000 {
        0.0
    } else {
        1.0
    }
};

struct TestController {
    integrator: f64,
}
impl TestController {
    fn new() -> Self {
        Self { integrator: 0.0 }
    }

    fn compute_control_input(&mut self, time_ms: u128, state: &State, dt: f64) -> f64 {
        let setpoint = STEP_INPUT(time_ms);
        let rotation_matrix =
            rotation_matrix_from_quaternion(state[6], state[7], state[8], state[9]);
        let velocity_ned = rotation_matrix * nalgebra::Vector3::new(state[0], state[1], state[2]);
        let pd = state[12];
        let error = setpoint - pd;
        self.integrator += -20.0 * error * dt;
        let control_input = -25.0 * error + 10.0 * velocity_ned[2] + self.integrator;
        control_input
    }
}

fn main() {
    let gravity_model = gravity_model::GravityModel {};
    let mut body = RigidBody::new(gravity_model);

    let mut test_controller = TestController::new();

    body.simulate(
        std::time::Duration::from_secs(10),
        std::time::Duration::from_millis(10),
        nalgebra::Vector3::new(0.0, 0.0, 0.0),
        nalgebra::Vector3::new(0.0 * PI / 180.0, 0.0, 0.0),
        nalgebra::Vector3::new(0.0 * PI / 180.0, 0.0, 0.0),
        nalgebra::Vector3::new(0.0, 0.0, 0.0),
        |time_ms, state, dt| {
            let control_input = test_controller.compute_control_input(time_ms, state, dt);
            control_input
        },
        Some("output.csv"),
    );
}
