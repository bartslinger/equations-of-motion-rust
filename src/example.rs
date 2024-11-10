use crate::bifilar_pendulum_model::ModelParameters;
use crate::rigid_body::{rotation_matrix_from_quaternion, RigidBody, State};
use std::f64::consts::PI;

mod bifilar_pendulum_model;
mod gravity_model;
mod rcam_model;
mod rigid_body;
mod sim_output;
mod talon250_model;

#[allow(unused)]
const STEP_INPUT: fn(u128) -> f64 = |time| {
    if time < 5000 {
        0.0
    } else {
        1.0
    }
};

#[allow(unused)]
struct TestController {
    integrator: f64,
}
impl TestController {
    #[allow(unused)]
    fn new() -> Self {
        Self { integrator: 0.0 }
    }

    #[allow(unused)]
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
    // // test calculation
    // // Inertia = MgT^2b^2 / 4 pi^2 L
    // let t: f64 = 1.833;
    // let r: f64 = 0.315 / 2.0;
    // let l: f64 = 1.65;
    // let m = 0.350;
    // let g = 9.80665;
    // let ixx = (m * g * t.powi(2) * r.powi(2)) / (4.0 * PI.powi(2) * l);
    // println!("Ixx calculated: {}", ixx);
    //
    // // expected T for Ixx = 0.05
    // let t = (2.0 * PI / r) * (l * 0.05 / (m * g)).sqrt();
    // println!("Expected period: {}", t);
    // // inverse calculation as check
    // let ixx = (m * g * t.powi(2) * r.powi(2)) / (4.0 * PI.powi(2) * l);
    // println!("Ixx calculated: {}", ixx);

    // let gravity_model = gravity_model::GravityModel {};
    // let rcam_model = rcam_model::RcamModel {};
    // let talon250_model = talon250_model::Talon250Model {};
    let bifilar_pendulum_model = bifilar_pendulum_model::BifilarPendulumModel {
        params: ModelParameters::default(),
    };
    let mut body = RigidBody::new(bifilar_pendulum_model);

    body.simulate(
        std::time::Duration::from_secs(15),
        // std::time::Duration::from_millis(10),
        std::time::Duration::from_millis(1),
        nalgebra::Vector3::new(0.0, 0.0, 0.0),
        nalgebra::Vector3::new(10.0 * PI / 180.0, 0.0, 0.0),
        nalgebra::Vector3::new(0.0 * PI / 180.0, 90.0 * PI / 180.0, 0.0),
        // nalgebra::Vector3::new(0.0, 0.0, 1.7155),
        nalgebra::Vector3::new(0.0, 0.0, 1.72),
        |_time_ms, _state, _dt| (),
        Some("output.csv"),
    );
}
