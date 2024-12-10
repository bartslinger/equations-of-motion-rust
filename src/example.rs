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
    // let model = gravity_model::GravityModel {};
    let model = rcam_model::RcamModel {};
    // let model = talon250_model::Talon250Model {};
    // let model = bifilar_pendulum_model::BifilarPendulumModel {
    //     params: ModelParameters::default(),
    // };

    let mut body = RigidBody::new(model);

    let mut throttle_integrator: f64 = 0.0;
    let mut pitch_integrator: f64 = 0.0;

    body.simulate(
        std::time::Duration::from_secs(50),
        std::time::Duration::from_millis(10),
        nalgebra::Vector3::new(90.0, 0.0, 0.0),
        nalgebra::Vector3::new(0.0 * PI / 180.0, 0.0, 0.0),
        nalgebra::Vector3::new(0.0 * PI / 180.0, 0.0 * PI / 180.0, 0.0),
        nalgebra::Vector3::new(0.0, 0.0, 0.0),
        |_time_ms, state, dt| {
            // use a control law to find the trim point
            let (u, v, w) = (state[0], state[1], state[2]);

            // euler angles roll pitch yaw
            let q = nalgebra::Quaternion::new(state[6], state[7], state[8], state[9]);
            let (_roll, pitch, _yaw) = nalgebra::UnitQuaternion::from_quaternion(q).euler_angles();

            // simple pitch control
            let pitch_error = 0.0 - pitch;
            pitch_integrator += -0.8 * pitch_error * dt;
            let d_e = -1.0 * pitch_error + pitch_integrator;

            let v_a = nalgebra::Vector3::new(u, v, w).norm();
            let v_err = 90.0 - v_a;
            throttle_integrator += v_err * dt;
            // simple throttle control
            let throttle = 0.6 + 0.6 * v_err + 0.5 * throttle_integrator;
            let d_th = throttle.min(1.0).max(0.0);

            nalgebra::SVector::<f64, 5>::new(0.0, d_e, 0.0, d_th, d_th)
        },
        Some("output.csv"),
    );
}
