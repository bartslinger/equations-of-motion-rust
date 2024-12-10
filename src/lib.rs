mod bifilar_pendulum_model;
mod rigid_body;
mod sim_output;

use crate::bifilar_pendulum_model::{BifilarPendulumModel, ModelParameters};
use crate::sim_output::SimOutput;
use wasm_bindgen::prelude::wasm_bindgen;

#[wasm_bindgen]
extern "C" {
    // Use `js_namespace` here to bind `console.log(..)` instead of just
    // `log(..)`
    #[wasm_bindgen(js_namespace = console)]
    fn log(s: &str);
}

#[wasm_bindgen]
pub fn run_bifilar_pendulum_simulation(
    duration_seconds: u32,
    dt_millis: u32,
    mass: f64,
    inertia: &[f64],
    spring1_b: &[f64],
    spring2_b: &[f64],
    spring1_w: &[f64],
    spring2_w: &[f64],
    spring_lengths: &[f64],
    initial_state: &[f64],
    spring_gains: &[f64],
    velocity_damping: &[f64],
    rotation_rate_damping: &[f64],
) -> SimOutput {
    let bifilar_pendulum_model = BifilarPendulumModel {
        params: ModelParameters {
            mass,
            inertia: nalgebra::Matrix3::new(
                inertia[0], inertia[1], inertia[2], inertia[3], inertia[4], inertia[5], inertia[6],
                inertia[7], inertia[8],
            ),
            spring1_xb: spring1_b[0],
            spring1_yb: spring1_b[1],
            spring1_zb: spring1_b[2],
            spring2_xb: spring2_b[0],
            spring2_yb: spring2_b[1],
            spring2_zb: spring2_b[2],
            spring1_xw: spring1_w[0],
            spring1_yw: spring1_w[1],
            spring1_zw: spring1_w[2],
            spring2_xw: spring2_w[0],
            spring2_yw: spring2_w[1],
            spring2_zw: spring2_w[2],
            spring1_length: spring_lengths[0],
            spring2_length: spring_lengths[1],
            spring_kp: spring_gains[0],
            spring_kd: spring_gains[1],
            velocity_damping: nalgebra::Vector3::new(
                velocity_damping[0],
                velocity_damping[1],
                velocity_damping[2],
            ),
            rotation_rate_damping: nalgebra::Vector3::new(
                rotation_rate_damping[0],
                rotation_rate_damping[1],
                rotation_rate_damping[2],
            ),
        },
    };
    let mut body = rigid_body::RigidBody::new(bifilar_pendulum_model);
    let sim_output = body.simulate(
        std::time::Duration::from_secs(duration_seconds as u64),
        // std::time::Duration::from_millis(10),
        std::time::Duration::from_millis(dt_millis as u64),
        nalgebra::Vector3::new(initial_state[0], initial_state[1], initial_state[2]),
        nalgebra::Vector3::new(initial_state[3], initial_state[4], initial_state[5]),
        nalgebra::Vector3::new(initial_state[6], initial_state[7], initial_state[8]),
        // nalgebra::Vector3::new(0.0, 0.0, 1.7155),
        nalgebra::Vector3::new(initial_state[9], initial_state[10], initial_state[11]),
        |_time_ms, _state, _dt| nalgebra::SVector::<f64, 0>::zeros(),
        None,
    );
    sim_output.clone()
}
