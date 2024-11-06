use crate::rigid_body::{DynamicsModel, State};

pub struct GravityModel {}
impl DynamicsModel for GravityModel {
    type ControlInput = f64;
    fn compute_forces_and_moments(
        &self,
        state: State,
        control_input: &Self::ControlInput,
    ) -> (f64, f64, f64, f64, f64, f64) {
        (0.0, 0.0, *control_input, 0.0, 0.0, 0.0)
    }
}
