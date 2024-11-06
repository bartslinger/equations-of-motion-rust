use crate::rigid_body::{DynamicsModel, State};

pub struct GravityModel {}
impl DynamicsModel for GravityModel {
    type ControlInput = ();
    fn compute_forces_and_moments(
        &self,
        state: State,
        control_input: &Self::ControlInput,
    ) -> (f64, f64, f64, f64, f64, f64) {
        (0.0, 0.0, 9.80665, 0.0, 0.0, 0.0)
    }
}
