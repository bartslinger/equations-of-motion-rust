use crate::rigid_body::{DynamicsModel, State};

pub struct GravityModel {}
impl DynamicsModel for GravityModel {
    type ControlInput = ();
    fn compute_forces_and_moments(
        &self,
        _state: &State,
        rotation_matrix: &nalgebra::Matrix3<f64>,
        _control_input: &Self::ControlInput,
    ) -> (f64, f64, f64, f64, f64, f64) {
        let gravity = rotation_matrix.transpose() * nalgebra::Vector3::new(0.0, 0.0, 9.80665);
        (gravity[0], gravity[1], gravity[2], 0.0, 0.0, 0.0)
    }
}
