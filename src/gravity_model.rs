use crate::rigid_body::{DynamicsModel, Forces, Moments, State};

const ADDITIONAL_OUTPUTS: usize = 1;
pub struct GravityModel {}
impl DynamicsModel<ADDITIONAL_OUTPUTS> for GravityModel {
    type ControlInput = f64; // as test, control input is an upwards force acting against gravity
    fn output_names() -> [&'static str; ADDITIONAL_OUTPUTS] {
        ["upforce"]
    }

    fn compute_forces_and_moments(
        &self,
        _state: &State,
        rotation_matrix: &nalgebra::Matrix3<f64>,
        control_input: &Self::ControlInput,
    ) -> (Forces, Moments, nalgebra::SVector<f64, ADDITIONAL_OUTPUTS>) {
        let gravity =
            rotation_matrix.transpose() * nalgebra::Vector3::new(0.0, 0.0, 9.80665 - control_input);
        (
            gravity,
            nalgebra::Vector3::zeros(),
            nalgebra::SVector::<f64, ADDITIONAL_OUTPUTS>::from([*control_input]),
        )
    }
}
