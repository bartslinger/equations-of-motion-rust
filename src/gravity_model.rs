use crate::rigid_body::{DynamicsModel, Forces, Moments, State};

const INPUTS: usize = 1;
const ADDITIONAL_OUTPUTS: usize = 2;
pub struct GravityModel {}
impl DynamicsModel<INPUTS, ADDITIONAL_OUTPUTS> for GravityModel {
    fn mass(&self) -> f64 {
        1.0
    }

    fn inertia(&self) -> nalgebra::Matrix3<f64> {
        nalgebra::Matrix3::new(0.0135, 0.0, 0.0, 0.0, 0.0010, 0.0, 0.0, 0.0, 0.0142)
    }

    fn output_names() -> [&'static str; ADDITIONAL_OUTPUTS] {
        ["control_input", "upforce"]
    }

    fn compute_forces_and_moments(
        &self,
        _state: &State,
        rotation_matrix: &nalgebra::Matrix3<f64>,
        control_input: &nalgebra::SVector<f64, INPUTS>,
    ) -> (Forces, Moments, nalgebra::SVector<f64, ADDITIONAL_OUTPUTS>) {
        let upforce_ned = nalgebra::Vector3::new(0.0, 0.0, -control_input[0]);
        let upforce = rotation_matrix.transpose() * upforce_ned;

        let gravity = rotation_matrix.transpose() * nalgebra::Vector3::new(0.0, 0.0, 9.80665);
        (
            gravity + upforce,
            nalgebra::Vector3::zeros(),
            nalgebra::SVector::<f64, ADDITIONAL_OUTPUTS>::from([control_input[0], upforce.z]),
        )
    }
}
