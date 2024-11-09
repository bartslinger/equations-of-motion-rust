use crate::rigid_body::{DynamicsModel, Forces, Moments, State};

const ADDITIONAL_OUTPUTS: usize = 4;
pub struct BifilarPendulumModel {}
impl DynamicsModel<ADDITIONAL_OUTPUTS> for BifilarPendulumModel {
    type ControlInput = (); // as test, control input is an upwards force acting against gravity

    fn mass() -> f64 {
        0.35
    }

    fn inertia() -> nalgebra::Matrix3<f64> {
        // measurement on Ixx on the talon 250
        // let's see if we can reproduce the oscillation period of 1.83 s
        // let ratio = 0.004393121676710427 / 40.07;
        // ratio * nalgebra::Matrix3::new(40.07, 0.0, -2.0923, 0.0, 64.0, 0.0, -2.0923, 0.0, 99.92)
        nalgebra::Matrix3::new(
            // 0.004393121676710427,
            0.001, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 100.0,
        )
    }

    fn output_names() -> [&'static str; ADDITIONAL_OUTPUTS] {
        ["L_spring1", "L_spring2", "F_spring1", "F_spring2"]
    }

    fn compute_forces_and_moments(
        &self,
        state: &State,
        rotation_matrix: &nalgebra::Matrix3<f64>,
        _control_input: &Self::ControlInput,
    ) -> (Forces, Moments, nalgebra::SVector<f64, ADDITIONAL_OUTPUTS>) {
        let r_cg_ned = nalgebra::Vector3::new(state[10], state[11], state[12]);
        let v_b = nalgebra::Vector3::new(state[0], state[1], state[2]);
        let r_ib = rotation_matrix;
        let r_bi = r_ib.transpose();
        let gravity_b = r_bi * Self::mass() * nalgebra::Vector3::new(0.0, 0.0, 9.80665);

        // we try first with a super strong damped spring, instead of a rigid string
        // so the dynamics of the spring should be much faster than the expected oscillation period
        // of 1.8s

        // the string is attached to the body 7cm in front of CG and 3.5cm to the left/right
        const Y_ATTACH: f64 = 0.07 / 2.0;
        // const Y_ATTACH: f64 = 0.315 / 2.0;
        let r_attach1_b = nalgebra::Vector3::new(0.06, Y_ATTACH, 0.0);
        let r_attach2_b = nalgebra::Vector3::new(0.06, -Y_ATTACH, 0.0);
        // let r_attach1_b = nalgebra::Vector3::new(0.06, 0.0, Y_ATTACH);
        // let r_attach2_b = nalgebra::Vector3::new(0.06, 0.0, -Y_ATTACH);

        // the string is attached to the north/east in the NED frame
        // the distance between the two strings is 31.5cm
        const E_ATTACH: f64 = 0.315 / 2.0;
        let r_origin1_ned = nalgebra::Vector3::new(0.0, E_ATTACH, 0.0);
        let r_origin2_ned = nalgebra::Vector3::new(0.0, -E_ATTACH, 0.0);

        let r_origin1_cg_ned = r_origin1_ned - r_cg_ned;
        let r_origin2_cg_ned = r_origin2_ned - r_cg_ned;

        let r_origin1_cg_b = r_bi * r_origin1_cg_ned;
        let r_origin2_cg_b = r_bi * r_origin2_cg_ned;

        // calculate the direction and length of the spring
        let r_spring1_b = r_origin1_cg_b - r_attach1_b;
        let r_spring2_b = r_origin2_cg_b - r_attach2_b;

        // calculate the body velocity projected in the direction of the spring
        let v_spring1_b = v_b.dot(&r_spring1_b.normalize());
        let v_spring2_b = v_b.dot(&r_spring2_b.normalize());

        let kp = 800.0;
        let kd = 22.0;
        const L_SPRING: f64 = 1.65;
        let spring1_force = -kp * (L_SPRING - r_spring1_b.norm()) - kd * v_spring1_b;
        let spring2_force = -kp * (L_SPRING - r_spring2_b.norm()) - kd * v_spring2_b;

        let f_spring1_b = r_spring1_b.normalize() * spring1_force;
        let f_spring2_b = r_spring2_b.normalize() * spring2_force;

        // calculate the moments from the spring forces
        let m_spring1_b = r_attach1_b.cross(&f_spring1_b);
        let m_spring2_b = r_attach2_b.cross(&f_spring2_b);

        // let m_damp = nalgebra::Vector3::new(-0.001 * state[3], 0.0, 0.0);

        (
            gravity_b + f_spring1_b + f_spring2_b,
            nalgebra::Vector3::zeros() + m_spring1_b + m_spring2_b,
            nalgebra::SVector::<f64, ADDITIONAL_OUTPUTS>::new(
                r_spring1_b.norm(),
                r_spring2_b.norm(),
                spring1_force,
                spring2_force,
            ),
        )
    }
}
