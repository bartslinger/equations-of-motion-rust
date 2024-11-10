use crate::rigid_body::{DynamicsModel, Forces, Moments, State};

const ADDITIONAL_OUTPUTS: usize = 4;

pub struct ModelParameters {
    pub mass: f64,
    pub inertia: nalgebra::Matrix3<f64>,
    pub spring1_xb: f64,
    pub spring1_yb: f64,
    pub spring1_zb: f64,
    pub spring2_xb: f64,
    pub spring2_yb: f64,
    pub spring2_zb: f64,
    pub spring1_xw: f64,
    pub spring1_yw: f64,
    pub spring1_zw: f64,
    pub spring2_xw: f64,
    pub spring2_yw: f64,
    pub spring2_zw: f64,
    pub spring1_length: f64,
    pub spring2_length: f64,
    pub spring_kp: f64,
    pub spring_kd: f64,
}

impl Default for ModelParameters {
    fn default() -> Self {
        Self {
            mass: 0.35,
            inertia: nalgebra::Matrix3::new(0.001, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01),
            spring1_xb: 0.06,
            spring1_yb: 0.07 / 2.0,
            spring1_zb: 0.0,
            spring2_xb: 0.06,
            spring2_yb: -0.07 / 2.0,
            spring2_zb: 0.0,
            spring1_xw: 0.0,
            spring1_yw: 0.315 / 2.0,
            spring1_zw: -3.0,
            spring2_xw: 0.0,
            spring2_yw: -0.315 / 2.0,
            spring2_zw: -3.0,
            spring1_length: 1.65,
            spring2_length: 1.65,
            spring_kp: 800.0,
            spring_kd: 22.0,
        }
    }
}

pub struct BifilarPendulumModel {
    pub params: ModelParameters,
}
impl DynamicsModel<ADDITIONAL_OUTPUTS> for BifilarPendulumModel {
    type ControlInput = (); // as test, control input is an upwards force acting against gravity

    fn mass(&self) -> f64 {
        self.params.mass
    }

    fn inertia(&self) -> nalgebra::Matrix3<f64> {
        self.params.inertia
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
        let gravity_b = r_bi * self.mass() * nalgebra::Vector3::new(0.0, 0.0, 9.80665);

        // we try first with a super strong damped spring, instead of a rigid string
        // so the dynamics of the spring should be much faster than the expected oscillation period
        // of 1.8s

        let r_attach1_b = nalgebra::Vector3::new(
            self.params.spring1_xb,
            self.params.spring1_yb,
            self.params.spring1_zb,
        );
        let r_attach2_b = nalgebra::Vector3::new(
            self.params.spring2_xb,
            self.params.spring2_yb,
            self.params.spring2_zb,
        );
        // let r_attach1_b = nalgebra::Vector3::new(0.06, 0.0, Y_ATTACH);
        // let r_attach2_b = nalgebra::Vector3::new(0.06, 0.0, -Y_ATTACH);

        // the string is attached to the north/east in the NED frame
        // the distance between the two strings is 31.5cm
        let r_origin1_ned = nalgebra::Vector3::new(
            self.params.spring1_xw,
            self.params.spring1_yw,
            self.params.spring1_zw,
        );
        let r_origin2_ned = nalgebra::Vector3::new(
            self.params.spring2_xw,
            self.params.spring2_yw,
            self.params.spring2_zw,
        );

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

        let spring1_force = -self.params.spring_kp
            * (self.params.spring1_length - r_spring1_b.norm())
            - self.params.spring_kd * v_spring1_b;
        let spring2_force = -self.params.spring_kp
            * (self.params.spring2_length - r_spring2_b.norm())
            - self.params.spring_kd * v_spring2_b;

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
