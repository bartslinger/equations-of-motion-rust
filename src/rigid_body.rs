// #[derive(Copy, Clone, Debug)]
// pub struct State {
//     u: f64,
//     v: f64,
//     w: f64,
//     p: f64,
//     q: f64,
//     r: f64,
//     q0: f64,
//     q1: f64,
//     q2: f64,
//     q3: f64,
//     pn: f64,
//     pe: f64,
//     pd: f64,
// }

pub type State = nalgebra::SVector<f64, 13>;
pub type StateDerivative = nalgebra::SVector<f64, 13>;

pub trait DynamicsModel {
    type ControlInput;
    fn compute_forces_and_moments(
        &self,
        state: &State,
        rotation_matrix: &nalgebra::Matrix3<f64>,
        control_input: &Self::ControlInput,
    ) -> (f64, f64, f64, f64, f64, f64);
}

pub struct RigidBody<M: DynamicsModel> {
    mass: f64,
    inertia: nalgebra::Matrix3<f64>,
    inertia_inverse: nalgebra::Matrix3<f64>,
    dynamics_model: M,
    state: State,
}

impl<M: DynamicsModel> RigidBody<M> {
    pub fn new(
        dynamics_model: M,
        mass: f64,
        inertia: nalgebra::Matrix3<f64>,
        initial_state: State,
    ) -> Self {
        let inertia_inverse = inertia.try_inverse().unwrap();

        Self {
            mass,
            inertia,
            inertia_inverse,
            dynamics_model,
            state: initial_state,
        }
    }

    pub fn get_state(&self) -> State {
        self.state
    }

    fn compute_state_derivative(
        &self,
        state: &State,
        control_input: &M::ControlInput,
    ) -> StateDerivative {
        let (u, v, w) = (state[0], state[1], state[2]);
        let (p, q, r) = (state[3], state[4], state[5]);
        let (q0, q1, q2, q3) = (state[6], state[7], state[8], state[9]);
        // let (pn, pe, pd) = (state[10], state[11], state[12]);

        let velocity_body = nalgebra::Vector3::new(u, v, w);
        let omega = nalgebra::Vector3::new(p, q, r);

        // Construct the rotation matrix from quaternion
        let r11 = q0.powi(2) + q1.powi(2) - q2.powi(2) - q3.powi(2);
        let r22 = q0.powi(2) - q1.powi(2) + q2.powi(2) - q3.powi(2);
        let r33 = q0.powi(2) - q1.powi(2) - q2.powi(2) + q3.powi(2);
        let r12 = 2.0 * (q1 * q2 - q0 * q3);
        let r21 = 2.0 * (q1 * q2 + q0 * q3);
        let r13 = 2.0 * (q1 * q3 + q0 * q2);
        let r31 = 2.0 * (q1 * q3 - q0 * q2);
        let r23 = 2.0 * (q2 * q3 - q0 * q1);
        let r32 = 2.0 * (q2 * q3 + q0 * q1);
        let rotation_matrix =
            nalgebra::Matrix3::<f64>::new(r11, r12, r13, r21, r22, r23, r31, r32, r33);
        // let inverse_rotation_matrix = rotation_matrix.transpose();

        let velocity_ned = rotation_matrix * velocity_body;

        let (force_x, force_y, force_z, moment_x, moment_y, moment_z) = self
            .dynamics_model
            .compute_forces_and_moments(state, &rotation_matrix, control_input);
        let moments = nalgebra::Vector3::new(moment_x, moment_y, moment_z);

        let part1 = self.inertia * omega;
        let coriolis = omega.cross(&part1);
        let omega_dot = self.inertia_inverse * (moments - coriolis);

        let u_dot = force_x / self.mass - w * q + v * r;
        let v_dot = force_y / self.mass - u * r + w * p;
        let w_dot = force_z / self.mass - v * p + u * q;
        let p_dot = omega_dot[0];
        let q_dot = omega_dot[1];
        let r_dot = omega_dot[2];
        let q0_dot = -0.5 * (q1 * p + q2 * q + q3 * r);
        let q1_dot = 0.5 * (q0 * p + q2 * r - q3 * q);
        let q2_dot = 0.5 * (q0 * q + q3 * p - q1 * r);
        let q3_dot = 0.5 * (q0 * r + q1 * q - q2 * p);
        let pn_dot = velocity_ned[0];
        let pe_dot = velocity_ned[1];
        let pd_dot = velocity_ned[2];

        let state_derivative: StateDerivative = nalgebra::SVector::<f64, 13>::from([
            u_dot, v_dot, w_dot, p_dot, q_dot, r_dot, q0_dot, q1_dot, q2_dot, q3_dot, pn_dot,
            pe_dot, pd_dot,
        ]);
        state_derivative
    }

    fn runge_kutta_propagation(&self, control_input: &M::ControlInput, dt: f64) -> State {
        let k1 = self.compute_state_derivative(&self.state, control_input);
        let k2 = self.compute_state_derivative(&(self.state + k1 * dt / 2.0), control_input);
        let k3 = self.compute_state_derivative(&(self.state + k2 * dt / 2.0), control_input);
        let k4 = self.compute_state_derivative(&(self.state + k3 * dt), control_input);
        self.state + (k1 + 2.0 * k2 + 2.0 * k3 + k4) * dt / 6.0
    }

    fn normalize_quaternion(&mut self) {
        let q = nalgebra::Vector4::new(self.state[6], self.state[7], self.state[8], self.state[9]);
        let q_normalized = q.normalize();
        self.state[6] = q_normalized[0];
        self.state[7] = q_normalized[1];
        self.state[8] = q_normalized[2];
        self.state[9] = q_normalized[3];
    }

    pub fn step(&mut self, control_input: &M::ControlInput, dt: f64) {
        self.state = self.runge_kutta_propagation(control_input, dt);
        self.normalize_quaternion();
    }
}
