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
        state: State,
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
    pub fn new(dynamics_model: M, mass: f64, inertia: nalgebra::Matrix3<f64>) -> Self {
        let state = nalgebra::SVector::<f64, 13>::from([
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        ]);

        let inertia_inverse = inertia.try_inverse().unwrap();

        Self {
            mass,
            inertia,
            inertia_inverse,
            dynamics_model,
            state,
        }
    }

    fn compute_state_derivative(
        &self,
        state: &State,
        control_input: &M::ControlInput,
    ) -> StateDerivative {
        let (p, q, r) = (self.state[0], self.state[1], self.state[2]);
        let (u, v, w) = (self.state[3], self.state[4], self.state[5]);
        let (q0, q1, q2, q3) = (self.state[6], self.state[7], self.state[8], self.state[9]);
        let (pn, pe, pd) = (self.state[10], self.state[11], self.state[12]);

        let (force_x, force_y, force_z, moment_x, moment_y, moment_z) = self
            .dynamics_model
            .compute_forces_and_moments(self.state, control_input);
        let moments = nalgebra::Vector3::new(moment_x, moment_y, moment_z);

        let omega = nalgebra::Vector3::new(p, q, r);
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

        let state_derivative: StateDerivative = nalgebra::SVector::<f64, 13>::from([
            u_dot, v_dot, w_dot, p_dot, q_dot, r_dot, q0_dot, q1_dot, q2_dot, q3_dot, 0.0, 0.0, 0.0,
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

    pub fn step(&mut self, control_input: &M::ControlInput, dt: f64) -> State {
        self.state = self.runge_kutta_propagation(control_input, dt);
        self.state
    }
}
