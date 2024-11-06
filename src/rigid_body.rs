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
    inertia: f64,
    dynamics_model: M,
    state: State,
}

impl<M: DynamicsModel> RigidBody<M> {
    pub fn new(mass: f64, dynamics_model: M) -> Self {
        let state = nalgebra::SVector::<f64, 13>::from([
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        ]);

        Self {
            mass,
            inertia: 10.0,
            dynamics_model,
            state,
        }
    }

    fn compute_state_derivative(
        &self,
        state: &State,
        control_input: &M::ControlInput,
    ) -> StateDerivative {
        let (Fx, Fy, Fz, Mx, My, Mz) = self
            .dynamics_model
            .compute_forces_and_moments(self.state, control_input);

        let w_dot = Fz / self.mass;

        let state_derivative: StateDerivative = nalgebra::SVector::<f64, 13>::from([
            0.0, 0.0, w_dot, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
