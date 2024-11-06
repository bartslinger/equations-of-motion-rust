#[derive(Copy, Clone, Debug)]
pub struct State {
    u: f64,
    v: f64,
    w: f64,
    p: f64,
    q: f64,
    r: f64,
    q0: f64,
    q1: f64,
    q2: f64,
    q3: f64,
    pn: f64,
    pe: f64,
    pd: f64,
}

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
        Self {
            mass,
            inertia: 10.0,
            dynamics_model,
            state: State {
                u: 0.0,
                v: 0.0,
                w: 0.0,
                p: 0.0,
                q: 0.0,
                r: 0.0,
                q0: 1.0,
                q1: 0.0,
                q2: 0.0,
                q3: 0.0,
                pn: 0.0,
                pe: 0.0,
                pd: 0.0,
            },
        }
    }
    pub fn step(&mut self, control_input: &M::ControlInput, dt: f64) -> State {
        let output = self
            .dynamics_model
            .compute_forces_and_moments(self.state, control_input);
        self.state
    }
}
