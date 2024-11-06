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

pub type Forces = nalgebra::Vector3<f64>;
pub type Moments = nalgebra::Vector3<f64>;

pub trait DynamicsModel<const O: usize> {
    type ControlInput;

    fn output_names() -> [&'static str; O];
    fn compute_forces_and_moments(
        &self,
        state: &State,
        rotation_matrix: &nalgebra::Matrix3<f64>,
        control_input: &Self::ControlInput,
    ) -> (Forces, Moments, nalgebra::SVector<f64, O>);
}

pub struct RigidBody<M, const O: usize>
where
    M: DynamicsModel<O>,
{
    mass: f64,
    inertia: nalgebra::Matrix3<f64>,
    inertia_inverse: nalgebra::Matrix3<f64>,
    dynamics_model: M,
    output_file: Option<csv::Writer<std::fs::File>>,
}

impl<M, const O: usize> RigidBody<M, O>
where
    M: DynamicsModel<O>,
{
    pub fn new(dynamics_model: M, mass: f64, inertia: nalgebra::Matrix3<f64>) -> Self {
        let inertia_inverse = inertia.try_inverse().unwrap();

        Self {
            mass,
            inertia,
            inertia_inverse,
            dynamics_model,
            output_file: None,
        }
    }

    pub fn set_output_file(&mut self, file_name: &str) {
        if self.output_file.is_some() {
            panic!("Output file is already set");
        }
        let mut csv_writer = csv::Writer::from_path(file_name).unwrap();
        let additional_outputs = M::output_names();
        let mut headers = vec![
            "time", "u", "v", "w", "p", "q", "r", "q0", "q1", "q2", "q3", "pn", "pe", "pd", "Fx",
            "Fy", "Fz", "Mx", "My", "Mz",
        ];
        headers.extend(additional_outputs.iter());
        csv_writer.write_record(&headers).unwrap();
        self.output_file = Some(csv_writer);
    }

    fn write_csv_line(
        &mut self,
        t: u128,
        state: State,
        forces: Forces,
        moments: Moments,
        additional_outputs: nalgebra::SVector<f64, O>,
    ) {
        if let Some(ref mut csv_writer) = &mut self.output_file {
            // modify this with additional outputs
            let original_record = [
                (t as f64 * 0.001),
                state[0],
                state[1],
                state[2],
                state[3],
                state[4],
                state[5],
                state[6],
                state[7],
                state[8],
                state[9],
                state[10],
                state[11],
                state[12],
                forces[0],
                forces[1],
                forces[2],
                moments[0],
                moments[1],
                moments[2],
            ];
            let record = original_record
                .iter()
                .chain(additional_outputs.iter())
                .map(|x| x.to_string())
                .collect::<Vec<String>>();
            csv_writer.write_record(record).unwrap();
        }
    }

    // pub fn get_state(&self) -> State {
    //     self.state
    // }

    fn compute_state_derivative(
        &self,
        state: &State,
        control_input: &M::ControlInput,
    ) -> (StateDerivative, Forces, Moments, nalgebra::SVector<f64, O>) {
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

        let (forces, moments, additional_outputs) =
            self.dynamics_model
                .compute_forces_and_moments(state, &rotation_matrix, control_input);

        let part1 = self.inertia * omega;
        let coriolis = omega.cross(&part1);
        let omega_dot = self.inertia_inverse * (moments - coriolis);

        let u_dot = forces[0] / self.mass - w * q + v * r;
        let v_dot = forces[1] / self.mass - u * r + w * p;
        let w_dot = forces[2] / self.mass - v * p + u * q;
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
        (state_derivative, forces, moments, additional_outputs)
    }

    fn runge_kutta_propagation(
        &self,
        state: &State,
        control_input: &M::ControlInput,
        dt: f64,
    ) -> (State, Forces, Moments, nalgebra::SVector<f64, O>) {
        let (k1, f1, m1, a1) = self.compute_state_derivative(state, control_input);
        let (k2, f2, m2, a2) =
            self.compute_state_derivative(&(state + k1 * dt / 2.0), control_input);
        let (k3, f3, m3, a3) =
            self.compute_state_derivative(&(state + k2 * dt / 2.0), control_input);
        let (k4, f4, m4, a4) = self.compute_state_derivative(&(state + k3 * dt), control_input);
        let next_state = state + (k1 + 2.0 * k2 + 2.0 * k3 + k4) * dt / 6.0;
        let forces = (f1 + 2.0 * f2 + 2.0 * f3 + f4) / 6.0;
        let moments = (m1 + 2.0 * m2 + 2.0 * m3 + m4) / 6.0;
        let additional_outputs = (a1 + 2.0 * a2 + 2.0 * a3 + a4) / 6.0;
        (next_state, forces, moments, additional_outputs)
    }

    pub fn step(
        &mut self,
        state: &State,
        control_input: &M::ControlInput,
        dt: f64,
    ) -> (State, Forces, Moments, nalgebra::SVector<f64, O>) {
        let (state, forces, moments, additional_outputs) =
            self.runge_kutta_propagation(state, control_input, dt);
        let state = normalize_quaternion(state);
        (state, forces, moments, additional_outputs)
    }

    pub fn simulate<F>(
        &mut self,
        duration: std::time::Duration,
        dt: std::time::Duration,
        control_input: F,
    ) where
        F: Fn(u128) -> M::ControlInput,
    {
        let initial_state = nalgebra::SVector::<f64, 13>::from([
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        ]);

        let steps = duration.as_millis() / dt.as_millis();
        let (mut state, mut forces, mut moments, mut additional_outputs) =
            self.runge_kutta_propagation(&initial_state, &control_input(0), dt.as_secs_f64());
        for i in 0..=steps {
            let t = i * dt.as_millis();
            self.write_csv_line(t, state, forces, moments, additional_outputs);
            let u = control_input(t);
            (state, forces, moments, additional_outputs) = self.step(&state, &u, dt.as_secs_f64());
        }
    }
}

fn normalize_quaternion(mut state: State) -> State {
    let q = nalgebra::Vector4::new(state[6], state[7], state[8], state[9]);
    let q_normalized = q.normalize();
    state[6] = q_normalized[0];
    state[7] = q_normalized[1];
    state[8] = q_normalized[2];
    state[9] = q_normalized[3];
    state
}
