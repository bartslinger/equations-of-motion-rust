use crate::rigid_body::RigidBody;

mod gravity_model;
mod rigid_body;

fn main() {
    let gravity_model = gravity_model::GravityModel {};
    let mass = 1.0;
    let inertia = nalgebra::Matrix3::new(0.0135, 0.0, 0.0, 0.0, 0.0010, 0.0, 0.0, 0.0, 0.0142);
    let mut body = RigidBody::new(gravity_model, mass, inertia);

    let mut csv_writer = csv::Writer::from_path("output.csv").unwrap();
    csv_writer
        .write_record(&[
            "time", "u", "v", "w", "p", "q", "r", "q0", "q1", "q2", "q3", "pn", "pe", "pd",
        ])
        .unwrap();

    let dt = 0.01;
    for i in 0..=200 {
        let state = body.get_state();
        csv_writer
            .write_record(&[
                (i as f64 * dt).to_string(),
                state[0].to_string(),
                state[1].to_string(),
                state[2].to_string(),
                state[3].to_string(),
                state[4].to_string(),
                state[5].to_string(),
                state[6].to_string(),
                state[7].to_string(),
                state[8].to_string(),
                state[9].to_string(),
                state[10].to_string(),
                state[11].to_string(),
                state[12].to_string(),
            ])
            .unwrap();
        body.step(&(), dt);
    }
    let next_state = body.step(&(), 0.01);
    println!("{:?}", next_state);
}
