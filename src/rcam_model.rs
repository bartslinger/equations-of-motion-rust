use crate::rigid_body::{DynamicsModel, Forces, Moments, State};
use std::f64::consts::PI;

const OUTPUTS: usize = 1;

pub struct RcamModel {}

impl DynamicsModel<OUTPUTS> for RcamModel {
    // https://github.com/clum/YouTube/blob/85e5e4e2c4815ee8e4893faabf2935f936ee2649/Controls28/RCAM_model.m
    type ControlInput = nalgebra::Vector5<f64>;
    fn mass() -> f64 {
        120_000.0
    }

    fn inertia() -> nalgebra::Matrix3<f64> {
        RcamModel::mass()
            * nalgebra::Matrix3::new(40.07, 0.0, -2.0923, 0.0, 64.0, 0.0, -2.0923, 0.0, 99.92)
    }

    fn output_names() -> [&'static str; OUTPUTS] {
        ["alpha"]
    }

    fn compute_forces_and_moments(
        &self,
        state: &State,
        rotation_matrix: &nalgebra::Matrix3<f64>,
        control_input: &Self::ControlInput,
    ) -> (Forces, Moments, nalgebra::SVector<f64, OUTPUTS>) {
        // Constants
        let cbar = 6.6; // Mean Aerodynamic Chord (m)
        let lt = 24.8; // Distance by AC of tail and body (m)
        let s_wing = 260.0; // Wing planform area (m^2)
        let s_tail = 64.0; // Tail planform area (m^2)
        let x_cg = 0.23 * cbar; // x position of CoG in Fm (m)
        let y_cg = 0.0; // y position of CoG in Fm (m)
        let z_cg = 0.10 * cbar; // z position of CoG in Fm (m)

        let x_ac = 0.12 * cbar; // x position of aerodynamic center in Fm (m)
        let y_ac = 0.0; // y position of aerodynamic center in Fm (m)
        let z_ac = 0.0; // z position of aerodynamic center in Fm (m)

        // Engine inputs
        let u_max = 120000.0 * 9.81; // maximum thrust provided by one engine (N)

        let x_apt1 = 0.0; // x position of engine 1 force in Fm (m)
        let y_apt1 = -7.94; // y position of engine 1 force in Fm (m)
        let z_apt1 = -1.9; // z position of engine 1 force in Fm (m)

        let x_apt2 = 0.0; // x position of engine 2 force in Fm (m)
        let y_apt2 = 7.94; // y position of engine 2 force in Fm (m)
        let z_apt2 = -1.9; // z position of engine 2 force in Fm (m)

        // Other constants
        let rho = 1.225; // Air density (kg/m^3)
        let g = 9.81; // Gravitational acceleration (m/s^2)
        let depsda = 0.25; // Change in downwash w.r.t. alpha (rad/rad)
        let alpha_l0 = -11.5 * PI / 180.0; // Zero lift angle of attack (rad)
        let n = 5.5; // Slope of linear region of lift slope
        let a3 = -768.5; // Coefficient of alpha^3
        let a2 = 609.2; // Coefficient of alpha^2
        let a1 = -155.2; // Coefficient of alpha^1
        let a0 = 15.2; // Coefficient of alpha^0
        let alpha_switch = 14.5 * (PI / 180.0); // alpha where lift slope goes from linear to non-linear

        let (u, v, w) = (state[0], state[1], state[2]);
        let (p, q, r) = (state[3], state[4], state[5]);
        let velocity_body = nalgebra::Vector3::new(u, v, w);
        let omega_body = nalgebra::Vector3::new(p, q, r);

        let u1 = control_input[0]; // d_A (aileron)
        let u2 = control_input[1]; // d_T (stabilizer)
        let u3 = control_input[2]; // d_R (rudder)
        let u4 = control_input[3]; // d_th1 (throttle 1)
        let u5 = control_input[4]; // d_th2 (throttle 2)

        // Calculate intermediate variables
        let airspeed = velocity_body.norm();
        let alpha = w.atan2(u);
        let beta = (v / airspeed).asin();

        // dynamic pressure
        let dynamic_pressure = 0.5 * rho * airspeed.powi(2);

        // Calculate the CL_wb
        // if alpha<=alpha_switch
        // CL_wb = n*(alpha - alpha_L0);
        // else
        // CL_wb = a3*alpha^3 + a2*alpha^2 + a1*alpha + a0;
        // end
        let cl_wb = if alpha <= alpha_switch {
            n * (alpha - alpha_l0)
        } else {
            a3 * alpha.powi(3) + a2 * alpha.powi(2) + a1 * alpha + a0
        };

        // Calculate CL_t
        let epsilon = depsda * (alpha - alpha_l0);
        let alpha_t = alpha - epsilon + u2 + 1.3 * q * lt / airspeed;
        let cl_t = 3.1 * (s_tail / s_wing) * alpha_t;

        // Total lift force
        let cl = cl_wb + cl_t;

        // Total drag force (neglecting tail)
        let cd = 0.13 + 0.07 * (5.5 * alpha + 0.654).powi(2);

        // Calculate sideforce
        let cy = -1.6 * beta + 0.24 * u3;

        // --------------DIMENSIONAL AERODYNAMIC FORCES---------------------
        // Calculate the actual dimensional forces.  These are in F_s (stability axis)
        let fa_s = nalgebra::Vector3::new(
            -cd * dynamic_pressure * s_wing,
            cy * dynamic_pressure * s_wing,
            -cl * dynamic_pressure * s_wing,
        );

        // Rotate these forces to F_b (body axis)
        let r_bs = nalgebra::Matrix3::new(
            alpha.cos(),
            0.0,
            -(alpha).sin(),
            0.0,
            1.0,
            0.0,
            alpha.sin(),
            0.0,
            alpha.cos(),
        );
        let fa_b = r_bs * fa_s;
        //

        // --------------AERODYNAMIC MOMENT ABOUT AC-------------------
        // Calculate the moments in Fb.  Define eta, dCMdx and dCMdu
        let eta11 = -1.4 * beta;
        let eta21 = -0.59 - (3.1 * (s_tail * lt) / (s_wing * cbar)) * (alpha - epsilon);
        let eta31 = (1.0 - alpha * (180.0 / (15.0 * PI))) * beta;

        let eta = nalgebra::Vector3::new(eta11, eta21, eta31);

        let d_cm_domega = (cbar / airspeed)
            * nalgebra::Matrix3::new(
                -11.0,
                0.0,
                5.0,
                0.0,
                -4.03 * (s_tail * lt.powi(2)) / (s_wing * cbar.powi(2)),
                0.0,
                1.7,
                0.0,
                -11.5,
            );

        let d_cm_du = nalgebra::Matrix3::new(
            -0.6,
            0.0,
            0.22,
            0.0,
            -3.1 * (s_tail * lt) / (s_wing * cbar),
            0.0,
            0.0,
            0.0,
            -0.63,
        );

        // Now calculate CM = [Cl;Cm;Cn] about Aerodynamic center in Fb
        let cm_ac_b = eta + d_cm_domega * omega_body + d_cm_du * nalgebra::Vector3::new(u1, u2, u3);

        // Normalize to an aerodynamic moment
        let m_ac_b = cm_ac_b * dynamic_pressure * s_wing * cbar;

        // OPTIONAL: Covert this to stability axis
        let r_sb = r_bs.transpose();
        let m_ac_s = r_sb * m_ac_b;

        // --------------AERODYNAMIC MOMENT ABOUT CG-------------------
        // Transfer moment to cg
        let rcg_b = nalgebra::Vector3::new(x_cg, y_cg, z_cg);
        let rac_b = nalgebra::Vector3::new(x_ac, y_ac, z_ac);
        let ma_cg_b = r_bs * m_ac_s + fa_b.cross(&(rcg_b - rac_b));

        // -----------------ENGINE FORCE & MOMENT----------------------------
        // Now effect of engine.  First, calculate the thrust of each engine
        let f1 = u4 * u_max;
        let f2 = u5 * u_max;

        // Assuming that engine thrust is aligned with Fb, we have
        let fe1_b = nalgebra::Vector3::new(f1, 0.0, 0.0);
        let fe2_b = nalgebra::Vector3::new(f2, 0.0, 0.0);

        let fe_b = fe1_b + fe2_b;

        // Now engine moment due to offset of engine thrust from CoG.
        // Note: This is weird because the sign of y-axis seems to be flipped
        let mew1 = nalgebra::Vector3::new(x_cg - x_apt1, y_apt1 - y_cg, z_cg - z_apt1);
        let mew2 = nalgebra::Vector3::new(x_cg - x_apt2, y_apt2 - y_cg, z_cg - z_apt2);

        let me_cg1_b = mew1.cross(&fe1_b);
        let me_cg2_b = mew2.cross(&fe2_b);

        let me_cg_b = me_cg1_b + me_cg2_b;

        let g_ned = nalgebra::Vector3::new(0.0, 0.0, g);
        let g_b = rotation_matrix.transpose() * g_ned;
        let fg_b = Self::mass() * g_b;

        let f_b = fg_b + fe_b + fa_b;
        let m_cg_b = ma_cg_b + me_cg_b;

        (f_b, m_cg_b, nalgebra::SVector::<f64, OUTPUTS>::new(alpha))
    }
}
