/// Single dimension linear kalman filter.
pub struct Kalman1D<Sf: Fn(f32) -> f32, Pf: Fn(f32) -> f32> {
    /// The previous updated state estimate
    x_n1: f32,
    /// The previous state estimates variance
    p_n_n1: f32,
    /// State prediction function
    state_est_fn: Sf,
    /// Variance prediction function
    var_est_fn: Pf,
}

impl<Sf: Fn(f32) -> f32, Pf: Fn(f32) -> f32> Kalman1D<Sf, Pf> {
    /// Initialises a kalman filter.
    ///
    /// # Args
    /// - state_est: guess at initial state
    /// - est_uncertainty: variance of your initial state guess. Filter will converge slower if this is large
    /// - state_model: dynamic model of your state, given previous state. Should predict where the state will be after one filter time-step. For a system that doesn't change, this is just identity.
    /// For something with motion, this will be kinematics.
    /// - var_model: models the variance of your state model, given previous variance. For constant dynamics, this is `prev + q`, where q is the variance of your estimate.
    pub fn new(state_est: f32, est_uncertainty: f32, state_model: Sf, var_model: Pf) -> Self {
        Self {
            x_n1: state_est,
            p_n_n1: est_uncertainty,
            state_est_fn: state_model,
            var_est_fn: var_model,
        }
    }

    /// Estimates the current state of the system given a measurement.
    ///
    /// This function will perform both the predict and update steps of a filter.
    ///
    /// # Args
    /// - z: measured value
    /// - r: measurement variance
    pub fn filter(&mut self, z: f32, r: f32) -> f32 {
        /* Prediction */

        // Predict state based off our model
        let pred_x = (self.state_est_fn)(self.x_n1);
        // Accumulate variance due to model uncertainty
        let pred_p = (self.var_est_fn)(self.p_n_n1);

        /* Update */

        // Weight predictions more if measurements have a higher uncertainty
        let gain = pred_p / (pred_p + r);
        // Refine estimate based off of the measurement
        self.x_n1 = pred_x + gain * (z - pred_x);
        // Lower our uncertainty depending on the certainty of the measurement
        self.p_n_n1 = (1.0 - gain) * pred_p;

        self.x_n1
    }
}

#[cfg(test)]
mod test {
    extern crate std;

    use std::prelude::rust_2021::*;
    use std::io::Read;

    use crate::kalman::Kalman1D;

    #[test]
    fn test() {
        let mut filter = Kalman1D::new(2000.0, 67.4f32.powi(2), |x| x, |v| v + 0.001);

        let mut f = std::fs::File::open("test_assets/data.txt").unwrap();
        let mut str = String::new();
        f.read_to_string(&mut str).unwrap();

        for val in str.split(", ").flat_map(|s| s.parse::<u16>()) {
            let est = filter.filter(val as f32, 191.93f32.powi(2));
            std::println!("{:.0}", est);
            std::println!("filter uncertainty: {}", filter.p_n_n1)
        }
    }
}