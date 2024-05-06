// src/stabilizer/angle.rs

//! # Angle-Based PID Flight Stabilization Controller
//!
//! This is an angle-based PID flight stabilization controller.
//! Note that yaw relies on rate-based stabilization.

use crate::pid::{compute_angle, compute_rate, AngleControlData, RateControlData};
use crate::{FlightStabilizer, FlightStabilizerConfig, Number};
use piddiy::PidController;

/// Struct representing the Angle PID Flight Stabilization Controller.
pub struct AngleStabilizer<T: Number> {
    roll_pid: PidController<T, AngleControlData<T>>,
    pitch_pid: PidController<T, AngleControlData<T>>,
    yaw_pid: PidController<T, RateControlData<T>>,
    i_limit: T,
    scale: T,
}

impl<T: Number> Default for AngleStabilizer<T> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T: Number> AngleStabilizer<T> {
    /// Creates a new controller using the provided configuration
    pub fn with_config(config: FlightStabilizerConfig<T>) -> Self {
        let mut roll_pid = PidController::new();
        roll_pid
            .compute_fn(compute_angle)
            .set_point(config.set_point_roll)
            .kp(config.kp_roll)
            .ki(config.ki_roll)
            .kd(config.kd_roll);

        let mut pitch_pid = PidController::new();
        pitch_pid
            .compute_fn(compute_angle)
            .set_point(config.set_point_pitch)
            .kp(config.kp_pitch)
            .ki(config.ki_pitch)
            .kd(config.kd_pitch);

        let mut yaw_pid = PidController::new();
        yaw_pid
            .compute_fn(compute_rate)
            .set_point(config.set_point_yaw)
            .kp(config.kp_yaw)
            .ki(config.ki_yaw)
            .kd(config.kd_yaw);

        AngleStabilizer {
            roll_pid,
            pitch_pid,
            yaw_pid,
            i_limit: config.i_limit,
            scale: config.scale,
        }
    }

    /// Creates a new controller with default settings
    pub fn new() -> Self {
        Self::with_config(FlightStabilizerConfig::new())
    }
}

impl<T: Number> FlightStabilizer<T> for AngleStabilizer<T> {
    fn control(
        &mut self,
        set_point: (T, T, T),
        imu_attitude: (T, T, T),
        gyro_rate: (T, T, T),
        dt: T,
        low_throttle: bool,
    ) -> (T, T, T) {
        // Set the setpoints for roll, pitch, and yaw
        let (set_point_roll, set_point_pitch, set_point_yaw) = set_point;
        self.roll_pid.set_point(set_point_roll);
        self.pitch_pid.set_point(set_point_pitch);
        self.yaw_pid.set_point(set_point_yaw);

        // Prepare control data for roll and pitch
        let (imu_roll, imu_pitch, _) = imu_attitude;
        let (gyro_roll, gyro_pitch, gyro_yaw) = gyro_rate;
        let roll_data = AngleControlData {
            measurement: imu_roll,
            rate: gyro_roll,
            dt,
            integral_limit: self.i_limit,
            reset_integral: low_throttle,
        };
        let pitch_data = AngleControlData {
            measurement: imu_pitch,
            rate: gyro_pitch,
            dt,
            integral_limit: self.i_limit,
            reset_integral: low_throttle,
        };

        // Prepare control data for yaw
        let yaw_data = RateControlData {
            rate: gyro_yaw,
            dt,
            integral_limit: self.i_limit,
            reset_integral: low_throttle,
        };

        // Compute outputs for roll, pitch, and yaw
        let roll_output = self.scale * self.roll_pid.compute(roll_data);
        let pitch_output = self.scale * self.pitch_pid.compute(pitch_data);
        let yaw_output = self.scale * self.yaw_pid.compute(yaw_data);

        (roll_output, pitch_output, yaw_output)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_utils::*;

    /// Default test configuration.
    fn default_config() -> FlightStabilizerConfig<f32> {
        let mut config = FlightStabilizerConfig::<f32>::new();

        // Set the PID gains for roll, pitch, and yaw.
        config.kp_roll = 0.2;
        config.ki_roll = 0.3;
        config.kd_roll = -0.05;

        config.kp_pitch = config.kp_roll;
        config.ki_pitch = config.ki_roll;
        config.kd_pitch = config.kd_roll;

        config.kp_yaw = 0.3;
        config.ki_yaw = 0.05;
        config.kd_yaw = 0.00015;

        // Set the initial setpoints for roll, pitch, and yaw.
        // These default to zero.
        config.set_point_roll = 0.0;
        config.set_point_pitch = 0.0;
        config.set_point_yaw = 0.0;

        // Set the upper limit for the integral term to prevent windup.
        config.i_limit = 25.0;

        // Set the scale to adjust the PID outputs to the actuator range.
        config.scale = 0.01;

        config
    }

    /// Test the initialization of the AngleStabilizer with a default configuration.
    #[test]
    fn test_stabilizer_angle_initialization_with_default_config() {
        let config = default_config();
        let stabilizer = AngleStabilizer::with_config(config);

        assert_eq!(stabilizer.roll_pid.kp, config.kp_roll);
        assert_eq!(stabilizer.pitch_pid.kp, config.kp_pitch);
        assert_eq!(stabilizer.yaw_pid.kp, config.kp_yaw);
    }

    /// Test that the integrator saturation works as expected by the DEFAULT_I_LIMIT.
    #[test]
    fn test_stabilizer_angle_integrator_saturation() {
        let config = default_config();
        let mut stabilizer = AngleStabilizer::with_config(config);

        // Simulated sensor inputs and desired setpoints
        let set_point = (100.0, -100.0, 50.0); // desired roll, pitch, yaw
        let imu_attitude = (0.0, 0.0, 0.0); // current roll, pitch, yaw
        let gyro_rate = (0.0, 0.0, 0.0); // current roll rate, pitch rate, yaw rate
        let dt = 0.01; // time step
        let low_throttle = false;

        // Apply consistent error over multiple cycles to force integrator saturation
        for _ in 0..100 {
            let _ = stabilizer.control(set_point, imu_attitude, gyro_rate, dt, low_throttle);
        }

        let integrals = (
            stabilizer.roll_pid.integral,
            stabilizer.pitch_pid.integral,
            stabilizer.yaw_pid.integral,
        );
        let expected_integrals = (config.i_limit, -config.i_limit, config.i_limit);
        assert!(
            vector_close(expected_integrals, integrals),
            "Integrals should be capped."
        );
    }

    /// Test to ensure integrators are reset when PWM is below threshold.
    #[test]
    fn test_stabilizer_angle_low_throttle_integral_reset() {
        let config = default_config();
        let mut stabilizer = AngleStabilizer::with_config(config);

        // Simulated sensor inputs and desired setpoints
        let set_point = (10.0, 0.0, 10.0); // desired roll, pitch, yaw
        let imu_attitude = (5.0, 5.0, 0.0); // current roll, pitch, yaw
        let gyro_rate = (1.0, -1.0, -1.0); // current roll rate, pitch rate, yaw rate
        let dt = 0.01; // time step

        // Allow integrators to build up
        let _ = stabilizer.control(set_point, imu_attitude, gyro_rate, dt, false);
        let integrals = (
            stabilizer.roll_pid.integral,
            stabilizer.pitch_pid.integral,
            stabilizer.yaw_pid.integral,
        );
        let unexpected_integrals = (0.0, 0.0, 0.0);
        assert!(
            vector_not_close(unexpected_integrals, integrals),
            "Integrals should not be zero."
        );

        // Apply low throttle, which should reset integrators
        let _ = stabilizer.control(set_point, imu_attitude, gyro_rate, dt, true);
        let integrals = (
            stabilizer.roll_pid.integral,
            stabilizer.pitch_pid.integral,
            stabilizer.yaw_pid.integral,
        );
        let expected_integrals = (0.0, 0.0, 0.0);
        assert!(
            vector_close(expected_integrals, integrals),
            "Integrals should be zero."
        );
    }

    /// Test the no error contidion.
    #[test]
    fn test_stabilizer_angle_no_error() {
        let config = default_config();
        let mut stabilizer = AngleStabilizer::with_config(config);

        // Simulated sensor inputs and desired setpoints
        let set_point = (0.0, 0.0, 0.0); // desired roll, pitch, yaw
        let imu_attitude = (0.0, 0.0, 0.0); // current roll, pitch, yaw
        let gyro_rate = (0.0, 0.0, 0.0); // current roll rate, pitch rate, yaw rate
        let dt = 0.01; // time step
        let low_throttle = false;

        // Perform the control computation
        let output = stabilizer.control(set_point, imu_attitude, gyro_rate, dt, low_throttle);
        let expected_output = (0.0, 0.0, 0.0);

        assert!(
            vector_close(expected_output, output),
            "Outputs should be zero as there is no error."
        );
    }

    /// Test with specific inputs to calculate expected PID outputs.
    #[test]
    fn test_stabilizer_angle_specific_pid_output() {
        // Set up stabilizer
        let config = default_config();
        let mut stabilizer = AngleStabilizer::with_config(config);
        stabilizer.roll_pid.integral = 0.2; // previous integral
        stabilizer.pitch_pid.integral = -0.2; // previous integral
        stabilizer.yaw_pid.integral = 0.2; // previous integral

        // Simulated sensor inputs and desired setpoints
        let set_point = (10.0, 0.0, 10.0); // desired roll, pitch, yaw
        let imu_attitude = (5.0, 5.0, 0.0); // current roll, pitch, yaw
        let gyro_rate = (1.0, -1.0, -1.0); // current roll rate, pitch rate, yaw rate
        let dt = 0.01; // time step
        let low_throttle = false;

        // Perform the control computation
        let output = stabilizer.control(set_point, imu_attitude, gyro_rate, dt, low_throttle);
        let expected_output = (0.01025, -0.01025, 0.034805);

        assert!(
            vector_close(expected_output, output),
            "PID outputs should match specific values."
        );
    }
}
