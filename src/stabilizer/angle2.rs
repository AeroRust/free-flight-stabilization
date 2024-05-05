// src/stabilizer/angle2.rs

//! # Angle2 Cascade PID Flight Stabilization Controller
//!
//! This is a angle and rate-based cascade syle PID flight control.
//! Note that yaw relies on rate-based stabilization.
//! It requires three configuration files- one of the angle-based PID,
//! one for the rate-based PID, and one for the blending.

use crate::pid::{compute_cascade_angle, compute_rate, CascadeAngleControlData, RateControlData};
use crate::{CascadeBlendingConfig, FlightStabilizer, FlightStabilizerConfig, Number};
use piddiy::PidController;

/// Struct representing the Angle2 PID Flight Stabilization Controller.
/// This is a cascade PID controller that combines angle and rate.
pub struct Angle2Stabilizer<T: Number> {
    angle_roll_pid: PidController<T, CascadeAngleControlData<T>>,
    angle_pitch_pid: PidController<T, CascadeAngleControlData<T>>,
    angle_i_limit: T,
    angle_scale: T,
    rate_roll_pid: PidController<T, RateControlData<T>>,
    rate_pitch_pid: PidController<T, RateControlData<T>>,
    rate_yaw_pid: PidController<T, RateControlData<T>>,
    rate_i_limit: T,
    rate_scale: T,
    kl: T,             // blending factor for angle and rate
    beta: T,           // blending factor for angle and rate
    blending_limit: T, // blending factor for angle and rate
    prev_set_point_roll: T,
    prev_set_point_pitch: T,
    prev_imu_roll: T,
    prev_imu_pitch: T,
}

impl<T: Number> Angle2Stabilizer<T> {
    /// Creates a new controller using the provided configuration
    pub fn with_config(
        angle_config: FlightStabilizerConfig<T>,
        rate_config: FlightStabilizerConfig<T>,
        blending_config: CascadeBlendingConfig<T>,
    ) -> Self {
        let mut angle_roll_pid = PidController::new();
        angle_roll_pid
            .compute_fn(compute_cascade_angle)
            .set_point(angle_config.set_point_roll)
            .kp(angle_config.kp_roll)
            .ki(angle_config.ki_roll)
            .kd(angle_config.kd_roll);

        let mut angle_pitch_pid = PidController::new();
        angle_pitch_pid
            .compute_fn(compute_cascade_angle)
            .set_point(angle_config.set_point_pitch)
            .kp(angle_config.kp_pitch)
            .ki(angle_config.ki_pitch)
            .kd(angle_config.kd_pitch);

        let mut rate_roll_pid = PidController::new();
        rate_roll_pid
            .compute_fn(compute_rate)
            .set_point(rate_config.set_point_roll)
            .kp(rate_config.kp_roll)
            .ki(rate_config.ki_roll)
            .kd(rate_config.kd_roll);

        let mut rate_pitch_pid = PidController::new();
        rate_pitch_pid
            .compute_fn(compute_rate)
            .set_point(rate_config.set_point_pitch)
            .kp(rate_config.kp_pitch)
            .ki(rate_config.ki_pitch)
            .kd(rate_config.kd_pitch);

        let mut rate_yaw_pid = PidController::new();
        rate_yaw_pid
            .compute_fn(compute_rate)
            .set_point(rate_config.set_point_yaw)
            .kp(rate_config.kp_yaw)
            .ki(rate_config.ki_yaw)
            .kd(rate_config.kd_yaw);

        Angle2Stabilizer {
            angle_roll_pid,
            angle_pitch_pid,
            angle_i_limit: angle_config.i_limit,
            angle_scale: angle_config.scale,
            rate_roll_pid,
            rate_pitch_pid,
            rate_yaw_pid,
            rate_i_limit: rate_config.i_limit,
            rate_scale: rate_config.scale,
            kl: blending_config.k,
            beta: blending_config.beta,
            blending_limit: blending_config.limit,
            prev_set_point_roll: angle_config.set_point_roll,
            prev_set_point_pitch: angle_config.set_point_pitch,
            prev_imu_roll: T::zero(),
            prev_imu_pitch: T::zero(),
        }
    }

    /// Creates a new controller with default settings
    pub fn new() -> Self {
        Self::with_config(
            FlightStabilizerConfig::new(),
            FlightStabilizerConfig::new(),
            CascadeBlendingConfig::new(),
        )
    }

    fn blend(&self, set_point: T, prev_set_point: T) -> T {
        let result = (set_point * self.kl).clamp(-self.blending_limit, self.blending_limit);
        self.beta * result + (T::one() - self.beta) * prev_set_point
    }
}

impl<T: Number> FlightStabilizer<T> for Angle2Stabilizer<T> {
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
        self.angle_roll_pid.set_point(set_point_roll);
        self.angle_pitch_pid.set_point(set_point_pitch);

        // Prepare control data for roll and pitch
        let (imu_roll, imu_pitch, _) = imu_attitude;
        let (gyro_roll, gyro_pitch, gyro_yaw) = gyro_rate;
        let angle_roll_data = CascadeAngleControlData {
            measurement: imu_roll,
            prev_measurement: self.prev_imu_roll,
            rate: gyro_roll,
            dt: dt,
            integral_limit: self.angle_i_limit,
            reset_integral: low_throttle,
        };
        let angle_pitch_data = CascadeAngleControlData {
            measurement: imu_pitch,
            prev_measurement: self.prev_imu_pitch,
            rate: gyro_pitch,
            dt: dt,
            integral_limit: self.angle_i_limit,
            reset_integral: low_throttle,
        };

        // Compute outputs for roll, pitch, and yaw
        let mut adjusted_set_point_roll =
            self.angle_scale * self.angle_roll_pid.compute(angle_roll_data);
        let mut adjusted_set_point_pitch =
            self.angle_scale * self.angle_pitch_pid.compute(angle_pitch_data);

        //Apply blending gain, clamp, and LP filter for artificial damping
        adjusted_set_point_roll = self.blend(adjusted_set_point_roll, self.prev_set_point_roll);
        adjusted_set_point_pitch = self.blend(adjusted_set_point_pitch, self.prev_set_point_pitch);

        // Set the rate set points for roll, pitch, and yaw
        self.rate_roll_pid.set_point(adjusted_set_point_roll);
        self.rate_pitch_pid.set_point(adjusted_set_point_pitch);
        self.rate_yaw_pid.set_point(set_point_yaw);

        // Prepare rate control data for roll, pitch, and yaw
        let rate_roll_data = RateControlData {
            rate: gyro_roll,
            dt: dt,
            integral_limit: self.rate_i_limit,
            reset_integral: low_throttle,
        };
        let rate_pitch_data = RateControlData {
            rate: gyro_pitch,
            dt: dt,
            integral_limit: self.rate_i_limit,
            reset_integral: low_throttle,
        };
        let rate_yaw_data = RateControlData {
            rate: gyro_yaw,
            dt: dt,
            integral_limit: self.rate_i_limit,
            reset_integral: low_throttle,
        };
        // Compute outputs for roll, pitch, and yaw
        let roll_output = self.rate_scale * self.rate_roll_pid.compute(rate_roll_data);
        let pitch_output = self.rate_scale * self.rate_pitch_pid.compute(rate_pitch_data);
        let yaw_output = self.rate_scale * self.rate_yaw_pid.compute(rate_yaw_data);

        // Store prevous values
        self.prev_set_point_roll = adjusted_set_point_roll;
        self.prev_set_point_pitch = adjusted_set_point_pitch;
        self.prev_imu_roll = imu_roll;
        self.prev_imu_pitch = imu_pitch;

        (roll_output, pitch_output, yaw_output)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_utils::*;

    /// Default test configuration.
    fn default_config() -> (
        FlightStabilizerConfig<f32>,
        FlightStabilizerConfig<f32>,
        CascadeBlendingConfig<f32>,
    ) {
        // Define angle config with shared values
        let mut angle_config = FlightStabilizerConfig::new();

        // Set the angle PID gains for roll, pitch, and yaw.
        angle_config.kp_roll = 0.2;
        angle_config.ki_roll = 0.3;
        angle_config.kd_roll = 0.0; // zero out angle derivative

        angle_config.kp_pitch = angle_config.kp_roll;
        angle_config.ki_pitch = angle_config.ki_roll;
        angle_config.kd_pitch = angle_config.kd_roll;

        angle_config.kp_yaw = 0.3;
        angle_config.ki_yaw = 0.05;
        angle_config.kd_yaw = 0.00015;

        // Do not scale the initial angle PID output.
        angle_config.scale = 1.0;

        // Set the shared initial setpoints for roll, pitch, and yaw.
        // These default to zero.
        angle_config.set_point_roll = 0.0;
        angle_config.set_point_pitch = 0.0;
        angle_config.set_point_yaw = 0.0;

        // Set the shared upper limit for the integral term to prevent windup.
        angle_config.i_limit = 25.0;

        // Copy shared values for rate config
        let mut rate_config = angle_config;

        // Set the rate PID gains for roll, pitch, and yaw.
        rate_config.kp_roll = 0.15;
        rate_config.ki_roll = 0.2;
        rate_config.kd_roll = 0.0002;

        rate_config.kp_pitch = rate_config.kp_roll;
        rate_config.ki_pitch = rate_config.ki_roll;
        rate_config.kd_pitch = rate_config.kd_roll;

        rate_config.kp_yaw = 0.3;
        rate_config.ki_yaw = 0.05;
        rate_config.kd_yaw = 0.00015;

        // Set the final scale to adjust the PID outputs to the actuator range.
        rate_config.scale = 0.01;

        // define blending config
        let mut blending_config = CascadeBlendingConfig::new();

        blending_config.beta = 0.9;
        blending_config.k = 30.0;
        blending_config.limit = 240.0;

        (angle_config, rate_config, blending_config)
    }

    /// Test the initialization of the Angle2Stabilizer with a default configuration.
    #[test]
    fn test_stabilizer_angle2_initialization_with_default_config() {
        let (angle_config, rate_config, blending_config) = default_config();
        let stabilizer = Angle2Stabilizer::with_config(angle_config, rate_config, blending_config);

        assert_eq!(stabilizer.angle_roll_pid.kp, angle_config.kp_roll);
        assert_eq!(stabilizer.angle_pitch_pid.kp, angle_config.kp_pitch);
        assert_eq!(stabilizer.rate_roll_pid.kp, rate_config.kp_roll);
        assert_eq!(stabilizer.rate_pitch_pid.kp, rate_config.kp_pitch);
        assert_eq!(stabilizer.rate_yaw_pid.kp, rate_config.kp_yaw);
        assert_eq!(stabilizer.beta, blending_config.beta);
    }

    /// Test that the integrator saturation works as expected by the DEFAULT_I_LIMIT.
    #[test]
    fn test_stabilizer_angle2_integrator_saturation() {
        let (angle_config, rate_config, blending_config) = default_config();
        let mut stabilizer =
            Angle2Stabilizer::with_config(angle_config, rate_config, blending_config);

        // Simulated sensor inputs and desired setpoints
        let set_point = (500.0, -500.0, 50.0); // desired roll, pitch, yaw
        let imu_attitude = (0.0, 0.0, 0.0); // current roll, pitch, yaw
        let gyro_rate = (0.0, 0.0, 0.0); // current roll rate, pitch rate, yaw rate
        let dt = 0.01; // time step
        let low_throttle = false;

        // Apply consistent error over multiple cycles to force integrator saturation
        for _ in 0..100 {
            let _ = stabilizer.control(set_point, imu_attitude, gyro_rate, dt, low_throttle);
        }

        let angle_integrals = (
            stabilizer.angle_roll_pid.integral,
            stabilizer.angle_pitch_pid.integral,
            0.0,
        );
        let expected_angle_integrals = (angle_config.i_limit, -angle_config.i_limit, 0.0);
        assert!(
            vector_close(expected_angle_integrals, angle_integrals),
            "Integrals should be capped."
        );
        let rate_integrals = (
            stabilizer.rate_roll_pid.integral,
            stabilizer.rate_pitch_pid.integral,
            stabilizer.rate_yaw_pid.integral,
        );
        let expected_rate_integrals = (
            rate_config.i_limit,
            -rate_config.i_limit,
            rate_config.i_limit,
        );
        assert!(
            vector_close(expected_rate_integrals, rate_integrals),
            "Integrals should be capped."
        );
    }

    /// Test to ensure integrators are reset when PWM is below threshold.
    #[test]
    fn test_stabilizer_angle2_low_throttle_integrator_reset() {
        let (angle_config, rate_config, blending_config) = default_config();
        let mut stabilizer =
            Angle2Stabilizer::with_config(angle_config, rate_config, blending_config);

        // Simulated sensor inputs and desired setpoints
        let set_point = (10.0, 0.0, 10.0); // desired roll, pitch, yaw
        let imu_attitude = (5.0, 5.0, 0.0); // current roll, pitch, yaw
        let gyro_rate = (1.0, -1.0, -1.0); // current roll rate, pitch rate, yaw rate
        let dt = 0.01; // time step

        // Allow integrators to build up
        let _ = stabilizer.control(set_point, imu_attitude, gyro_rate, dt, false);
        let angle_integrals = (
            stabilizer.angle_roll_pid.integral,
            stabilizer.angle_pitch_pid.integral,
            1.0, // not calculated
        );
        let unexpected_angle_integrals = (0.0, 0.0, 0.0);
        assert!(
            vector_not_close(unexpected_angle_integrals, angle_integrals),
            "Integrals should not be zero."
        );
        let rate_integrals = (
            stabilizer.rate_roll_pid.integral,
            stabilizer.rate_pitch_pid.integral,
            stabilizer.rate_yaw_pid.integral,
        );
        let unexpected_rate_integrals = (0.0, 0.0, 0.0);
        assert!(
            vector_not_close(unexpected_rate_integrals, rate_integrals),
            "Integrals should not be zero."
        );

        // Apply low throttle, which should reset integrators
        let _ = stabilizer.control(set_point, imu_attitude, gyro_rate, dt, true);
        let angle_integrals = (
            stabilizer.angle_roll_pid.integral,
            stabilizer.angle_pitch_pid.integral,
            0.0, // not calculated
        );
        let expected_angle_integrals = (0.0, 0.0, 0.0);
        assert!(
            vector_close(expected_angle_integrals, angle_integrals),
            "Integrals should be zero."
        );
        let rate_integrals = (
            stabilizer.rate_roll_pid.integral,
            stabilizer.rate_pitch_pid.integral,
            stabilizer.rate_yaw_pid.integral,
        );
        let expected_rate_integrals = (0.0, 0.0, 0.0);
        assert!(
            vector_close(expected_rate_integrals, rate_integrals),
            "Integrals should be zero."
        );
    }

    /// Test the no error contidion.
    #[test]
    fn test_stabilizer_angle2_no_error() {
        let (angle_config, rate_config, blending_config) = default_config();
        let mut stabilizer =
            Angle2Stabilizer::with_config(angle_config, rate_config, blending_config);

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
    fn test_stabilizer_angle2_specific_pid_output() {
        // Configuration
        const PREV_ANGLE_ROLL_INTEGRAL: f32 = 5.0;
        const PREV_ANGLE_PITCH_INTEGRAL: f32 = -5.0;
        const PREV_RATE_ROLL_INTEGRAL: f32 = 0.2;
        const PREV_RATE_PITCH_INTEGRAL: f32 = -0.2;
        const PREV_RATE_YAW_INTEGRAL: f32 = 0.2;
        const PREV_SET_POINT_ROLL: f32 = 50.0;
        const PREV_SET_POINT_PITCH: f32 = -50.0;
        let (angle_config, rate_config, blending_config) = default_config();

        // Simulated sensor inputs and desired setpoints
        let set_point = (10.0, 0.0, 10.0); // desired roll, pitch, yaw
        let imu_attitude = (5.0, 5.0, 0.0); // current roll, pitch, yaw
        let gyro_rate = (1.0, -1.0, -1.0); // current roll rate, pitch rate, yaw rate
        let dt = 0.01; // time step
        let low_throttle = false;

        let mut stabilizer =
            Angle2Stabilizer::with_config(angle_config, rate_config, blending_config);
        stabilizer.angle_roll_pid.integral = PREV_ANGLE_ROLL_INTEGRAL;
        stabilizer.angle_pitch_pid.integral = PREV_ANGLE_PITCH_INTEGRAL;
        stabilizer.rate_roll_pid.integral = PREV_RATE_ROLL_INTEGRAL;
        stabilizer.rate_pitch_pid.integral = PREV_RATE_PITCH_INTEGRAL;
        stabilizer.rate_yaw_pid.integral = PREV_RATE_YAW_INTEGRAL;
        stabilizer.prev_set_point_roll = PREV_SET_POINT_ROLL;
        stabilizer.prev_set_point_pitch = PREV_SET_POINT_PITCH;

        // Test the first part of the cascade.
        let angle_roll_data = CascadeAngleControlData {
            measurement: imu_attitude.0,
            prev_measurement: imu_attitude.0,
            rate: gyro_rate.0,
            dt: dt,
            integral_limit: angle_config.i_limit,
            reset_integral: low_throttle,
        };
        let angle_pitch_data = CascadeAngleControlData {
            measurement: imu_attitude.1,
            prev_measurement: imu_attitude.1,
            rate: gyro_rate.1,
            dt: dt,
            integral_limit: angle_config.i_limit,
            reset_integral: low_throttle,
        };

        // Compute the adjusted roll setpoint and internal values
        stabilizer.angle_roll_pid.set_point(set_point.0);
        let (roll_error, roll_integral, roll_derivative) = compute_cascade_angle(&mut stabilizer.angle_roll_pid, angle_roll_data);
        let mut adjusted_set_point_roll =
            angle_config.scale * (angle_config.kp_roll * roll_error + angle_config.ki_roll * roll_integral + angle_config.kd_roll * roll_derivative);
        adjusted_set_point_roll = stabilizer.blend(adjusted_set_point_roll, PREV_SET_POINT_ROLL);

        // Compute the adjusted pitch setpoint and internal values
        stabilizer.angle_pitch_pid.set_point(set_point.1);
        let (pitch_error, pitch_integral, pitch_derivative) = compute_cascade_angle(&mut stabilizer.angle_pitch_pid, angle_pitch_data);
        let mut adjusted_set_point_pitch =
            angle_config.scale * (angle_config.kp_pitch * pitch_error + angle_config.ki_pitch * pitch_integral + angle_config.kd_pitch * pitch_derivative);
        adjusted_set_point_pitch = stabilizer.blend(adjusted_set_point_pitch, PREV_SET_POINT_PITCH);

        // Internal values should be inverted
        assert_eq!(roll_error, -pitch_error);
        assert_eq!(roll_integral, -pitch_integral);
        assert_eq!(roll_derivative, -pitch_derivative);

        // Adjusted set points should match standard numbers
        let adjusted_set_points = (adjusted_set_point_roll, adjusted_set_point_pitch, 0.0);
        let expected_adjusted_set_points = (72.905, -72.905, 0.0);
        assert!(
            vector_close(expected_adjusted_set_points, adjusted_set_points),
            "Adjusted set points should match specific values."
        );

        // Perform the control computation
        let output = stabilizer.control(set_point, imu_attitude, gyro_rate, dt, low_throttle);
        let expected_output = (0.124076605, -0.124076605, 0.034805);
        assert!(
            vector_close(expected_output, output),
            "PID outputs should match specific values."
        );
    }
}
