// src/stabilizer/angle.rs

//! # Angle PID Flight Stabilization Controller
//!
//! This module provides a direct no_std Rust translation of the PID stabilization from dRehmFlight
//! version 1.3, an Arduino-based PID flight controller, aimed at use in a single-threaded environment
//! for UAVs or similar vehicles. The translation has been done with the intent to keep the code as
//! close to the original as possible, allowing for easier verification and comparison by those familiar
//! with the Arduino implementation.
//!
//! ## Overview
//!
//! The code consists of functions that compute PID control commands based on sensor inputs and desired
//! flight parameters. Each function manages specific aspects of flight control—angle stabilization,
//! rate control, etc.—updating global mutable state that represents the system's outputs.
//!
//! ## Usage
//!
//! This module is intended for embedded systems where Rust's `no_std` environment is applicable.
//! It uses unsafe global variables to store state and outputs, reflecting the structure of the original
//! Arduino code. Each function is marked `unsafe` due to the direct manipulation of these global states,
//! which could lead to data races in a multithreaded context or other side effects due to improper use.
//!
//! ## Limitations and Concerns
//!
//! - **Global Mutable State:** The extensive use of `static mut` for storing system state is inherently
//!   unsafe and non-idiomatic in Rust. This approach was chosen to simplify the initial translation
//!   process but is prone to errors and issues in multi-threaded environments. Users must ensure that
//!   these functions are only accessed in a single-threaded context to prevent undefined behavior.
//!
//! - **Unsafe Code:** The reliance on unsafe code blocks is a significant concern for safety-critical
//!   applications typical of flight control systems. Future revisions should aim to encapsulate state
//!   management within safer abstractions and minimize or eliminate the use of `unsafe` by leveraging
//!   Rust's type and concurrency safety features.
//!
//! - **Flight Readiness:** The current architecture, heavily reliant on global state and unsafe practices,
//!   is not suitable for actual flight applications. It represents a fundamental mismatch with Rust's
//!   design principles, emphasizing safety and expressiveness through ownership and borrowing.
//!
//! ## Future Improvements
//!
//! Future improvements should focus on refactoring the code to embrace Rust's safety and concurrency
//! features, reducing the reliance on global mutable state, and implementing more robust error handling
//! and validation mechanisms. Additionally, exploring the use of Rust's async capabilities might provide
//! avenues for safer and more efficient concurrency models in multi-threaded or interrupt-driven contexts.

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
            dt: dt,
            integral_limit: self.i_limit,
            reset_integral: low_throttle,
        };
        let pitch_data = AngleControlData {
            measurement: imu_pitch,
            rate: gyro_pitch,
            dt: dt,
            integral_limit: self.i_limit,
            reset_integral: low_throttle,
        };

        // Prepare control data for yaw
        let yaw_data = RateControlData {
            rate: gyro_yaw,
            dt: dt,
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

// Time step for PID calculation, mutable to accommodate dynamic updates
static mut DT: f32 = 0.01; // Initial default value

// Default PID Coefficients
static DEFAULT_KP_ROLL_ANGLE: f32 = 0.2;
static DEFAULT_KI_ROLL_ANGLE: f32 = 0.3;
static DEFAULT_KD_ROLL_ANGLE: f32 = 0.05;
static DEFAULT_KP_PITCH_ANGLE: f32 = 0.2;
static DEFAULT_KI_PITCH_ANGLE: f32 = 0.3;
static DEFAULT_KD_PITCH_ANGLE: f32 = 0.05;
static DEFAULT_KP_YAW_RATE: f32 = 0.3;
static DEFAULT_KI_YAW_RATE: f32 = 0.05;
static DEFAULT_KD_YAW_RATE: f32 = 0.00015;

// Integrator limit
static DEFAULT_I_LIMIT: f32 = 25.0;

// Sensor Variables
static mut ROLL_IMU: f32 = 0.0;
static mut PITCH_IMU: f32 = 0.0;
static mut GYROX: f32 = 0.0;
static mut GYROY: f32 = 0.0;
static mut GYROZ: f32 = 0.0;

// Throttle Variables
static mut CHANNEL_1_PWM: u16 = 1000;
static LOW_THROTTLE_LIMIT: u16 = 1060;

// Set Points
static mut ROLL_DES: f32 = 0.0;
static mut PITCH_DES: f32 = 0.0;
static mut YAW_DES: f32 = 0.0;

// PID Output Variables
static mut ROLL_PID: f32 = 0.0;
static mut PITCH_PID: f32 = 0.0;
static mut YAW_PID: f32 = 0.0;
static OUTPUT_SCALE: f32 = 0.01;

// Previous state for errors
static mut ERROR_YAW_PREV: f32 = 0.0;

// Previous state for integrators
static mut INTEGRAL_ROLL_PREV: f32 = 0.0;
static mut INTEGRAL_PITCH_PREV: f32 = 0.0;
static mut INTEGRAL_YAW_PREV: f32 = 0.0;

/// Computes control commands based on state error (angle).
///
/// This function implements a basic PID control to stabilize on an angle
/// set_point derived from `ROLL_DES`, `PITCH_DES`, and `YAW_DES`, which
/// are computed in another part of the system (e.g., `getDesState()`).
/// It manages integral wind-up by resetting the integrators if the throttle
/// is below a threshold, ensuring the motors do not spool up when the
/// aircraft is on the ground. It updates the global variables `ROLL_PID`,
/// `PITCH_PID`, and `YAW_PID`, which represent the 1-D stabilized control
/// signals for roll, pitch, and yaw axes respectively. These outputs are
/// typically used for actuator control in a flight control system. The
/// function is marked `unsafe` due to its reliance on global mutable state,
/// requiring careful concurrency management in a multithreaded context.
pub unsafe fn control_angle() {
    // Roll
    let error_roll = ROLL_DES - ROLL_IMU;
    let mut integral_roll = INTEGRAL_ROLL_PREV + error_roll * DT;
    if CHANNEL_1_PWM < LOW_THROTTLE_LIMIT {
        // Avoid integrator buildup if low throttle
        integral_roll = 0.0;
    }
    integral_roll = integral_roll.clamp(-DEFAULT_I_LIMIT, DEFAULT_I_LIMIT); //Saturate integrator to prevent unsafe buildup
    let derivative_roll = GYROX;
    ROLL_PID = OUTPUT_SCALE
        * (DEFAULT_KP_ROLL_ANGLE * error_roll + DEFAULT_KI_ROLL_ANGLE * integral_roll
            - DEFAULT_KD_ROLL_ANGLE * derivative_roll);

    // Pitch
    let error_pitch = PITCH_DES - PITCH_IMU;
    let mut integral_pitch = INTEGRAL_PITCH_PREV + error_pitch * DT;
    if CHANNEL_1_PWM < LOW_THROTTLE_LIMIT {
        // Avoid integrator buildup if low throttle
        integral_pitch = 0.0;
    }
    integral_pitch = integral_pitch.clamp(-DEFAULT_I_LIMIT, DEFAULT_I_LIMIT); //Saturate integrator to prevent unsafe buildup
    let derivative_pitch = GYROY;
    PITCH_PID = OUTPUT_SCALE
        * (DEFAULT_KP_PITCH_ANGLE * error_pitch + DEFAULT_KI_PITCH_ANGLE * integral_pitch
            - DEFAULT_KD_PITCH_ANGLE * derivative_pitch);

    // Yaw, stablize on rate from GyroZ
    let error_yaw = YAW_DES - GYROZ;
    let mut integral_yaw = INTEGRAL_YAW_PREV + error_yaw * DT;
    if CHANNEL_1_PWM < LOW_THROTTLE_LIMIT {
        // Avoid integrator buildup if low throttle
        integral_yaw = 0.0;
    }
    integral_yaw = integral_yaw.clamp(-DEFAULT_I_LIMIT, DEFAULT_I_LIMIT); //Saturate integrator to prevent unsafe buildup
    let derivative_yaw = (error_yaw - ERROR_YAW_PREV) / DT;
    YAW_PID = OUTPUT_SCALE
        * (DEFAULT_KP_YAW_RATE * error_yaw
            + DEFAULT_KI_YAW_RATE * integral_yaw
            + DEFAULT_KD_YAW_RATE * derivative_yaw);

    // Update roll variables
    INTEGRAL_ROLL_PREV = integral_roll;

    // Update pitch variables
    INTEGRAL_PITCH_PREV = integral_pitch;

    // Update yaw variables
    INTEGRAL_YAW_PREV = integral_yaw;
    ERROR_YAW_PREV = error_yaw;
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_utils::*;

    fn legacy_reset_initial_conditions() {
        unsafe {
            // Time step for PID calculation
            DT = 0.01; // , _blending_configInitial default value

            // Sensor Variables
            ROLL_IMU = 0.0;
            PITCH_IMU = 0.0;
            GYROX = 0.0;
            GYROY = 0.0;
            GYROZ = 0.0;
            CHANNEL_1_PWM = 1000;

            // Set Points
            ROLL_DES = 0.0;
            PITCH_DES = 0.0;
            YAW_DES = 0.0;

            // PID Output Variables
            ROLL_PID = 0.0;
            PITCH_PID = 0.0;
            YAW_PID = 0.0;

            // Previous state for errors
            ERROR_YAW_PREV = 0.0;

            // Previous state for integrators
            INTEGRAL_ROLL_PREV = 0.0;
            INTEGRAL_PITCH_PREV = 0.0;
            INTEGRAL_YAW_PREV = 0.0;
        }
    }

    /// Test the no error contidion.
    #[test]
    fn legacy_test_stabilizer_angle_no_error() {
        unsafe {
            legacy_reset_initial_conditions();

            // Initialize global state
            CHANNEL_1_PWM = 1200; // Above the threshold to allow integrator to build
            ROLL_IMU = 10.0;
            PITCH_IMU = 10.0;
            GYROX = 0.0;
            GYROY = 0.0;
            GYROZ = 10.0;

            // Expected initial conditions
            ROLL_DES = ROLL_IMU;
            PITCH_DES = PITCH_IMU;
            YAW_DES = GYROZ;
            INTEGRAL_ROLL_PREV = 0.0;
            INTEGRAL_PITCH_PREV = 0.0;
            INTEGRAL_YAW_PREV = 0.0;

            // Call the function to test
            control_angle();

            // Check if the PID outputs are as expected
            assert!(
                value_close(0.0, ROLL_PID),
                "Roll PID should be zero as there is no error."
            );
            assert!(
                value_close(0.0, PITCH_PID),
                "Pitch PID should be zero as there is no error."
            );
            assert!(
                value_close(0.0, YAW_PID),
                "Yaw PID should be zero as there is no error."
            );
        }
    }

    /// Test to ensure integrators are reset when PWM is below threshold.
    #[test]
    fn legacy_test_stabilizer_angle_low_throttle_integrator_reset() {
        unsafe {
            legacy_reset_initial_conditions();

            // Initialize global state with some values
            CHANNEL_1_PWM = 1050; // Below the threshold to prevent integrator buildup

            // Assume some previous integrator values
            INTEGRAL_ROLL_PREV = 5.0;
            INTEGRAL_PITCH_PREV = 5.0;
            INTEGRAL_YAW_PREV = 5.0;

            // Call the function to test
            control_angle();

            // Verify that integrators are reset
            assert!(
                value_close(0.0, INTEGRAL_ROLL_PREV),
                "Roll integrator should be reset."
            );
            assert!(
                value_close(0.0, INTEGRAL_PITCH_PREV),
                "Pitch integrator should be reset."
            );
            assert!(
                value_close(0.0, INTEGRAL_YAW_PREV),
                "Yaw integrator should be reset."
            );
        }
    }

    /// Test the control_angle function with specific inputs to calculate expected PID outputs.
    #[test]
    fn legacy_test_stabilizer_angle_specific_pid_output() {
        unsafe {
            legacy_reset_initial_conditions();

            // Set specific test values
            CHANNEL_1_PWM = 1200; // Above the minimum threshold for integrator activity
            ROLL_IMU = 5.0;
            PITCH_IMU = 5.0;
            GYROX = 1.0; // Simulated rate of change in roll
            GYROY = -1.0; // Simulated rate of change in pitch
            GYROZ = -1.0; // Simulated rate of change in yaw

            // Desired angles are set to create a specific error
            ROLL_DES = 10.0; // Desired roll is 5 degrees greater than actual
            PITCH_DES = 0.0; // Desired pitch is 5 degrees less than actual
            YAW_DES = 10.0; // Set a desired yaw to generate a specific rate error

            // Set previous integrals
            INTEGRAL_ROLL_PREV = 0.2;
            INTEGRAL_PITCH_PREV = -0.2;
            INTEGRAL_YAW_PREV = 0.2;

            // Expected PID calculations
            let expected_roll_pid = OUTPUT_SCALE
                * (DEFAULT_KP_ROLL_ANGLE * (ROLL_DES - ROLL_IMU)
                    + DEFAULT_KI_ROLL_ANGLE * (INTEGRAL_ROLL_PREV + (ROLL_DES - ROLL_IMU) * DT)
                    - DEFAULT_KD_ROLL_ANGLE * GYROX);
            assert!(
                value_close(0.01025, expected_roll_pid),
                "Expected roll PID calcualted incorrectly."
            );
            let expected_pitch_pid = OUTPUT_SCALE
                * (DEFAULT_KP_PITCH_ANGLE * (PITCH_DES - PITCH_IMU)
                    + DEFAULT_KI_PITCH_ANGLE
                        * (INTEGRAL_PITCH_PREV + (PITCH_DES - PITCH_IMU) * DT)
                    - DEFAULT_KD_PITCH_ANGLE * GYROY);
            assert!(
                value_close(-0.01025, expected_pitch_pid),
                "Expected pitch PID calcualted incorrectly."
            );
            let expected_yaw_pid = OUTPUT_SCALE
                * (DEFAULT_KP_YAW_RATE * (YAW_DES - GYROZ)
                    + DEFAULT_KI_YAW_RATE * (INTEGRAL_YAW_PREV + (YAW_DES - GYROZ) * DT)
                    + DEFAULT_KD_YAW_RATE * ((YAW_DES - GYROZ) - ERROR_YAW_PREV) / DT);
            assert!(
                value_close(0.034805, expected_yaw_pid),
                "Expected yaw PID calcualted incorrectly."
            );

            // Execute the control function
            control_angle();

            // Validate the outputs
            assert_eq!(
                ROLL_PID, expected_roll_pid,
                "Roll PID output does not match expected value."
            );
            assert_eq!(
                PITCH_PID, expected_pitch_pid,
                "Pitch PID output does not match expected value."
            );
            assert_eq!(
                YAW_PID, expected_yaw_pid,
                "Yaw PID output does not match expected value."
            );
        }
    }

    /// Test that the integrator saturation works as expected by the DEFAULT_I_LIMIT.
    #[test]
    fn legacy_test_stabilizer_angle_integrator_saturation() {
        unsafe {
            legacy_reset_initial_conditions();

            // Setup test conditions
            CHANNEL_1_PWM = 1200; // Above the threshold to allow integrator buildup
            ROLL_IMU = 0.0;
            PITCH_IMU = 0.0;
            GYROX = 0.0;
            GYROY = 0.0;
            GYROZ = 0.0;

            // Set a high error that would normally cause the integrator to exceed DEFAULT_I_LIMIT
            ROLL_DES = 100.0;
            PITCH_DES = 100.0;
            YAW_DES = 100.0;

            // Call control_angle multiple times to simulate multiple time steps
            for _ in 0..100 {
                control_angle();
            }

            // Check if the integrators are clamped correctly
            assert!(
                INTEGRAL_ROLL_PREV.abs() <= DEFAULT_I_LIMIT,
                "Roll integrator exceeded DEFAULT_I_LIMIT: {}",
                INTEGRAL_ROLL_PREV
            );
            assert!(
                INTEGRAL_PITCH_PREV.abs() <= DEFAULT_I_LIMIT,
                "Pitch integrator exceeded DEFAULT_I_LIMIT: {}",
                INTEGRAL_PITCH_PREV
            );
            assert!(
                INTEGRAL_YAW_PREV.abs() <= DEFAULT_I_LIMIT,
                "Yaw integrator exceeded DEFAULT_I_LIMIT: {}",
                INTEGRAL_YAW_PREV
            );
        }
    }

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

    /// Test the control_angle function with specific inputs to calculate expected PID outputs.
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
