// src/stabilizer/angle2.rs

//! # Angle2 PID-based Flight Stabilization Controller
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

// Time step for PID calculation, mutable to accommodate dynamic updates
static mut DT: f32 = 0.01; // Initial default value

// Integrator limit
const I_LIMIT: f32 = 25.0;

// PID Coefficients for Angle Control
static KP_ROLL_ANGLE: f32 = 0.2;
static KI_ROLL_ANGLE: f32 = 0.3;
//static KD_ROLL_ANGLE: f32 = 0.05;
static KP_PITCH_ANGLE: f32 = 0.2;
static KI_PITCH_ANGLE: f32 = 0.3;
//static KD_PITCH_ANGLE: f32 = 0.05;

// PID Coefficients for Rate Control
static KP_ROLL_RATE: f32 = 0.15;
static KI_ROLL_RATE: f32 = 0.2;
static KD_ROLL_RATE: f32 = 0.0002;
static KP_PITCH_RATE: f32 = 0.15;
static KI_PITCH_RATE: f32 = 0.2;
static KD_PITCH_RATE: f32 = 0.0002;
static KP_YAW_RATE: f32 = 0.3;
static KI_YAW_RATE: f32 = 0.05;
static KD_YAW_RATE: f32 = 0.00015;

// Loop Damping Factors
static B_LOOP_ROLL: f32 = 0.9;
static B_LOOP_PITCH: f32 = 0.9;

// Sensor Variables
static mut ROLL_IMU: f32 = 0.0;
static mut PITCH_IMU: f32 = 0.0;
static mut GYROX: f32 = 0.0;
static mut GYROY: f32 = 0.0;
static mut GYROZ: f32 = 0.0;
static mut CHANNEL_1_PWM: u16 = 1000;

// Set Points
static mut ROLL_DES: f32 = 0.0;
static mut PITCH_DES: f32 = 0.0;
static mut YAW_DES: f32 = 0.0;

// PID Output Variables
static mut ROLL_PID: f32 = 0.0;
static mut PITCH_PID: f32 = 0.0;
static mut YAW_PID: f32 = 0.0;

// Previous state for integrators
static mut ERROR_ROLL_PREV: f32 = 0.0;
static mut INTEGRAL_ROLL_OL_PREV: f32 = 0.0;
static mut INTEGRAL_ROLL_IL_PREV: f32 = 0.0;
static mut ROLL_IMU_PREV: f32 = 0.0;
static mut ROLL_DES_PREV: f32 = 0.0;
static mut ERROR_PITCH_PREV: f32 = 0.0;
static mut INTEGRAL_PITCH_OL_PREV: f32 = 0.0;
static mut INTEGRAL_PITCH_IL_PREV: f32 = 0.0;
static mut PITCH_IMU_PREV: f32 = 0.0;
static mut PITCH_DES_PREV: f32 = 0.0;
static mut INTEGRAL_YAW_PREV: f32 = 0.0;
static mut ERROR_YAW_PREV: f32 = 0.0;

/// Advanced PID control for angle stabilization with cascaded control scheme.
pub unsafe fn control_angle2() {
    // Outer loop calculations for roll
    let error_roll = ROLL_DES - ROLL_IMU;
    let mut integral_roll_ol = INTEGRAL_ROLL_OL_PREV + error_roll * DT;
    if CHANNEL_1_PWM < 1060 {
        integral_roll_ol = 0.0;
    }
    integral_roll_ol = integral_roll_ol.clamp(-I_LIMIT, I_LIMIT);
    //let derivative_roll = (ROLL_IMU - ROLL_IMU_PREV) / DT;
    let mut roll_des_ol = KP_ROLL_ANGLE * error_roll + KI_ROLL_ANGLE * integral_roll_ol;
    //    - KD_ROLL_ANGLE * derivative_roll;

    // Outer loop calculations for pitch
    let error_pitch = PITCH_DES - PITCH_IMU;
    let mut integral_pitch_ol = INTEGRAL_PITCH_OL_PREV + error_pitch * DT;
    if CHANNEL_1_PWM < 1060 {
        integral_pitch_ol = 0.0;
    }
    integral_pitch_ol = integral_pitch_ol.clamp(-I_LIMIT, I_LIMIT);
    //let derivative_pitch = (PITCH_IMU - PITCH_IMU_PREV) / DT;
    let mut pitch_des_ol = KP_PITCH_ANGLE * error_pitch + KI_PITCH_ANGLE * integral_pitch_ol;
    //    - KD_PITCH_ANGLE * derivative_pitch;

    // Apply loop gain and damping
    let kl = 30.0;
    roll_des_ol *= kl;
    roll_des_ol = roll_des_ol.clamp(-240.0, 240.0);
    roll_des_ol = (1.0 - B_LOOP_ROLL) * ROLL_DES_PREV + B_LOOP_ROLL * roll_des_ol;
    pitch_des_ol *= kl;
    pitch_des_ol = pitch_des_ol.clamp(-240.0, 240.0);
    pitch_des_ol = (1.0 - B_LOOP_PITCH) * PITCH_DES_PREV + B_LOOP_PITCH * pitch_des_ol;

    // Inner loop for roll rate
    let error_roll_il = roll_des_ol - GYROX;
    let mut integral_roll_il = INTEGRAL_ROLL_IL_PREV + error_roll_il * DT;
    if CHANNEL_1_PWM < 1060 {
        integral_roll_il = 0.0;
    }
    integral_roll_il = integral_roll_il.clamp(-I_LIMIT, I_LIMIT);
    let derivative_roll = (error_roll_il - ERROR_ROLL_PREV) / DT;
    ROLL_PID = 0.01
        * (KP_ROLL_RATE * error_roll_il
            + KI_ROLL_RATE * integral_roll_il
            + KD_ROLL_RATE * derivative_roll);

    // Inner loop for pitch rate
    let error_pitch_il = pitch_des_ol - GYROY;
    let mut integral_pitch_il = INTEGRAL_PITCH_IL_PREV + error_pitch_il * DT;
    if CHANNEL_1_PWM < 1060 {
        integral_pitch_il = 0.0;
    }
    integral_pitch_il = integral_pitch_il.clamp(-I_LIMIT, I_LIMIT);
    let derivative_pitch = (error_pitch_il - ERROR_PITCH_PREV) / DT;
    PITCH_PID = 0.01
        * (KP_PITCH_RATE * error_pitch_il
            + KI_PITCH_RATE * integral_pitch_il
            + KD_PITCH_RATE * derivative_pitch);

    // Yaw
    let error_yaw = YAW_DES - GYROZ;
    let mut integral_yaw = INTEGRAL_YAW_PREV + error_yaw * DT;
    if CHANNEL_1_PWM < 1060 {
        integral_yaw = 0.0;
    }
    integral_yaw = integral_yaw.clamp(-I_LIMIT, I_LIMIT);
    let derivative_yaw = (error_yaw - ERROR_YAW_PREV) / DT;
    YAW_PID = 0.01
        * (KP_YAW_RATE * error_yaw + KI_YAW_RATE * integral_yaw + KD_YAW_RATE * derivative_yaw);

    // Update roll global state
    INTEGRAL_ROLL_OL_PREV = integral_roll_ol;
    INTEGRAL_ROLL_IL_PREV = integral_roll_il;
    ERROR_ROLL_PREV = error_roll;
    ROLL_IMU_PREV = ROLL_IMU;
    ROLL_DES_PREV = roll_des_ol;

    // Update pitch global state
    INTEGRAL_PITCH_OL_PREV = integral_pitch_ol;
    INTEGRAL_PITCH_IL_PREV = integral_pitch_il;
    ERROR_PITCH_PREV = error_pitch;
    PITCH_IMU_PREV = PITCH_IMU;
    PITCH_DES_PREV = pitch_des_ol;

    // Update yaw global state
    ERROR_YAW_PREV = error_yaw;
    INTEGRAL_YAW_PREV = integral_yaw;
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_utils::*;
    use crate::FlightStabilizerConfig;

    fn reset_initial_conditions() {
        unsafe {
            // Time step for PID calculation
            DT = 0.01; // Initial default value

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

            // Previous state for integrators
            ERROR_ROLL_PREV = 0.0;
            INTEGRAL_ROLL_OL_PREV = 0.0;
            INTEGRAL_ROLL_IL_PREV = 0.0;
            ROLL_IMU_PREV = 0.0;
            ROLL_DES_PREV = 0.0;
            ERROR_PITCH_PREV = 0.0;
            INTEGRAL_PITCH_OL_PREV = 0.0;
            INTEGRAL_PITCH_IL_PREV = 0.0;
            PITCH_IMU_PREV = 0.0;
            PITCH_DES_PREV = 0.0;
            INTEGRAL_YAW_PREV = 0.0;
            ERROR_YAW_PREV = 0.0;
        }
    }

    /// Test the no error condition.
    #[test]
    fn test_control_angle2_no_error() {
        unsafe {
            reset_initial_conditions();

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
            INTEGRAL_ROLL_OL_PREV = 0.0;
            INTEGRAL_ROLL_IL_PREV = 0.0;
            INTEGRAL_PITCH_OL_PREV = 0.0;
            INTEGRAL_PITCH_IL_PREV = 0.0;
            INTEGRAL_YAW_PREV = 0.0;

            // Call the function to test
            control_angle2();

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
    fn test_control_angle2_low_throttle_integrator_reset() {
        unsafe {
            reset_initial_conditions();

            // Initialize global state with some values
            CHANNEL_1_PWM = 1050; // Below the threshold to prevent integrator buildup

            // Assume some previous integrator values
            INTEGRAL_ROLL_OL_PREV = 5.0;
            INTEGRAL_ROLL_IL_PREV = 5.0;
            INTEGRAL_PITCH_OL_PREV = 5.0;
            INTEGRAL_PITCH_IL_PREV = 5.0;
            INTEGRAL_YAW_PREV = 5.0;

            // Call the function to test
            control_angle2();

            // Verify that integrators are reset
            assert!(
                value_close(0.0, INTEGRAL_ROLL_OL_PREV),
                "Outer loop roll integrator should be reset."
            );
            assert!(
                value_close(0.0, INTEGRAL_ROLL_IL_PREV),
                "Inner loop oll integrator should be reset."
            );
            assert!(
                value_close(0.0, INTEGRAL_PITCH_OL_PREV),
                "Outer loop pitch integrator should be reset."
            );
            assert!(
                value_close(0.0, INTEGRAL_PITCH_IL_PREV),
                "Inner loop pitch integrator should be reset."
            );
            assert!(
                value_close(0.0, INTEGRAL_YAW_PREV),
                "Yaw integrator should be reset."
            );
        }
    }

    /// Test the control_angle function with specific inputs to calculate expected PID outputs.
    #[test]
    fn test_control_angle2_specific_pid_output() {
        unsafe {
            reset_initial_conditions();

            // Set specific test values
            CHANNEL_1_PWM = 1200; // Above the minimum threshold for integrator activity
            ROLL_IMU = 5.0;
            PITCH_IMU = 5.0;
            GYROX = 1.0; // Simulated rate of change in roll
            GYROY = -1.0; // Simulated rate of change in pitch
            GYROY = -1.0; // Simulated rate of change in yaw

            // Desired angles are set to create a specific error
            ROLL_DES = 10.0; // Desired roll is 5 degrees greater than actual
            PITCH_DES = 0.0; // Desired pitch is 5 degrees less than actual
            YAW_DES = 10.0; // Set a desired yaw to generate a specific rate error

            // Set initial integrals
            INTEGRAL_ROLL_OL_PREV = 5.0;
            INTEGRAL_ROLL_IL_PREV = 0.2;
            INTEGRAL_PITCH_OL_PREV = -5.0;
            INTEGRAL_PITCH_IL_PREV = -0.2;
            INTEGRAL_YAW_PREV = 0.2;

            // Outer loop calculates desired rate changes based on angle errors
            let roll_des_ol = (KP_ROLL_ANGLE * (ROLL_DES - ROLL_IMU)
                + KI_ROLL_ANGLE * (INTEGRAL_ROLL_OL_PREV + (ROLL_DES - ROLL_IMU) * DT))
                * 30.0
                * B_LOOP_ROLL;
            assert!(
                value_close(67.905, roll_des_ol),
                "Expected outer loop roll calculated incorrectly."
            );
            let pitch_des_ol = (KP_PITCH_ANGLE * (PITCH_DES - PITCH_IMU)
                + KI_PITCH_ANGLE * (INTEGRAL_PITCH_OL_PREV + (PITCH_DES - PITCH_IMU) * DT))
                * 30.0
                * B_LOOP_PITCH;
            assert!(
                value_close(-67.905, pitch_des_ol),
                "Expected outer loop pitch calculated incorrectly."
            );

            // Inner loop calculates the actual PID outputs based on the rate set by the outer loop
            let expected_roll_pid = 0.01
                * (KP_ROLL_RATE * (roll_des_ol - GYROX)
                    + KI_ROLL_RATE * (INTEGRAL_ROLL_IL_PREV + (roll_des_ol - GYROX) * DT)
                    + KD_ROLL_RATE * ((roll_des_ol - GYROX) - ERROR_ROLL_PREV) / DT);
            assert!(
                value_close(0.11547661, expected_roll_pid),
                "Expected roll PID calcualted incorrectly."
            );
            let expected_pitch_pid = 0.01
                * (KP_PITCH_RATE * (pitch_des_ol - GYROY)
                    + KI_PITCH_RATE * (INTEGRAL_PITCH_IL_PREV + (pitch_des_ol - GYROY) * DT)
                    + KD_PITCH_RATE * ((pitch_des_ol - GYROY - ERROR_PITCH_PREV) / DT));
            assert!(
                value_close(-0.11547661, expected_pitch_pid),
                "Expected pitch PID calcualted incorrectly."
            );
            let expected_yaw_pid = 0.01
                * (KP_YAW_RATE * (YAW_DES - GYROZ)
                    + KI_YAW_RATE * (INTEGRAL_YAW_PREV + (YAW_DES - GYROZ) * DT)
                    + KD_YAW_RATE * ((YAW_DES - GYROZ - ERROR_YAW_PREV) / DT));
            assert!(
                value_close(0.031650003, expected_yaw_pid),
                "Expected yaw PID calcualted incorrectly."
            );

            // Execute the control function
            control_angle2();

            // Validate the outputs
            assert!(
                value_close(ROLL_PID, expected_roll_pid),
                "Roll PID output does not match expected value."
            );
            assert!(
                value_close(PITCH_PID, expected_pitch_pid),
                "Pitch PID output does not match expected value."
            );
            assert!(
                value_close(YAW_PID, expected_yaw_pid),
                "Yaw PID output does not match expected value."
            );
        }
    }

    /// Test that the integrator saturation works as expected by the I_LIMIT.
    #[test]
    fn test_control_angle2_integrator_saturation() {
        unsafe {
            reset_initial_conditions();

            // Setup test conditions
            CHANNEL_1_PWM = 1200; // Above the threshold to allow integrator buildup
            ROLL_IMU = 0.0;
            PITCH_IMU = 0.0;
            GYROX = 0.0;
            GYROY = 0.0;
            GYROZ = 0.0;

            // Set a high error that would normally cause the integrator to exceed I_LIMIT
            ROLL_DES = 100.0;
            PITCH_DES = 100.0;
            YAW_DES = 100.0;

            // Call control_angle multiple times to simulate multiple time steps
            for _ in 0..100 {
                control_angle2();
            }

            // Check if the integrators are clamped correctly
            assert!(
                INTEGRAL_ROLL_OL_PREV.abs() <= I_LIMIT,
                "Roll integrator exceeded I_LIMIT: {}",
                INTEGRAL_ROLL_OL_PREV
            );
            assert!(
                INTEGRAL_ROLL_IL_PREV.abs() <= I_LIMIT,
                "Roll integrator exceeded I_LIMIT: {}",
                INTEGRAL_ROLL_IL_PREV
            );
            assert!(
                INTEGRAL_PITCH_OL_PREV.abs() <= I_LIMIT,
                "Pitch integrator exceeded I_LIMIT: {}",
                INTEGRAL_PITCH_OL_PREV
            );
            assert!(
                INTEGRAL_PITCH_IL_PREV.abs() <= I_LIMIT,
                "Pitch integrator exceeded I_LIMIT: {}",
                INTEGRAL_PITCH_IL_PREV
            );
            assert!(
                INTEGRAL_YAW_PREV.abs() <= I_LIMIT,
                "Yaw integrator exceeded I_LIMIT: {}",
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

        config.kp_pitch = 0.2;
        config.ki_pitch = 0.3;
        config.kd_pitch = -0.05;

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

    /// Test the no error contidion.
    #[test]
    fn test_control_angle_no_error() {
        let _config = default_config();
        assert!(true, "TODO");
    }

    /// Test to ensure integrators are reset when PWM is below threshold.
    #[test]
    fn test_control_angle_low_throttle_integrator_reset() {
        let _config = default_config();
        assert!(true, "TODO");
    }

    /// Test the control_angle function with specific inputs to calculate expected PID outputs.
    #[test]
    fn test_control_angle_specific_pid_output() {
        let _config = default_config();
        assert!(true, "TODO");
    }

    /// Test that the integrator saturation works as expected by the DEFAULT_I_LIMIT.
    #[test]
    fn test_control_angle_integrator_saturation() {
        let _config = default_config();
        assert!(true, "TODO");
    }
}
