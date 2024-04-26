// src/controller/rate.rs

//! # Rate PID-based Flight Stabilization Controller
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

// Sensor Variables
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
static mut INTEGRAL_ROLL_PREV: f32 = 0.0;
static mut ERROR_ROLL_PREV: f32 = 0.0;
static mut INTEGRAL_PITCH_PREV: f32 = 0.0;
static mut ERROR_PITCH_PREV: f32 = 0.0;
static mut INTEGRAL_YAW_PREV: f32 = 0.0;
static mut ERROR_YAW_PREV: f32 = 0.0;

/// PID control for rate stabilization.
pub unsafe fn control_rate() {
    // Roll control
    let error_roll = ROLL_DES - GYROX;
    let mut integral_roll = INTEGRAL_ROLL_PREV + error_roll * DT;
    if CHANNEL_1_PWM < 1060 {
        integral_roll = 0.0; // Prevent integrator buildup if throttle is too low
    }
    integral_roll = integral_roll.clamp(-I_LIMIT, I_LIMIT);
    let derivative_roll = (error_roll - ERROR_ROLL_PREV) / DT;
    ROLL_PID = 0.01
        * (KP_ROLL_RATE * error_roll
            + KI_ROLL_RATE * integral_roll
            + KD_ROLL_RATE * derivative_roll);

    // Pitch control
    let error_pitch = PITCH_DES - GYROY;
    let mut integral_pitch = INTEGRAL_PITCH_PREV + error_pitch * DT;
    if CHANNEL_1_PWM < 1060 {
        integral_pitch = 0.0; // Prevent integrator buildup if throttle is too low
    }
    integral_pitch = integral_pitch.clamp(-I_LIMIT, I_LIMIT);
    let derivative_pitch = (error_pitch - ERROR_PITCH_PREV) / DT;
    PITCH_PID = 0.01
        * (KP_PITCH_RATE * error_pitch
            + KI_PITCH_RATE * integral_pitch
            + KD_PITCH_RATE * derivative_pitch);

    // Yaw control, stabilize on rate from GYROZ
    let error_yaw = YAW_DES - GYROZ;
    let mut integral_yaw = INTEGRAL_YAW_PREV + error_yaw * DT;
    if CHANNEL_1_PWM < 1060 {
        integral_yaw = 0.0; // Prevent integrator buildup if throttle is too low
    }
    integral_yaw = integral_yaw.clamp(-I_LIMIT, I_LIMIT);
    let derivative_yaw = (error_yaw - ERROR_YAW_PREV) / DT;
    YAW_PID = 0.01
        * (KP_YAW_RATE * error_yaw + KI_YAW_RATE * integral_yaw + KD_YAW_RATE * derivative_yaw);

    // Update previous states for next iteration
    ERROR_ROLL_PREV = error_roll;
    INTEGRAL_ROLL_PREV = integral_roll;
    ERROR_PITCH_PREV = error_pitch;
    INTEGRAL_PITCH_PREV = integral_pitch;
    ERROR_YAW_PREV = error_yaw;
    INTEGRAL_YAW_PREV = integral_yaw;
}

#[cfg(test)]
mod tests {
    use super::*;

    fn reset_initial_conditions() {
        unsafe {
            // Time step for PID calculation
            DT = 0.01; // Initial default value

            // Sensor Variables
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
            INTEGRAL_ROLL_PREV = 0.0;
            ERROR_ROLL_PREV = 0.0;
            INTEGRAL_PITCH_PREV = 0.0;
            ERROR_PITCH_PREV = 0.0;
            INTEGRAL_YAW_PREV = 0.0;
            ERROR_YAW_PREV = 0.0;
        }
    }

    /// Test the no error contidion.
    #[test]
    fn test_control_rate_no_error() {
        unsafe {
            reset_initial_conditions();

            // Initialize global state
            CHANNEL_1_PWM = 1200; // Above the threshold to allow integrator to build
            GYROX = 5.0;
            GYROY = 10.0;
            GYROZ = -10.0;

            // Expected initial conditions
            ROLL_DES = GYROX;
            PITCH_DES = GYROY;
            YAW_DES = GYROZ;
            INTEGRAL_ROLL_PREV = 0.0;
            INTEGRAL_PITCH_PREV = 0.0;
            INTEGRAL_YAW_PREV = 0.0;

            // Call the function to test
            control_rate();

            // Check if the PID outputs are as expected
            assert_eq!(
                0.0, ROLL_PID,
                "Roll PID should be zero as there is no error."
            );
            assert_eq!(
                0.0, PITCH_PID,
                "Pitch PID should be zero as there is no error."
            );
            assert_eq!(0.0, YAW_PID, "Yaw PID should be zero as there is no error.");
        }
    }

    /// Test to ensure integrators are reset when PWM is below threshold.
    #[test]
    fn test_control_rate_low_throttle_integrator_reset() {
        unsafe {
            reset_initial_conditions();

            // Initialize global state with some values
            CHANNEL_1_PWM = 1050; // Below the threshold to prevent integrator buildup

            // Assume some previous integrator values
            INTEGRAL_ROLL_PREV = 5.0;
            INTEGRAL_PITCH_PREV = 5.0;
            INTEGRAL_YAW_PREV = 5.0;

            // Call the function to test
            control_rate();

            // Verify that integrators are reset
            assert_eq!(INTEGRAL_ROLL_PREV, 0.0, "Roll integrator should be reset.");
            assert_eq!(
                INTEGRAL_PITCH_PREV, 0.0,
                "Pitch integrator should be reset."
            );
            assert_eq!(INTEGRAL_YAW_PREV, 0.0, "Yaw integrator should be reset.");
        }
    }

    /// Test the control_angle function with specific inputs to calculate expected PID outputs.
    #[test]
    fn test_control_rate_specific_pid_output() {
        unsafe {
            reset_initial_conditions();

            // Set specific test values
            CHANNEL_1_PWM = 1200; // Above the minimum threshold for integrator activity
            GYROX = 1.0; // Simulated rate of change in roll
            GYROY = -1.0; // Simulated rate of change in pitch
            GYROY = -1.0; // Simulated rate of change in yaw

            // Desired angles are set to create a specific error
            ROLL_DES = 10.0; // Desired roll is 5 degrees greater than actual
            PITCH_DES = -10.0; // Desired pitch is 5 degrees less than actual
            YAW_DES = 5.0; // Set a desired yaw to generate a specific rate error

            // Set previous integrals
            INTEGRAL_ROLL_PREV = 0.2;
            INTEGRAL_PITCH_PREV = -0.2;
            INTEGRAL_YAW_PREV = 0.1;

            // Expected PID calculations
            let expected_roll_pid = 0.01
                * (KP_ROLL_RATE * (ROLL_DES - GYROX)
                    + KI_ROLL_RATE * (INTEGRAL_ROLL_PREV + (ROLL_DES - GYROX) * DT)
                    + KD_ROLL_RATE * ((ROLL_DES - GYROX) - ERROR_ROLL_PREV) / DT);
            assert!(
                (0.01588 - expected_roll_pid).abs() < 1e-5,
                "Expected roll PID calcualted incorrectly."
            );
            let expected_pitch_pid = 0.01
                * (KP_PITCH_RATE * (PITCH_DES - GYROY)
                    + KI_PITCH_RATE * (INTEGRAL_PITCH_PREV + (PITCH_DES - GYROY) * DT)
                    + KD_PITCH_RATE * ((PITCH_DES - GYROY) - ERROR_PITCH_PREV) / DT);
            assert!(
                (-0.01588 - expected_pitch_pid).abs() < 1e-5,
                "Expected pitch PID calcualted incorrectly."
            );
            let expected_yaw_pid = 0.01
                * (KP_YAW_RATE * (YAW_DES - GYROZ)
                    + KI_YAW_RATE * (INTEGRAL_YAW_PREV + (YAW_DES - GYROZ) * DT)
                    + KD_YAW_RATE * ((YAW_DES - GYROZ) - ERROR_YAW_PREV) / DT);
            assert!(
                (0.015825002 - expected_yaw_pid).abs() < 1e-5,
                "Expected yaw PID calcualted incorrectly."
            );

            // Execute the control function
            control_rate();

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

    /// Test that the integrator saturation works as expected by the I_LIMIT.
    #[test]
    fn test_control_rate_integrator_saturation() {
        unsafe {
            reset_initial_conditions();

            // Setup test conditions
            CHANNEL_1_PWM = 1200; // Above the threshold to allow integrator buildup
            GYROX = 0.0;
            GYROY = 0.0;
            GYROZ = 0.0;

            // Set a high error that would normally cause the integrator to exceed I_LIMIT
            ROLL_DES = 100.0;
            PITCH_DES = 100.0;
            YAW_DES = 100.0;

            // Call control_angle multiple times to simulate multiple time steps
            for _ in 0..100 {
                control_rate();
            }

            // Check if the integrators are clamped correctly
            assert!(
                INTEGRAL_ROLL_PREV.abs() <= I_LIMIT,
                "Roll integrator exceeded I_LIMIT: {}",
                INTEGRAL_ROLL_PREV
            );
            assert!(
                INTEGRAL_PITCH_PREV.abs() <= I_LIMIT,
                "Pitch integrator exceeded I_LIMIT: {}",
                INTEGRAL_PITCH_PREV
            );
            assert!(
                INTEGRAL_YAW_PREV.abs() <= I_LIMIT,
                "Yaw integrator exceeded I_LIMIT: {}",
                INTEGRAL_YAW_PREV
            );
        }
    }
}