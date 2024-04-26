// lib/controller.rs

//! # PID-based Flight Stabilization Controller
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
static KD_ROLL_ANGLE: f32 = 0.05;
static KP_PITCH_ANGLE: f32 = 0.2;
static KI_PITCH_ANGLE: f32 = 0.3;
static KD_PITCH_ANGLE: f32 = 0.05;

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
static mut INTEGRAL_ROLL_PREV: f32 = 0.0;
static mut ERROR_ROLL_PREV: f32 = 0.0;
static mut INTEGRAL_ROLL_OL_PREV: f32 = 0.0;
static mut INTEGRAL_ROLL_IL_PREV: f32 = 0.0;
static mut ROLL_IMU_PREV: f32 = 0.0;
static mut ROLL_DES_PREV: f32 = 0.0;
static mut INTEGRAL_PITCH_PREV: f32 = 0.0;
static mut ERROR_PITCH_PREV: f32 = 0.0;
static mut INTEGRAL_PITCH_OL_PREV: f32 = 0.0;
static mut INTEGRAL_PITCH_IL_PREV: f32 = 0.0;
static mut PITCH_IMU_PREV: f32 = 0.0;
static mut PITCH_DES_PREV: f32 = 0.0;
static mut INTEGRAL_YAW_PREV: f32 = 0.0;
static mut ERROR_YAW_PREV: f32 = 0.0;

/// Computes control commands based on state error (angle).
///
/// This function implements a basic PID control to stabilize on an angle
/// setpoint derived from `ROLL_DES`, `PITCH_DES`, and `YAW_DES`, which
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
    if CHANNEL_1_PWM < 1060 {
        // Avoid integrator buildup if low throttle
        integral_roll = 0.0;
    }
    integral_roll = integral_roll.clamp(-I_LIMIT, I_LIMIT); //Saturate integrator to prevent unsafe buildup
    let derivative_roll = GYROX;
    ROLL_PID = 0.01
        * (KP_ROLL_ANGLE * error_roll + KI_ROLL_ANGLE * integral_roll
            - KD_ROLL_ANGLE * derivative_roll);

    // Pitch
    let error_pitch = PITCH_DES - PITCH_IMU;
    let mut integral_pitch = INTEGRAL_PITCH_PREV + error_pitch * DT;
    if CHANNEL_1_PWM < 1060 {
        // Avoid integrator buildup if low throttle
        integral_pitch = 0.0;
    }
    integral_pitch = integral_pitch.clamp(-I_LIMIT, I_LIMIT); //Saturate integrator to prevent unsafe buildup
    let derivative_pitch = GYROY;
    PITCH_PID = 0.01
        * (KP_PITCH_ANGLE * error_pitch + KI_PITCH_ANGLE * integral_pitch
            - KD_PITCH_ANGLE * derivative_pitch);

    // Yaw, stablize on rate from GyroZ
    let error_yaw = YAW_DES - GYROZ;
    let mut integral_yaw = INTEGRAL_YAW_PREV + error_yaw * DT;
    if CHANNEL_1_PWM < 1060 {
        // Avoid integrator buildup if low throttle
        integral_yaw = 0.0;
    }
    integral_yaw = integral_yaw.clamp(-I_LIMIT, I_LIMIT); //Saturate integrator to prevent unsafe buildup
    let derivative_yaw = (error_yaw - ERROR_YAW_PREV) / DT;
    YAW_PID = 0.01 * (KP_YAW_RATE * error_yaw + KI_YAW_RATE * integral_yaw + KD_YAW_RATE * derivative_yaw);

    // Update roll variables
    INTEGRAL_ROLL_PREV = integral_roll;

    // Update pitch variables
    INTEGRAL_PITCH_PREV = integral_pitch;

    // Update yaw variables
    INTEGRAL_YAW_PREV = integral_yaw;
    ERROR_YAW_PREV = error_yaw;
}

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
    YAW_PID = 0.01 * (KP_YAW_RATE * error_yaw + KI_YAW_RATE * integral_yaw + KD_YAW_RATE * derivative_yaw);

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
    YAW_PID = 0.01 * (KP_YAW_RATE * error_yaw + KI_YAW_RATE * integral_yaw + KD_YAW_RATE * derivative_yaw);

    // Update previous states for next iteration
    ERROR_ROLL_PREV = error_roll;
    INTEGRAL_ROLL_PREV = integral_roll;
    ERROR_PITCH_PREV = error_pitch;
    INTEGRAL_PITCH_PREV = integral_pitch;
    ERROR_YAW_PREV = error_yaw;
    INTEGRAL_YAW_PREV = integral_yaw;
}
