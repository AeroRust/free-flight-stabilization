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

use crate::pid::{compute_angle, compute_rate, AngleControlData, RateControlData};
use crate::{CascadeBlendingConfig, FlightStabilizer, FlightStabilizerConfig, Number};
use piddiy::PidController;

/// Struct representing the Angle2 PID Flight Stabilization Controller.
/// This is a cascade PID controller that combines angle and rate.
pub struct Angle2Stabilizer<T: Number> {
    angle_roll_pid: PidController<T, AngleControlData<T>>,
    angle_pitch_pid: PidController<T, AngleControlData<T>>,
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
            .compute_fn(compute_angle)
            .set_point(angle_config.set_point_roll)
            .kp(angle_config.kp_roll)
            .ki(angle_config.ki_roll)
            .kd(angle_config.kd_roll);

        let mut angle_pitch_pid = PidController::new();
        angle_pitch_pid
            .compute_fn(compute_angle)
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
        let angle_roll_data = AngleControlData {
            measurement: imu_roll,
            rate: gyro_roll,
            dt: dt,
            integral_limit: self.angle_i_limit,
            reset_integral: low_throttle,
        };
        let angle_pitch_data = AngleControlData {
            measurement: imu_pitch,
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

// Time step for PID calculation, mutable to accommodate dynamic updates
static mut DT: f32 = 0.01; // Initial default value

// Integrator Limit
static I_LIMIT: f32 = 25.0;

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

// Blending Factors
static BLEND_GAIN: f32 = 30.0;
static BLEND_LIMIT: f32 = 240.0;
static B_LOOP_ROLL: f32 = 0.9;
static B_LOOP_PITCH: f32 = 0.9;

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
static mut ERROR_ROLL_PREV: f32 = 0.0;
static mut ERROR_PITCH_PREV: f32 = 0.0;
static mut ERROR_YAW_PREV: f32 = 0.0;
static mut ROLL_DES_PREV: f32 = 0.0;
static mut ROLL_IMU_PREV: f32 = 0.0;
static mut PITCH_DES_PREV: f32 = 0.0;
static mut PITCH_IMU_PREV: f32 = 0.0;

// Previous state for integrators
static mut INTEGRAL_ROLL_OL_PREV: f32 = 0.0;
static mut INTEGRAL_ROLL_IL_PREV: f32 = 0.0;
static mut INTEGRAL_PITCH_OL_PREV: f32 = 0.0;
static mut INTEGRAL_PITCH_IL_PREV: f32 = 0.0;
static mut INTEGRAL_YAW_PREV: f32 = 0.0;

/// Advanced PID control for angle stabilization with cascaded control scheme.
pub unsafe fn control_angle2() {
    // Outer loop calculations for roll
    let error_roll = ROLL_DES - ROLL_IMU;
    let mut integral_roll_ol = INTEGRAL_ROLL_OL_PREV + error_roll * DT;
    if CHANNEL_1_PWM < LOW_THROTTLE_LIMIT {
        integral_roll_ol = 0.0;
    }
    integral_roll_ol = integral_roll_ol.clamp(-I_LIMIT, I_LIMIT);
    //let derivative_roll = (ROLL_IMU - ROLL_IMU_PREV) / DT;
    let mut roll_des_ol = KP_ROLL_ANGLE * error_roll + KI_ROLL_ANGLE * integral_roll_ol;
    //    - KD_ROLL_ANGLE * derivative_roll;

    // Outer loop calculations for pitch
    let error_pitch = PITCH_DES - PITCH_IMU;
    let mut integral_pitch_ol = INTEGRAL_PITCH_OL_PREV + error_pitch * DT;
    if CHANNEL_1_PWM < LOW_THROTTLE_LIMIT {
        integral_pitch_ol = 0.0;
    }
    integral_pitch_ol = integral_pitch_ol.clamp(-I_LIMIT, I_LIMIT);
    //let derivative_pitch = (PITCH_IMU - PITCH_IMU_PREV) / DT;
    let mut pitch_des_ol = KP_PITCH_ANGLE * error_pitch + KI_PITCH_ANGLE * integral_pitch_ol;
    //    - KD_PITCH_ANGLE * derivative_pitch;

    // Apply loop gain and damping
    let kl = BLEND_GAIN;
    roll_des_ol *= kl;
    roll_des_ol = roll_des_ol.clamp(-BLEND_LIMIT, BLEND_LIMIT);
    roll_des_ol = (1.0 - B_LOOP_ROLL) * ROLL_DES_PREV + B_LOOP_ROLL * roll_des_ol;
    pitch_des_ol *= kl;
    pitch_des_ol = pitch_des_ol.clamp(-BLEND_LIMIT, BLEND_LIMIT);
    pitch_des_ol = (1.0 - B_LOOP_PITCH) * PITCH_DES_PREV + B_LOOP_PITCH * pitch_des_ol;

    // Inner loop for roll rate
    let error_roll_il = roll_des_ol - GYROX;
    let mut integral_roll_il = INTEGRAL_ROLL_IL_PREV + error_roll_il * DT;
    if CHANNEL_1_PWM < LOW_THROTTLE_LIMIT {
        integral_roll_il = 0.0;
    }
    integral_roll_il = integral_roll_il.clamp(-I_LIMIT, I_LIMIT);
    let derivative_roll = (error_roll_il - ERROR_ROLL_PREV) / DT;
    ROLL_PID = OUTPUT_SCALE
        * (KP_ROLL_RATE * error_roll_il
            + KI_ROLL_RATE * integral_roll_il
            + KD_ROLL_RATE * derivative_roll);

    // Inner loop for pitch rate
    let error_pitch_il = pitch_des_ol - GYROY;
    let mut integral_pitch_il = INTEGRAL_PITCH_IL_PREV + error_pitch_il * DT;
    if CHANNEL_1_PWM < LOW_THROTTLE_LIMIT {
        integral_pitch_il = 0.0;
    }
    integral_pitch_il = integral_pitch_il.clamp(-I_LIMIT, I_LIMIT);
    let derivative_pitch = (error_pitch_il - ERROR_PITCH_PREV) / DT;
    PITCH_PID = OUTPUT_SCALE
        * (KP_PITCH_RATE * error_pitch_il
            + KI_PITCH_RATE * integral_pitch_il
            + KD_PITCH_RATE * derivative_pitch);

    // Yaw
    let error_yaw = YAW_DES - GYROZ;
    let mut integral_yaw = INTEGRAL_YAW_PREV + error_yaw * DT;
    if CHANNEL_1_PWM < LOW_THROTTLE_LIMIT {
        integral_yaw = 0.0;
    }
    integral_yaw = integral_yaw.clamp(-I_LIMIT, I_LIMIT);
    let derivative_yaw = (error_yaw - ERROR_YAW_PREV) / DT;
    YAW_PID = OUTPUT_SCALE
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

    fn legacy_reset_initial_conditions() {
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

            // Previous state for errors
            ERROR_ROLL_PREV = 0.0;
            ERROR_PITCH_PREV = 0.0;
            ERROR_YAW_PREV = 0.0;
            ROLL_DES_PREV = 0.0;
            ROLL_IMU_PREV = 0.0;
            PITCH_DES_PREV = 0.0;
            PITCH_IMU_PREV = 0.0;

            // Previous state for integrators
            INTEGRAL_ROLL_OL_PREV = 0.0;
            INTEGRAL_ROLL_IL_PREV = 0.0;
            INTEGRAL_PITCH_OL_PREV = 0.0;
            INTEGRAL_PITCH_IL_PREV = 0.0;
            INTEGRAL_YAW_PREV = 0.0;
        }
    }

    /// Test the no error condition.
    #[test]
    fn legacy_test_stabilizer_angle2_no_error() {
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
    fn legacy_test_stabilizer_angle2_low_throttle_integrator_reset() {
        unsafe {
            legacy_reset_initial_conditions();

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
    fn legacy_test_stabilizer_angle2_specific_pid_output() {
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

            // Set initial integrals
            INTEGRAL_ROLL_OL_PREV = 5.0;
            INTEGRAL_ROLL_IL_PREV = 0.2;
            INTEGRAL_PITCH_OL_PREV = -5.0;
            INTEGRAL_PITCH_IL_PREV = -0.2;
            INTEGRAL_YAW_PREV = 0.2;

            // Set previous outer loop results
            ROLL_DES_PREV = 50.0;
            PITCH_DES_PREV = -50.0;

            // Outer loop calculates desired rate changes based on angle errors
            let roll_des_ol = (KP_ROLL_ANGLE * (ROLL_DES - ROLL_IMU)
                + KI_ROLL_ANGLE * (INTEGRAL_ROLL_OL_PREV + (ROLL_DES - ROLL_IMU) * DT))
                * BLEND_GAIN
                * B_LOOP_ROLL
                + (1.0 - B_LOOP_ROLL) * ROLL_DES_PREV;
            assert!(
                value_close(72.905, roll_des_ol),
                "Expected outer loop roll calculated incorrectly."
            );
            let pitch_des_ol = (KP_PITCH_ANGLE * (PITCH_DES - PITCH_IMU)
                + KI_PITCH_ANGLE * (INTEGRAL_PITCH_OL_PREV + (PITCH_DES - PITCH_IMU) * DT))
                * BLEND_GAIN
                * B_LOOP_PITCH
                + (1.0 - B_LOOP_PITCH) * PITCH_DES_PREV;
            assert!(
                value_close(-72.905, pitch_des_ol),
                "Expected outer loop pitch calculated incorrectly."
            );

            // Inner loop calculates the actual PID outputs based on the rate set by the outer loop
            let expected_roll_pid = OUTPUT_SCALE
                * (KP_ROLL_RATE * (roll_des_ol - GYROX)
                    + KI_ROLL_RATE * (INTEGRAL_ROLL_IL_PREV + (roll_des_ol - GYROX) * DT)
                    + KD_ROLL_RATE * ((roll_des_ol - GYROX) - ERROR_ROLL_PREV) / DT);
            assert!(
                value_close(0.124076605, expected_roll_pid),
                "Expected roll PID calcualted incorrectly."
            );
            let expected_pitch_pid = OUTPUT_SCALE
                * (KP_PITCH_RATE * (pitch_des_ol - GYROY)
                    + KI_PITCH_RATE * (INTEGRAL_PITCH_IL_PREV + (pitch_des_ol - GYROY) * DT)
                    + KD_PITCH_RATE * ((pitch_des_ol - GYROY - ERROR_PITCH_PREV) / DT));
            assert!(
                value_close(-0.124076605, expected_pitch_pid),
                "Expected pitch PID calcualted incorrectly."
            );
            let expected_yaw_pid = OUTPUT_SCALE
                * (KP_YAW_RATE * (YAW_DES - GYROZ)
                    + KI_YAW_RATE * (INTEGRAL_YAW_PREV + (YAW_DES - GYROZ) * DT)
                    + KD_YAW_RATE * ((YAW_DES - GYROZ - ERROR_YAW_PREV) / DT));
            assert!(
                value_close(0.034805, expected_yaw_pid),
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
    fn legacy_test_stabilizer_angle2_integrator_saturation() {
        unsafe {
            legacy_reset_initial_conditions();

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

    /// Test the control_angle function with specific inputs to calculate expected PID outputs.
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
        let angle_roll_data = AngleControlData {
            measurement: imu_attitude.0,
            rate: gyro_rate.0,
            dt: dt,
            integral_limit: angle_config.i_limit,
            reset_integral: low_throttle,
        };
        let angle_pitch_data = AngleControlData {
            measurement: imu_attitude.1,
            rate: gyro_rate.1,
            dt: dt,
            integral_limit: angle_config.i_limit,
            reset_integral: low_throttle,
        };

        // Compute the adjusted roll setpoint and internal values
        stabilizer.angle_roll_pid.set_point(set_point.0);
        let (roll_error, roll_integral, roll_derivative) = compute_angle(&mut stabilizer.angle_roll_pid, angle_roll_data);
        let mut adjusted_set_point_roll =
            angle_config.scale * (angle_config.kp_roll * roll_error + angle_config.ki_roll * roll_integral + angle_config.kd_roll * roll_derivative);
        adjusted_set_point_roll = stabilizer.blend(adjusted_set_point_roll, PREV_SET_POINT_ROLL);

        // Compute the adjusted pitch setpoint and internal values
        stabilizer.angle_pitch_pid.set_point(set_point.1);
        let (pitch_error, pitch_integral, pitch_derivative) = compute_angle(&mut stabilizer.angle_pitch_pid, angle_pitch_data);
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
