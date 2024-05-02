// src/stabilizer/flight_stabilizer.rs

//! A module specifying the shared interface for PID-based flight stabilizers.
//! This module is designed for embedded systems in no_std environments and can be
//! adapted for various types of aircraft, including drones and RC planes.
//! It includes a configuration structure for PID gains and a trait defining
//! the stabilization functionality.

use piddiy::Number as PiddiyNumber;

/// Custom trait to encapsulate base number requirements.
pub trait Number: PiddiyNumber {
    /// Clamps generic PartialOrd values within a given range.
    fn clamp(self, min: Self, max: Self) -> Self {
        if self < min {
            min
        } else if max < self {
            max
        } else {
            self
        }
    }
}

impl<T: PiddiyNumber> Number for T {}

/// Configuration for PID gains and other settings.
#[derive(Clone, Copy)]
pub struct FlightStabilizerConfig<T: Number> {
    /// Proportional gain for roll control.
    pub kp_roll: T,
    /// Integral gain for roll control.
    pub ki_roll: T,
    /// Derivative gain for roll control.
    pub kd_roll: T,
    /// Proportional gain for pitch control.
    pub kp_pitch: T,
    /// Integral gain for pitch control.
    pub ki_pitch: T,
    /// Derivative gain for pitch control.
    pub kd_pitch: T,
    /// Proportional gain for yaw control.
    pub kp_yaw: T,
    /// Integral gain for yaw control.
    pub ki_yaw: T,
    /// Derivative gain for yaw control.
    pub kd_yaw: T,
    /// Initial setpoint for roll angle.
    pub set_point_roll: T,
    /// Initial setpoint for pitch angle.
    pub set_point_pitch: T,
    /// Initial setpoint for roll angle.
    pub set_point_yaw: T,
    /// Upper limit for integral term to prevent integral windup.
    pub i_limit: T,
    /// Scale factor applied to PID output to match actuator range.
    pub scale: T,
}

impl<T: Number> FlightStabilizerConfig<T> {
    /// Creates a new configuration with default values for all parameters.
    /// Default values are zero or one are used.
    /// These should be replaced meaningful values that are tuned for the hardware.
    ///
    /// Example Usage
    /// ```
    /// use pid_flight_stabilization::stabilizer::flight_stabilizer::FlightStabilizerConfig;
    ///
    /// let mut config = FlightStabilizerConfig::<f32>::new();
    ///
    /// // Set the PID gains for roll, pitch, and yaw.
    /// config.kp_roll = 0.2;
    /// config.ki_roll = 0.3;
    /// config.kd_roll = -0.05;
    ///
    /// config.kp_pitch = 0.2;
    /// config.ki_pitch = 0.3;
    /// config.kd_pitch = -0.05;
    ///
    /// config.kp_yaw = 0.3;
    /// config.ki_yaw = 0.05;
    /// config.kd_yaw = 0.00015;
    ///
    /// // Set the initial setpoints for roll, pitch, and yaw.
    /// // These default to zero.
    /// config.set_point_roll = 0.0;
    /// config.set_point_pitch = 0.0;
    /// config.set_point_yaw = 0.0;
    ///
    /// // Set the upper limit for the integral term to prevent windup.
    /// config.i_limit = 25.0;
    ///
    /// // Set the scale to adjust the PID outputs to the actuator range.
    /// config.scale = 0.01;
    ///
    /// // The configuration is ready to use.
    /// use pid_flight_stabilization::stabilizer::angle::AngleStabilizer;
    ///
    /// let flight_stabilizer = AngleStabilizer::with_config(config);
    /// ```
    pub fn new() -> Self {
        Self {
            kp_roll: T::one(),
            ki_roll: T::zero(),
            kd_roll: T::zero(),
            kp_pitch: T::one(),
            ki_pitch: T::zero(),
            kd_pitch: T::zero(),
            kp_yaw: T::one(),
            ki_yaw: T::zero(),
            kd_yaw: T::zero(),
            set_point_roll: T::zero(),
            set_point_pitch: T::zero(),
            set_point_yaw: T::zero(),
            i_limit: T::one(),
            scale: T::one(),
        }
    }
}

/// A trait for PID-based flight stabilizers that handle roll, pitch,
/// and yaw control based on attitude and gyro data and dt..
pub trait FlightStabilizer<T: Number> {
    /// Takes desired setpoints, current IMU attitude, and gyro rates, then computes the control outputs.
    ///
    /// - `set_point`: A tuple of (roll, pitch, yaw) desired setpoints.
    /// - `imu_attitude`: A tuple of (roll, pitch, yaw) current IMU measurements.
    /// - `gyro_rate`: A tuple of (roll rate, pitch rate, yaw rate) from the gyroscope.
    /// - `dt`: Time delta since the last update.
    /// - `low_throttle`: Flag indicating if the throttle is low. Used for anti-integral windup.
    ///
    /// Returns a tuple of (roll control, pitch control, yaw control) outputs scaled for actuation.
    fn control(
        &mut self,
        set_point: (T, T, T),
        imu_attitude: (T, T, T),
        gyro_rate: (T, T, T),
        dt: T,
        low_throttle: bool,
    ) -> (T, T, T);
}
