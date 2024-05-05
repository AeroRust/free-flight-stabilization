// src/pid/rate.rs

//! # Rate-Based PID Control Module
//!
//! This module provides a compute function and control data structure
//! to perform rate-based PID (Proportional-Integral-Derivative) control
//! calculations.

use crate::Number;
use piddiy::PidController;

/// Control data for rate-based PID stabilization callback.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct RateControlData<T> {
    /// The current rate of change, typically reported by a gyro.
    pub rate: T,
    /// The time delta since the last computation.
    pub dt: T,
    /// The maximum allowed value for the integral term, used to prevent integral windup.
    pub integral_limit: T,
    /// Flag to reset the integral term, typically used when the controller is inactive.
    pub reset_integral: bool,
}

/// Rate-based PID stabilization compute callback.
pub fn compute_rate<T: Number>(
    pid: &mut PidController<T, RateControlData<T>>,
    data: RateControlData<T>,
) -> (T, T, T) {
    let error = pid.set_point - data.rate;
    let integral = if !data.reset_integral {
        (pid.integral + error * data.dt).clamp(-data.integral_limit, data.integral_limit)
    } else {
        T::zero()
    };
    let derivative = (error - pid.error) / data.dt;

    (error, integral, derivative)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_utils::*;

    /// Test that the integral term is clamped to the specified limit.
    #[test]
    fn test_pid_rate_integral_clamping() {
        let mut pid = PidController::new();
        pid.compute_fn(compute_rate)
            .set_point(50.0)
            .kp(1.0)
            .ki(5.0)
            .kd(0.1);
        let data = RateControlData {
            rate: 0.0,
            dt: 1.0,
            integral_limit: 100.0, // Integral should not exceed this value.
            reset_integral: false,
        };

        // This would normally push integral way over 100 if not clamped
        for _ in 0..10 {
            let _ = pid.compute(data);
        }

        let (_, integral, _) = compute_rate(&mut pid, data);
        assert!(
            value_close(100.0, integral),
            "Integral should be clamped to 100."
        );
    }

    /// Test the behavior when the reset_integral flag is true.
    #[test]
    fn test_pid_rate_integral_reset() {
        let mut pid = PidController::new();
        pid.compute_fn(compute_rate)
            .set_point(10.0)
            .kp(1.0)
            .ki(1.0)
            .kd(0.1);
        let data = RateControlData {
            rate: 0.0,
            dt: 1.0,
            integral_limit: 100.0,
            reset_integral: false,
        };

        // First compute without reset to build up the integral.
        let (_, integral_first, _) = compute_rate(&mut pid, data);
        let _ = pid.compute(data);

        // Now compute with reset.
        let data_reset = RateControlData {
            reset_integral: true,
            ..data
        };
        let (_, integral_reset, _) = compute_rate(&mut pid, data_reset);
        let _ = pid.compute(data);

        assert!(
            value_close(integral_first, 10.0),
            "Integral before reset should accumulate."
        );
        assert!(
            value_close(integral_reset, 0.0),
            "Integral after reset should be zero."
        );
    }

    /// Test PID response with non-zero set point and zero rate.
    #[test]
    fn test_pid_rate_response() {
        let mut pid = PidController::new();
        pid.compute_fn(compute_rate)
            .set_point(10.0)
            .kp(1.0)
            .ki(1.0)
            .kd(1.0);
        let data = RateControlData {
            rate: 0.0,
            dt: 1.0,
            integral_limit: 100.0,
            reset_integral: false,
        };

        let (mut error, mut integral, mut derivative) = compute_rate(&mut pid, data);
        let mut output = pid.compute(data);

        assert!(value_close(10.0, error), "Error should be 10.");
        assert!(
            value_close(10.0, integral),
            "Integral should start to accumulate."
        );
        assert!(value_close(10.0, derivative), "Derivative should be 10.");
        assert!(
            value_close(30.0, output),
            "Output should be the sum of terms."
        );

        // Call again to test accumulation
        (error, integral, derivative) = compute_rate(&mut pid, data);
        output = pid.compute(data);
        assert!(value_close(10.0, error), "Error should be 10.");
        assert!(
            value_close(20.0, integral),
            "Integral should accumulate to 20."
        );
        assert!(value_close(0.0, derivative), "Derivative should be zero.");
        assert!(
            value_close(30.0, output),
            "Output should be the sum of terms."
        );
    }

    /// Test PID specific response with non-zero values.
    #[test]
    fn test_pid_rate_specific_output() {
        let mut pid = PidController::new();
        pid.compute_fn(compute_rate)
            .set_point(10.0)
            .kp(1.0)
            .ki(1.0)
            .kd(1.0);
        let data = RateControlData {
            rate: 7.0,
            dt: 1.0,
            integral_limit: 100.0,
            reset_integral: false,
        };

        let (error, integral, derivative) = compute_rate(&mut pid, data);
        let output = pid.compute(data);

        assert!(value_close(3.0, error), "Error should be 3.");
        assert!(
            value_close(3.0, integral),
            "Integral should start to accumulate."
        );
        assert!(value_close(3.0, derivative), "Derivative should 3.");
        assert!(
            value_close(9.0, output),
            "Output should be the sum of terms."
        );

        // Call again to test accumulation
        let (_, integral_second, _) = compute_rate(&mut pid, data);
        let _ = pid.compute(data);
        assert!(
            value_close(6.0, integral_second),
            "Integral should accumulate to 6."
        );
    }

    /// Test that PID computes zero output for zero error with zero initial conditions.
    #[test]
    fn test_pid_rate_zero_conditions() {
        let mut pid = PidController::new();
        pid.compute_fn(compute_rate)
            .set_point(0.0)
            .kp(1.0)
            .ki(0.0)
            .kd(0.0);
        let data = RateControlData {
            rate: 0.0,
            dt: 1.0,
            integral_limit: 10.0,
            reset_integral: false,
        };
        let (error, integral, derivative) = compute_rate(&mut pid, data);
        let output = pid.compute(data);

        assert!(value_close(0.0, error), "Error should be zero.");
        assert!(value_close(0.0, integral), "Integral should be zero.");
        assert!(value_close(0.0, derivative), "Derivative should be zero.");
        assert!(value_close(0.0, output), "Output should be zero.");
    }
}
