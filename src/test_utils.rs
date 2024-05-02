// src/test_utils.rs

//! This module contains utilities for testing.

/// A constant defining the tolerance within which floating-point values
/// are considered close enough to be equal.
pub const TEST_TOLERANCE: f32 = 1e-5;

/// Checks if two floating point numbers are close enough to be considered
/// equal.
///
/// # Arguments
/// * `target` - The target value.
/// * `value` - The value to compare against the target.
///
/// # Returns
/// `true` if the absolute difference between `target` and `value` is less than
/// `TEST_TOLERANCE`, otherwise `false`.
pub fn value_close(target: f32, value: f32) -> bool {
    (target - value).abs() < TEST_TOLERANCE
}

/// Checks if two floating point numbers are not close enough to be
/// considered equal.
///
/// # Arguments
/// * `target` - The target value.
/// * `value` - The value to compare against the target.
///
/// # Returns
/// `true` if the absolute difference between `target` and `value` exceeds
/// `TEST_TOLERANCE`, otherwise `false`.
pub fn value_not_close(target: f32, value: f32) -> bool {
    TEST_TOLERANCE <= (target - value).abs()
}

/// Checks if each of the components in a vector is close enough to
/// be considered equal.
///
/// # Arguments
/// * `target` - The target vector as a tuple of three `f32` values.
/// * `value` - The vector to compare against the target.
///
/// # Returns
/// `true` if each component of `target` and `value` is close as per `value_close`,
/// otherwise `false`.
pub fn vector_close(target: (f32, f32, f32), value: (f32, f32, f32)) -> bool {
    value_close(target.0, value.0)
        && value_close(target.1, value.1)
        && value_close(target.2, value.2)
}

/// Checks if each of the components in a vector is not close enough
/// to be considered equal.
///
/// # Arguments
/// * `target` - The target vector as a tuple of three `f32` values.
/// * `value` - The vector to compare against the target.
///
/// # Returns
/// `true` if any component of `target` and `value` is not close as per `value_not_close`,
/// otherwise `false`.
pub fn vector_not_close(target: (f32, f32, f32), value: (f32, f32, f32)) -> bool {
    value_not_close(target.0, value.0)
        && value_not_close(target.1, value.1)
        && value_not_close(target.2, value.2)
}
