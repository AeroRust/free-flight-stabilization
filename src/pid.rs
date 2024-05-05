// src/pid.rs

//! # PID Control Module
//!
//! This module provides compute functions and control data structures
//! to perform PID (Proportional-Integral-Derivative) control calculations.

pub mod angle;
pub use angle::*;
pub mod cascade_angle;
pub use cascade_angle::*;
pub mod rate;
pub use rate::*;
