// src/pid.rs

//! # PID Control Module
//!
//! This module provides compute functions and control data structures
//! to perform PID (Proportional-Integral-Derivative) control calculations.

mod angle;
#[doc(inline)]
pub use angle::*;

mod cascade_angle;
#[doc(inline)]
pub use cascade_angle::*;

mod rate;
#[doc(inline)]
pub use rate::*;
