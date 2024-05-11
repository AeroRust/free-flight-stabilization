// src/lib.rs

//! # PID Flight Stabilization Controller
//!
//! This module provides a `no_std`, no-alloc Rust translation of the PID
//! (Proportional, Integral, Derivative) control functions from dRehmFlight
//! version 1.3, an Arduino-based flight controller software. These functions
//! are used to stabilize unmanned aerial vehicles (UAVs).

#![no_std]
#![deny(missing_docs)]

pub mod pid;
pub mod stabilizer;

#[doc(inline)]
pub use stabilizer::*;

#[cfg(test)]
mod test_utils;
