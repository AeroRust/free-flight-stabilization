// src/lib.rs

//! # PID-based Flight Stabilization Controller
//!
//! This module provides a `no_std`, no-alloc Rust translation of the PID
//! (Proportional, Integral, Derivative) control functions from dRehmFlight
//! version 1.3, an Arduino-based flight controller software. These functions
//! are used to stabilize unmanned aerial vehicles (UAVs) and assume a
//! single-threaded environment.
//!
//! ## Flight Readiness
//!
//! This code requires significant refactoring to be considered flight-ready.
//! For more details, see the README.md file.

#![no_std]
#![deny(missing_docs)]

pub mod controller;
