<!-- README.md -->

# PID-based Flight Stabilization Controller

This repository contains a `no_std`, no-alloc Rust translation of the PID
(Proportional, Integral, Derivative) control functions from dRehmFlight
version 1.3, an Arduino-based flight controller software. These functions
are used to stabilize unmanned aerial vehicles (UAVs) and are written with
no external dependencies.

## Overview

The initial translation preserves the structure of the original Arduino
implementation to facilitate easy verification and comparison for those
familiar with dRehmFlight. The aim is to maintain a detailed commit history
to document the refactoring process. As currently written, the code assumes
a single-threaded environment.

## Flight Readiness

The current architecture relies heavily on global state and `unsafe`
practices, making it unsuitable for actual flight applications. This approach
represents a fundamental mismatch with Rust’s design principles, and needs
to be refactored.

## License

dRehmFlight was originally released under the GNU General Public License
(GPL) Version 3. Accordingly, this translation as a derivative work is
also released under the GPL Version 3. For more details, see the
[GPLv3 license](https://www.gnu.org/licenses/gpl-3.0.html), or the LICENSE
file in this repository.

