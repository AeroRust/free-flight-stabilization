<!-- README.md -->

# PID-based Flight Stabilization Controller

This repository contains a `no_std`, no-alloc Rust translation of the PID
(Proportional, Integral, Derivative) control functions from dRehmFlight
version 1.3, an Arduino-based flight controller software. These functions
are used to stabilize unmanned aerial vehicles (UAVs) and are written with
minor external dependencies.

## Overview

The initial translation preserved the structure of the original Arduino
implementation to facilitate easy verification and comparison for those
familiar with dRehmFlight. The aim was to maintain a detailed commit history
to document the refactoring process.

Tagged version v0.0.0 contains code for both the old direct translation,
the new refactor, and tests with matching values for both. See the angle,
rate, and angle2 stabilizers. Tests pass and values match. Only the refactored
code remains in future commits.

## License

dRehmFlight was originally released under the GNU General Public License
(GPL) Version 3. Accordingly, this translation as a derivative work is
also released under the GPL Version 3. For more details, see the
[GPLv3 license](https://www.gnu.org/licenses/gpl-3.0.html), or the LICENSE
file in this repository.

