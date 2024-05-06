<!-- README.md -->

# Free Flight Stabilization Controller

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

## Usage Example

```rust
// Angle Stabilizer Example
use free_flight_stabilization::{AngleStabilizer, FlightStabilizer, FlightStabilizerConfig};
let mut config = FlightStabilizerConfig::<f32>::new();

// Set the PID gains for roll, pitch, and yaw.
config.kp_roll = 0.2;
config.ki_roll = 0.3;
config.kd_roll = -0.05;
config.kp_pitch = config.kp_roll;
config.ki_pitch = config.ki_roll;
config.kd_pitch = config.kd_roll;
config.kp_yaw = 0.3;
config.ki_yaw = 0.05;
config.kd_yaw = 0.00015;

// Set the upper limit for the integral term to prevent windup.
config.i_limit = 25.0;

// Set the scale to adjust the PID outputs to the actuator range.
config.scale = 0.01;

// Create the angle stabilizer.
let mut stabilizer = AngleStabilizer::with_config(config);

// Simulated sensor inputs and desired setpoints.
let set_point = (10.0, 0.0, 10.0); // desired roll, pitch, yaw
let imu_attitude = (5.0, 5.0, 0.0); // current roll, pitch, yaw
let gyro_rate = (1.0, -1.0, -1.0); // current roll rate, pitch rate, yaw rate
let dt = 0.01; // time step
let low_throttle = false;

// Perform the control computation.
let (roll_pid, pitch_pid, yaw_pid) =
    stabilizer.control(set_point, imu_attitude, gyro_rate, dt, low_throttle);
```

## License

dRehmFlight was originally released under the GNU General Public License
(GPL) Version 3. Accordingly, this translation as a derivative work is
also released under the GPL Version 3. For more details, see the
[GPLv3 license](https://www.gnu.org/licenses/gpl-3.0.html), or the LICENSE
file in this repository.

