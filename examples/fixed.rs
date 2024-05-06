// examples/fixed.rs

#![feature(start)]
#![no_std]

extern crate libc;
use core::fmt::Write;
use fixed::types::I16F16;
use free_flight_stabilization::{AngleStabilizer, FlightStabilizer, FlightStabilizerConfig};

// Implement minimal formatting features for output.
struct Stdout;

impl Write for Stdout {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let mut buffer = [0u8; 1024]; // Buffer to hold the string and null terminator

        // Ensure we don't exceed the buffer size
        if s.len() + 1 > buffer.len() {
            return Err(core::fmt::Error);
        }

        // Copy the string into the buffer and null-terminate it
        buffer[..s.len()].copy_from_slice(s.as_bytes());
        buffer[s.len()] = 0; // Null terminator

        unsafe {
            // Use %s to print the string from the buffer
            libc::printf(b"%s\0".as_ptr() as *const _, buffer.as_ptr() as *const _);
        }
        Ok(())
    }
}

#[start]
fn _start(_: isize, _: *const *const u8) -> isize {
    let mut config = FlightStabilizerConfig::<I16F16>::new();

    // Set the PID gains for roll, pitch, and yaw.
    config.kp_roll = I16F16::from_num(0.2);
    config.ki_roll = I16F16::from_num(0.3);
    config.kd_roll = I16F16::from_num(-0.05);

    config.kp_pitch = config.kp_roll;
    config.ki_pitch = config.ki_roll;
    config.kd_pitch = config.kd_roll;

    config.kp_yaw = I16F16::from_num(0.3);
    config.ki_yaw = I16F16::from_num(0.05);
    config.kd_yaw = I16F16::from_num(0.00015);

    // Set the initial setpoints for roll, pitch, and yaw.
    // These default to zero.
    config.set_point_roll = I16F16::from_num(10.0);
    config.set_point_pitch = I16F16::from_num(0.0);
    config.set_point_yaw = I16F16::from_num(10.0);

    // Set the upper limit for the integral term to prevent windup.
    config.i_limit = I16F16::from_num(25.0);

    // Set the scale to adjust the PID outputs to the actuator range.
    config.scale = I16F16::from_num(0.01);

    // Set the angle stabilizer
    let mut stabilizer = AngleStabilizer::with_config(config);

    // Simulated sensor inputs and desired setpoints
    let set_point = (
        I16F16::from_num(10.0),
        I16F16::from_num(0.0),
        I16F16::from_num(10.0),
    ); // desired roll, pitch, yaw
    let mut imu_attitude = (
        I16F16::from_num(5.0),
        I16F16::from_num(5.0),
        I16F16::from_num(0.0),
    ); // current roll, pitch, yaw
    let mut gyro_rate = (
        I16F16::from_num(1.0),
        I16F16::from_num(-1.0),
        I16F16::from_num(-1.0),
    ); // current roll rate, pitch rate, yaw rate
    let dt = I16F16::from_num(1.0); // time step
    let low_throttle = false;

    let mut stdout = Stdout;
    writeln!(stdout, "                      Roll,    Pitch,      Yaw").ok();
    let mut t = I16F16::from_num(0.0);
    for _ in 0..=10 {
        // Perform the control computation
        let (roll_pid, pitch_pid, yaw_pid) =
            stabilizer.control(set_point, imu_attitude, gyro_rate, dt, low_throttle);

        // print results
        writeln!(stdout, "t = {:.3}, dt = {:.3}", t, dt).ok();
        writeln!(
            stdout,
            "    Set Point:    {:-8.3}, {:-8.3}, {:-8.3}",
            set_point.0, set_point.1, set_point.2
        )
        .ok();
        writeln!(
            stdout,
            "    IMU Attitude: {:-8.3}, {:-8.3}, {:-8.3}",
            imu_attitude.0, imu_attitude.1, imu_attitude.2
        )
        .ok();
        writeln!(
            stdout,
            "    Gyroscope:    {:-8.3}, {:-8.3}, {:-8.3}",
            gyro_rate.0, gyro_rate.1, gyro_rate.2
        )
        .ok();
        writeln!(
            stdout,
            "    PID:          {:-8.3}, {:-8.3}, {:-8.3}",
            roll_pid, pitch_pid, yaw_pid
        )
        .ok();

        // simulate response
        imu_attitude.0 += roll_pid * dt;
        imu_attitude.1 += pitch_pid * dt;
        imu_attitude.2 += yaw_pid * dt;
        gyro_rate.0 = roll_pid;
        gyro_rate.1 = pitch_pid;
        gyro_rate.2 = yaw_pid;

        t += dt;
    }

    0
}
