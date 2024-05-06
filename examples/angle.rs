// examples/angle.rs

use free_flight_stabilization::{AngleStabilizer, FlightStabilizer, FlightStabilizerConfig};

fn main() {
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

    // Set the initial setpoints for roll, pitch, and yaw.
    // These default to zero.
    config.set_point_roll = 10.0;
    config.set_point_pitch = 0.0;
    config.set_point_yaw = 10.0;

    // Set the upper limit for the integral term to prevent windup.
    config.i_limit = 25.0;

    // Set the scale to adjust the PID outputs to the actuator range.
    config.scale = 0.01;

    // Set the angle stabilizer
    let mut stabilizer = AngleStabilizer::with_config(config);

    // Simulated sensor inputs and desired setpoints
    let set_point = (10.0, 0.0, 10.0); // desired roll, pitch, yaw
    let mut imu_attitude = (5.0, 5.0, 0.0); // current roll, pitch, yaw
    let mut gyro_rate = (1.0, -1.0, -1.0); // current roll rate, pitch rate, yaw rate
    let dt = 1.0; // time step
    let low_throttle = false;

    println!("                      Roll,    Pitch,      Yaw");
    let mut t = 0.0;
    for _ in 0..=10 {
        // Perform the control computation
        let (roll_pid, pitch_pid, yaw_pid) =
            stabilizer.control(set_point, imu_attitude, gyro_rate, dt, low_throttle);

        // print results
        println!("t = {:.3}, dt = {:.3}", t, dt);
        println!(
            "    Set Point:    {:-8.3}, {:-8.3}, {:-8.3}",
            set_point.0, set_point.1, set_point.2
        );
        println!(
            "    IMU Attitude: {:-8.3}, {:-8.3}, {:-8.3}",
            imu_attitude.0, imu_attitude.1, imu_attitude.2
        );
        println!(
            "    Gyroscope:    {:-8.3}, {:-8.3}, {:-8.3}",
            gyro_rate.0, gyro_rate.1, gyro_rate.2
        );
        println!(
            "    PID:          {:-8.3}, {:-8.3}, {:-8.3}",
            roll_pid, pitch_pid, yaw_pid
        );

        // simulate response
        imu_attitude.0 += roll_pid * dt;
        imu_attitude.1 += pitch_pid * dt;
        imu_attitude.2 += yaw_pid * dt;
        gyro_rate.0 = roll_pid;
        gyro_rate.1 = pitch_pid;
        gyro_rate.2 = yaw_pid;

        t += dt;
    }
}
