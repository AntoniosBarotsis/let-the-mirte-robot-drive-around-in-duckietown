use std::time::Duration;

use ros::drive::{DriveClient, MotorValue};

/// Sets both motors to 100 and keeps them on for 2 seconds before coming to a stop.
fn main() {
  let client = DriveClient::create().expect("Create client");

  let left_power = MotorValue::try_from(100).expect("Left value");
  let right_power = MotorValue::try_from(100).expect("Right value");

  let _res = client.drive(left_power, right_power).expect("Drive");

  std::thread::sleep(Duration::from_secs(2));

  let left_power = MotorValue::try_from(0).expect("Left value");
  let right_power = MotorValue::try_from(0).expect("Right value");

  let _res = client.drive(left_power, right_power).expect("Drive");
}
