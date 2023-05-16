use std::time::Duration;

use ros::drive::DriveClient;

/// Sets both motors to 100 and keeps them on for 2 seconds before coming to a stop.
fn main() {
  let client = DriveClient::create().expect("Create client");

  let _res = client.drive(100, 100).expect("Drive");

  std::thread::sleep(Duration::from_secs(2));

  let _res = client.drive(0, 0).expect("Drive");
}
