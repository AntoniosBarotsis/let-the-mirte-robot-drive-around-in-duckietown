use once_cell::sync::OnceCell;
use rosrust::Client;
use rosrust_msg::mirte_msgs::{SetMotorSpeed, SetMotorSpeedReq};

use crate::ros_error::RosError;

// Create the `mirte_msgs` types.
rosrust::rosmsg_include!(mirte_msgs / SetMotorSpeed);

static CLIENT_INSTANCE: OnceCell<DriveClient> = OnceCell::new();

/// Client responsible for interacting with the motors.
#[allow(missing_debug_implementations)]
pub struct DriveClient {
  client_left: Client<SetMotorSpeed>,
  client_right: Client<SetMotorSpeed>,
}

/// Holds whether each motor RPC succeeded or not.
#[derive(Debug, Clone, Copy)]
pub struct ClientMotorResponse {
  pub left_suceeded: bool,
  pub right_suceeded: bool,
}

impl DriveClient {
  /// Instantiates a new client.
  pub fn create() -> Result<&'static Self, RosError> {
    // The client is initialized only once for performance reasons and also because `rosrust::init`
    // needs to be ran exactly once for the requests to work. Any more would cause a panic.
    CLIENT_INSTANCE.get_or_try_init(|| {
      rosrust::init("vroom");

      let client_left = rosrust::client::<SetMotorSpeed>("/mirte/set_left_speed").map_err(|e| {
        RosError::ClientCreation {
          name: "/mirte/set_left_speed".to_owned(),
          message: e.to_string(),
        }
      })?;

      let client_right =
        rosrust::client::<SetMotorSpeed>("/mirte/set_right_speed").map_err(|e| {
          RosError::ClientCreation {
            name: "/mirte/set_right_speed".to_owned(),
            message: e.to_string(),
          }
        })?;

      Ok(DriveClient {
        client_left,
        client_right,
      })
    })
  }

  /// Sets the motors' power levels.
  pub fn drive(&self, left_power: i32, right_power: i32) -> Result<ClientMotorResponse, RosError> {
    let left_suceeded = self
      .client_left
      .req_async(SetMotorSpeedReq { speed: left_power })
      .read()?
      .map_err(RosError::ClientResponse)?
      .status;

    let right_suceeded = self
      .client_right
      .req_async(SetMotorSpeedReq { speed: right_power })
      .read()?
      .map_err(RosError::ClientResponse)?
      .status;

    Ok(ClientMotorResponse {
      left_suceeded,
      right_suceeded,
    })
  }
}
