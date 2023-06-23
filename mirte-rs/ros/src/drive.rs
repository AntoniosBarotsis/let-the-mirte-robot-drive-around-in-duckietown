//! Utility for driving Mirte around.
use once_cell::sync::OnceCell;
use rosrust::Client;
use rosrust_msg::mirte_msgs::{SetMotorSpeed, SetMotorSpeedReq};

use crate::ros_error::RosError;

// Create the `mirte_msgs` types.
rosrust::rosmsg_include!(mirte_msgs / SetMotorSpeed);

static CLIENT_INSTANCE: OnceCell<DriveClient> = OnceCell::new();

/// Minimum value a motor can hold.
pub const MOTOR_VALUE_MIN: i32 = -100;
/// Maximum value a motor can hold.
pub const MOTOR_VALUE_MAX: i32 = 100;

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

/// A Value Object for representing Motor Values. The value must be between [`MOTOR_VALUE_MIN`] and
/// [`MOTOR_VALUE_MAX`].
///
///
/// # Example
///
/// ```
/// # use ros::drive::MotorValue;
///
/// let power = MotorValue::try_from(100);
/// assert!(power.is_ok());
///
/// let power = MotorValue::try_from(101);
/// assert!(power.is_err());
/// ```
#[derive(Debug, Clone, Copy)]
pub struct MotorValue {
  value: i32,
}

impl TryFrom<i32> for MotorValue {
  type Error = RosError;

  fn try_from(value: i32) -> Result<Self, Self::Error> {
    if !(MOTOR_VALUE_MIN..=MOTOR_VALUE_MAX).contains(&value) {
      return Err(RosError::InvalidMotorValue { actual: value });
    }

    Ok(MotorValue { value })
  }
}

impl DriveClient {
  /// Instantiates a new client.
  ///
  /// This function is only ran once internally and is therefore very cheap. Any subsequent calls
  /// will return the cached result of the first call.
  pub fn create() -> Result<&'static Self, RosError> {
    // The client is initialized only once for performance reasons and also because `rosrust::init`
    // needs to be ran exactly once for the requests to work. Any more would cause a panic.
    CLIENT_INSTANCE.get_or_try_init(|| {
      rosrust::init("motor_control");

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
  pub fn drive(
    &self,
    left_power: MotorValue,
    right_power: MotorValue,
  ) -> Result<ClientMotorResponse, RosError> {
    let left_suceeded = self
      .client_left
      .req_async(SetMotorSpeedReq {
        speed: left_power.value,
      })
      .read()?
      .map_err(RosError::ClientResponse)?
      .status;

    let right_suceeded = self
      .client_right
      .req_async(SetMotorSpeedReq {
        speed: right_power.value,
      })
      .read()?
      .map_err(RosError::ClientResponse)?
      .status;

    Ok(ClientMotorResponse {
      left_suceeded,
      right_suceeded,
    })
  }
}

#[cfg(test)]
mod test {
  use super::{MotorValue, MOTOR_VALUE_MAX, MOTOR_VALUE_MIN};

  #[test]
  fn motor_values_bound_checks() {
    let power = MotorValue::try_from(MOTOR_VALUE_MIN);
    assert!(power.is_ok());

    let power = MotorValue::try_from(MOTOR_VALUE_MAX);
    assert!(power.is_ok());

    let power = MotorValue::try_from((MOTOR_VALUE_MAX - MOTOR_VALUE_MIN) / 2);
    assert!(power.is_ok());

    let power = MotorValue::try_from(MOTOR_VALUE_MAX + 1);
    assert!(power.is_err());

    let power = MotorValue::try_from(MOTOR_VALUE_MIN - 1);
    assert!(power.is_err());
  }
}
