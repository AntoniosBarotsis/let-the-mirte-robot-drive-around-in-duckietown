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
