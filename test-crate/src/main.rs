use rosrust::{publish, spin, subscribe};
use rostest::ros_test;

/// A subscriber that listens to the '/test/strings' topic and publishes the length of the string to '/test/lengths'
pub fn strlen() {
  let publisher = publish("/test/lengths", 1).expect("Create publisher");
  publisher
    .wait_for_subscribers(None)
    .expect("wait for subscribers on /test/lengths");
  let _subscriber_raii = subscribe(
    "/test/strings",
    1,
    move |msg: rosrust_msg::std_msgs::String| {
      let message = rosrust_msg::std_msgs::UInt32 {
        data: u32::try_from(msg.data.len().to_owned()).expect("Couldn't convert length"),
      };
      publisher.send(message).expect("Could not send message");
    },
  )
  .expect("Create subscriber on /test/strings");
  spin();
}

fn main() {}

#[ros_test]
fn test_outside_crate() {
  // Init topics
  let strings = test_framework::Topic::<rosrust_msg::std_msgs::String>::create("/test/strings");
  let ints = test_framework::Topic::<rosrust_msg::std_msgs::UInt32>::create("/test/lengths");

  // Create node
  test_framework::instantiate_node(strlen);

  // Publish message
  let message = rosrust_msg::std_msgs::String {
    data: "Hello World".to_string(),
  };
  strings.ros_publish(message);

  // Assert response
  let expected = rosrust_msg::std_msgs::UInt32 { data: 11 };
  ints.assert_message(expected);
}

#[ros_test]
fn test_outside_crate_sequential() {
  // Init topics
  let strings = test_framework::Topic::<rosrust_msg::std_msgs::String>::create("/test/strings");
  let ints = test_framework::Topic::<rosrust_msg::std_msgs::UInt32>::create("/test/lengths");

  // Create node
  test_framework::instantiate_node(strlen);

  // Publish message
  let message = rosrust_msg::std_msgs::String {
    data: "I am testing".to_string(),
  };
  strings.ros_publish(message);

  // Assert response
  let expected = rosrust_msg::std_msgs::UInt32 { data: 12 };
  ints.assert_message(expected);
}
