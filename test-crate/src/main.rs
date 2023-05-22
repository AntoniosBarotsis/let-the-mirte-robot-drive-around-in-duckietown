use rosrust::*;
use rostest::ros_test;
use std::thread;
use std::time::Duration;
use test_framework::{instantiate_node, instantiate_node_timeout, Topic};

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
        data: msg.data.len().to_owned() as u32,
      };
      let _ = publisher.send(message);
    },
  )
  .expect("Create subscriber on /test/strings");
  spin();
}

fn main() {}

#[ros_test]
fn test_outside_crate() {
  // Init topics
  let strings = Topic::<rosrust_msg::std_msgs::String>::create("/test/strings");
  let ints = Topic::<rosrust_msg::std_msgs::UInt32>::create("/test/lengths");

  // Create node
  instantiate_node(strlen);

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
  let strings = Topic::<rosrust_msg::std_msgs::String>::create("/test/strings");
  let ints = Topic::<rosrust_msg::std_msgs::UInt32>::create("/test/lengths");

  // Create node
  instantiate_node(strlen);

  // Publish message
  let message = rosrust_msg::std_msgs::String {
    data: "I am testing".to_string(),
  };
  strings.ros_publish(message);

  // Assert response
  let expected = rosrust_msg::std_msgs::UInt32 { data: 13 };
  ints.assert_message(expected);
}
