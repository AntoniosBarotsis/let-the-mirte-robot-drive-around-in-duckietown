//! Test framework for ROS nodes. Allows testing of ROS topics and services.
//! # Examples
//! Testing ROS topics of a node:
//! ```
//! #[ros_test]
//! fn test() {
//!   // Init topics
//!   let strings = test_framework::Topic::<rosrust_msg::std_msgs::String>::create("/test/strings");
//!   let ints = test_framework::Topic::<rosrust_msg::std_msgs::UInt32>::create("/test/lengths");
//!
//!   // Create node
//!   test_framework::instantiate_node(strlen);
//!
//!   // Publish message
//!   let message = rosrust_msg::std_msgs::String {
//!     data: "Hello World".to_string(),
//!   };
//!   strings.ros_publish(message);
//!
//!   // Assert response
//!   let expected = rosrust_msg::std_msgs::UInt32 { data: 11 };
//!   ints.assert_message(expected);
//! }
//! ```
//! Testing ROS services of a node:
//! ```
//! #[ros_test]
//! fn test_service() {
//!   // Init
//!   let service =
//!     test_framework::Service::<rosrust_msg::roscpp_tutorials::TwoInts>::create("/test/add");
//!
//!   // Create node
//!   test_framework::instantiate_node(add_service);
//!
//!   // Publish message & assert response
//!   let message = rosrust_msg::roscpp_tutorials::TwoIntsReq { a: 9, b: 10 };
//!   let expected = rosrust_msg::roscpp_tutorials::TwoIntsRes { sum: 19 };
//!   service.assert_response(message, expected);
//! }
//! ```
//! # Notes
//! - The `init()` function is needed to run the test successfully. It is inserted automatically by the `#[ros_test]` macro.
//! - The ROSCORE process is launched automatically by the test framework, launching it on your machine will cause an error in the terminal, but the tests will still run.
//! - Topics and services only accept one datatype, even across tests. This means that it is not possible to have "/test/input" be a topic of type `String` in one test and a topic of type `UInt32` in another test.
//! - The ROSCORE process is kept alive by binding it to a variable called `_lifetime_variable` at the start of the test. This means you cannot use this variable name in your test.
#![allow(dead_code, clippy::string_add)]

use rosrust::{Client, Message, Publisher, ServicePair, Subscriber};
use std::path::PathBuf;
use std::process::{Child, Command, Stdio};
use std::sync::{Arc, Mutex};

use std::fmt::Formatter;
use std::thread::JoinHandle;
use std::{collections::VecDeque, fmt, sync::MutexGuard, thread, time::Duration};
use std::{env, time::Instant};

/// Struct to hold the ROSCORE process so it can be dropped upon panic.
#[derive(Debug)]
pub struct ProcessWrapper {
  process: Child,
}

/// Function that launches the ROSCORE process and initialized ROSRUST.
/// Inserted automatically by the `#[ros_test]` macro.
pub fn init() -> ProcessWrapper {
  //this will launch roscore if it isn't already running
  let path = env::var_os("PATH").expect("`PATH` not found");
  let ros_base = env::var("CMAKE_PREFIX_PATH").expect("Could not get CMAKE_PREFIX_PATH");
  let ros_path = ros_base.clone() + "/bin";
  let ros_setup = ros_base + "/setup.bash";
  let mut paths = env::split_paths(&path).collect::<Vec<_>>();
  paths.push(PathBuf::from(ros_path));
  let new_path = env::join_paths(paths).expect("Invalid character in the PATH variable");
  env::set_var("PATH", &new_path);

  let ros_launch = Command::new("/bin/bash")
    .arg("-c")
    .arg("source ".to_owned() + &ros_setup + "&& roscore")
    .stdout(Stdio::null())
    .spawn()
    .expect("Could not launch ROS");

  //wait for ros to initialize
  let mut init = false;
  while !init {
    init = Command::new("/bin/bash")
      .arg("-c")
      .arg("source ".to_owned() + &ros_setup + "&& rostopic list")
      .stdout(Stdio::null())
      .output()
      .expect("`rostopic list` exited with an error")
      .status
      .success();
  }

  //initiate rosrust
  if !rosrust::is_initialized() {
    rosrust::init("test");
    while !rosrust::is_initialized() {}
  }

  ProcessWrapper {
    process: ros_launch,
  }
}

impl Drop for ProcessWrapper {
  /// Kills the ROSCORE and ROSMASTER processes.
  fn drop(&mut self) {
    let process_id = i32::try_from(self.process.id()).expect("PID could not be cast to i32.");

    nix::sys::signal::kill(
      nix::unistd::Pid::from_raw(process_id),
      nix::sys::signal::Signal::SIGINT,
    )
    .expect("send SIGINT to roscore");
  }
}

/// Handles sending/receiving on a topic
pub struct Topic<T: Message> {
  topic: String,
  publisher: Publisher<T>,
  subscriber: Subscriber,
  received_messages: Arc<Mutex<VecDeque<T>>>,
}

impl<T: Message> fmt::Debug for Topic<T> {
  fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
    f.debug_struct("Topic")
      .field("topic", &self.topic)
      .field("received_messages", &self.received_messages)
      .finish()
  }
}

impl<T> Topic<T>
where
  T: Message,
{
  /// Instantiates a [`Topic`] that corresponds to the given topic.
  pub fn create(topic: &str) -> Topic<T> {
    //create the node on a separate thread so waiting for subscribers doesn't deadlock the test
    let received_messages: Arc<Mutex<VecDeque<T>>> = Arc::new(Mutex::new(VecDeque::new()));
    let received_messages_clone = Arc::clone(&received_messages);

    let publisher = rosrust::publish(topic, 1).expect("Could not create publisher on topic");

    let subscriber = rosrust::subscribe(topic, 1, move |msg: T| {
      received_messages_clone
        .lock()
        .expect("Reading thread panicked while holding the lock on the message queue")
        .push_back(msg);
    })
    .expect("Could not create subscriber on topic");

    // Wait for subscribers
    while publisher.subscriber_count() == 0 {}

    Topic {
      topic: topic.to_owned(),
      publisher,
      subscriber,
      received_messages,
    }
  }

  /// Publishes a message to the given topic.
  /// # Panics
  /// if the message could not be published
  pub fn ros_publish(&self, message: T) {
    self
      .publisher
      .send(message)
      .unwrap_or_else(|_| panic!("Could not send message to topic {}", &self.topic));
  }

  /// Gets received topic messages with a timeout of 1s.
  pub fn get_messages(&self) -> MutexGuard<'_, VecDeque<T>> {
    self.get_messages_timeout(Duration::from_secs(1))
  }

  /// Gets received topic messages.
  /// # Panics
  /// if there was no message in the queue after waiting for the timeout
  pub fn get_messages_timeout(&self, timeout: Duration) -> MutexGuard<'_, VecDeque<T>> {
    let start = Instant::now();

    while self
      .received_messages
      .lock()
      .expect("Another thread of the received_messages mutex panicked.")
      .is_empty()
    {
      let elapsed = start.elapsed();

      // TODO: Return an error instead
      assert!(elapsed <= timeout, "No message received.");
    }

    self
      .received_messages
      .lock()
      .expect("Another thread of the received_messages mutex panicked.")
  }

  /// Gets a message and checks if it's equal to the given message.
  /// # Panics
  /// if the message was not equal
  #[allow(clippy::needless_pass_by_value)] // makes test code more readable
  pub fn assert_message(&self, message: T) {
    let messages = self.get_messages();
    let actual = messages
      .front()
      .expect("Message queue was empty after timeout")
      .to_owned();
    assert_eq!(message, actual);
  }
}

/// Handles sending/receiving on a service.
pub struct Service<T: ServicePair> {
  service: String,
  client: Client<T>,
}

impl<T> Service<T>
where
  T: ServicePair,
{
  /// Instantiates a [`Service`] that corresponds to the given service.
  pub fn create(topic: &str) -> Service<T> {
    let client = rosrust::client::<T>(topic).expect("Could not create client on topic");
    Service {
      service: topic.to_owned(),
      client,
    }
  }

  /// Asserts if a service returns an expected response.
  /// Only possible with response messages that implement the `PartialEq` and `Debug` traits.
  /// # Panics
  /// if the response was not equal to the expected response
  #[allow(clippy::needless_pass_by_value)] // makes test code more readable
  pub fn assert_response(&self, request: T::Request, response: T::Response)
  where
    T::Response: PartialEq,
    T::Response: fmt::Debug,
  {
    let actual = self
      .client
      .req(&request)
      .expect("Failed to send request")
      .expect("Failed to get response");
    assert_eq!(response, actual);
  }
}

impl<T: ServicePair> fmt::Debug for Service<T> {
  fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
    write!(f, "{:?}", self.service)
  }
}

/// Instantiates the node defined by the given function, then waits for the duration specified by the timeout variable.
pub fn instantiate_node_timeout(node_function: fn(), timeout: Duration) {
  let _: JoinHandle<_> = thread::spawn(move || {
    node_function();
    rosrust::spin(); //just in case the supplied function does not call spin()
  });
  thread::sleep(timeout);
}

/// Instantiates the node defined by the given function, then waits one second for it to spin up
pub fn instantiate_node(node_function: fn()) {
  instantiate_node_timeout(node_function, Duration::from_secs(1));
}
