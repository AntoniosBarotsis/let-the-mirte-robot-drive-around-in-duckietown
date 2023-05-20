#![allow(dead_code, clippy::string_add)]

use rosrust::{Message, Publisher, Subscriber};
use std::path::PathBuf;
use std::process::{Command, Stdio};
use std::sync::{Arc, Mutex};
use std::{collections::VecDeque, sync::MutexGuard, time::Duration};
use std::{env, time::Instant};

/// Handles sending/receiving messages of type T
struct TopicAgent<T: Message> {
  process_id: u32,
  topic: String,
  publisher: Publisher<T>,
  subscriber: Subscriber,
  received_messages: Arc<Mutex<VecDeque<T>>>,
}

impl<T> TopicAgent<T>
where
  T: Message,
{
  /// Internal function that launches the ROSCORE process.
  fn init() -> u32 {
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
    rosrust::init("test");
    while !rosrust::is_initialized() {}

    ros_launch.id()
  }

  /// Instantiates [`TopicAgent`].
  pub fn create(topic: String) -> TopicAgent<T> {
    let process_id = Self::init();
    let received_messages: Arc<Mutex<VecDeque<T>>> = Arc::new(Mutex::new(VecDeque::new()));
    let received_messages_clone = Arc::clone(&received_messages);

    let publisher = rosrust::publish(&topic, 1).expect("Could not create publisher on topic");

    let subscriber = rosrust::subscribe(&topic, 1, move |msg: T| {
      received_messages_clone
        .lock()
        .expect("Reading thread panicked while holding the lock on the message queue")
        .push_back(msg);
    })
    .expect("Could not create subscriber on topic");

    // Wait for subscribers
    while publisher.subscriber_count() == 0 {}

    TopicAgent {
      process_id,
      topic,
      publisher,
      subscriber,
      received_messages,
    }
  }

  /// Publishes a message to the given topic.
  pub fn ros_publish(&self, message: T) {
    self
      .publisher
      .send(message)
      .expect("Could not send message to topic");
  }

  /// Gets received topic messages with a timeout of 50ms.
  pub fn get_messages(&self) -> MutexGuard<'_, VecDeque<T>> {
    self.get_messages_timeout(Duration::from_millis(50))
  }

  /// Gets received topic messages.
  pub fn get_messages_timeout(&self, timeout: Duration) -> MutexGuard<'_, VecDeque<T>> {
    let start = Instant::now();

    while self
      .received_messages
      .lock()
      .expect("Another thread of the received_messages mutex paniced.")
      .is_empty()
    {
      let elapsed = start.elapsed();

      // TODO: Return an error instead
      assert!(elapsed <= timeout, "No message received.");
    }

    self
      .received_messages
      .lock()
      .expect("Another thread of the received_messages mutex paniced.")
  }
}

impl<T> Drop for TopicAgent<T>
where
  T: Message,
{
  /// Kills the ROSCORE process.
  fn drop(&mut self) {
    let process_id = i32::try_from(self.process_id).expect("PID could not be cast to i32.");

    nix::sys::signal::kill(
      nix::unistd::Pid::from_raw(process_id),
      nix::sys::signal::Signal::SIGINT,
    )
    .expect("send SIGINT to roscore");

    rosrust::shutdown();
  }
}

#[cfg(test)]
mod tests {
  #![allow(clippy::unwrap_used)]
  use super::*;
  use rostest::ros_test;

  #[ros_test]
  fn it_works() {
    // Create agent
    let agent = TopicAgent::<rosrust_msg::std_msgs::String>::create("/test/strings".to_owned());

    // Publish message
    let message = rosrust_msg::std_msgs::String {
      data: "Hello World".to_string(),
    };
    agent.ros_publish(message);

    // Get messages
    let received = agent.get_messages();
    assert!(!received.is_empty());

    let first_received = received.front().unwrap();
    let expected = &rosrust_msg::std_msgs::String {
      data: "Hello World".to_string(),
    };
    assert_eq!(first_received, expected);
  }
}
