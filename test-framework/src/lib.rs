#![allow(dead_code, clippy::string_add)]

use rosrust::{Message, Publisher, Subscriber};
use std::path::PathBuf;
use std::process::{Child, Command, Stdio};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::{collections::VecDeque, sync::MutexGuard, thread, time::Duration};
use std::{env, time::Instant};

/// Struct to hold the ROSCORE process so it can be dropped upon panic.
struct ProcessWrapper {
  process: Child,
}

/// Function that launches the ROSCORE process and initialized ROSRUST.
fn init() -> ProcessWrapper {
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

    rosrust::shutdown();
  }
}

/// Handles sending/receiving on a topic
struct Topic<T: Message> {
  topic: String,
  publisher: Publisher<T>,
  subscriber: Subscriber,
  received_messages: Arc<Mutex<VecDeque<T>>>,
}

impl<T> Topic<T>
where
  T: Message,
{
  /// Instantiates [`TopicAgent`] asynchronously.
  pub fn create(topic: &str) -> Topic<T> {
    //create the node on a separate thread so waiting for subscribers doesn't deadlock the test
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

    Topic {
      topic: topic.to_owned(),
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
      .expect(&*format!("Could not send message to topic {}", &self.topic));
  }

  /// Gets received topic messages with a timeout of 1s.
  pub fn get_messages(&self) -> MutexGuard<'_, VecDeque<T>> {
    self.get_messages_timeout(Duration::from_secs(1))
  }

  /// Gets received topic messages.
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

  /// Gets a message and checks if it's equal to the given message
  pub fn assert_message(&self, message: T) {
    let messages = self.get_messages();
    let actual = messages
      .front()
      .expect("Message queue was empty after timeout")
      .to_owned();
    assert_eq!(message, actual);
  }
}

/// A subscriber that listens to the '/test/strings' topic and publishes the length of the string to '/test/lengths'
pub fn strlen() {
  let publisher = rosrust::publish("/test/lengths", 1).expect("Create publisher");
  publisher
    .wait_for_subscribers(None)
    .expect("wait for subscribers on /test/lengths");
  let _subscriber_raii = rosrust::subscribe(
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
  rosrust::spin();
}

#[cfg(test)]
mod tests {
  #![allow(clippy::unwrap_used)]
  use super::*;
  use rostest::ros_test;

  #[ros_test]
  fn it_works() {
    // Init environment
    let test = init();

    // Init topics
    let strings = Topic::<rosrust_msg::std_msgs::String>::create("/test/strings");
    let ints = Topic::<rosrust_msg::std_msgs::UInt32>::create("/test/lengths");

    // Create node
    thread::spawn(|| strlen());
    thread::sleep(Duration::from_secs(1));

    // Publish message
    let message = rosrust_msg::std_msgs::String {
      data: "Hello World".to_string(),
    };
    strings.ros_publish(message);

    // Assert response
    let expected = rosrust_msg::std_msgs::UInt32 { data: 11 };
    ints.assert_message(expected);
  }
}
