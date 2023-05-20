mod example_nodes;

use anymap::AnyMap;
use rosrust::{Message, Publisher, RosMsg, Subscriber};
use rostest::rostest;
use std::any::Any;
use std::collections::VecDeque;
use std::io::Read;
use std::panic::AssertUnwindSafe;
use std::path::PathBuf;
use std::process::{Child, Command, Stdio};
use std::sync::{Arc, Mutex};
use std::{env, thread};

/// Specifies the type of event
enum EventType {
  Send,
  Receive,
}

/// Handles events of messages with type T
struct Event<'a, T: Message> {
  event_type: EventType,
  message: T,
  agent: &'a TopicAgent<T>,
}

/// Handles sending/receiving messages of type T
struct TopicAgent<T: Message> {
  topic: String,
  publisher: Publisher<T>,
  subscriber: Subscriber,
  received_messages: Arc<Mutex<VecDeque<T>>>,
}

trait Agent<T: Message> {
  fn handle_event(&self, event: Event<T>);
}

impl<T: Message> Agent<T> for TopicAgent<T> {
  fn handle_event(&self, event: Event<T>) {
    match event.event_type {
      EventType::Send => self
        .publisher
        .send(event.message)
        .expect("Could not send message"),
      EventType::Receive => {
        let message = self
          .received_messages
          .lock()
          .expect("Writing thread panicked while holding the lock on the message queue")
          .pop_front();
        let actual = match message {
          Some(msg) => msg,
          None => panic!("No message left to read!"),
        };
        assert_eq!(event.message, actual);
      }
    }
  }
}

fn new<T: Message>(topic: String) -> Box<dyn Agent<T>> {
  let mut received_messages: Arc<Mutex<VecDeque<T>>> = Arc::new(Mutex::new(VecDeque::new()));
  let mut received_messages_clone = Arc::clone(&received_messages);
  let publisher =
    rosrust::publish(&*topic.clone(), 1).expect("Could not create publisher on topic");
  let subscriber = rosrust::subscribe(&*topic.clone(), 1, move |msg: T| {
    received_messages_clone
      .lock()
      .expect("Reading thread panicked while holding the lock on the message queue")
      .push_back(msg);
  })
  .expect("Could not create subscriber on topic");
  let agent = TopicAgent {
    topic,
    publisher,
    subscriber,
    received_messages,
  };
  Box::from(agent)
}

/// Keeps track of the state of the test
/// Creates a new test agent for every (topic, messageType) pair
/// agents will always outlive an Event object
struct TestData {
  pub roscore_process: Child,
  pub agents: std::collections::HashMap<String, anymap::Map<dyn Message>>,
}

impl Drop for TestData {
  fn drop(&mut self) {
    let process_id = self.roscore_process.id();
    nix::sys::signal::kill(
      nix::unistd::Pid::from_raw(process_id as i32),
      nix::sys::signal::Signal::SIGINT,
    )
    .expect("send SIGINT to roscore");
    self
      .roscore_process
      .wait()
      .expect("wait for roscore to end");

    rosrust::shutdown();
  }
}

fn instantiate_examples() {
  example_nodes::strlen();
}

/// Sets up a ROS environment for a test
fn setup() -> Option<TestData> {
  //this will launch roscore if it isn't already running
  let path = env::var_os("PATH")?;
  let ros_base = env::var("CMAKE_PREFIX_PATH").expect("get ROS path");
  let ros_path = ros_base.clone() + "/bin";
  let ros_setup = ros_base.clone() + "/setup.bash";
  let mut paths = env::split_paths(&path).collect::<Vec<_>>();
  paths.push(PathBuf::from(ros_path));
  let new_path = env::join_paths(paths).expect("append ROS path");
  env::set_var("PATH", &new_path);
  let ros_launch = Command::new("/bin/bash")
    .arg("-c")
    .arg("source ".to_owned() + &ros_setup + "&& roscore")
    .stdout(Stdio::null())
    .spawn()
    .expect("launch ROS");

  //wait for ros to initialize
  let mut init = false;
  while !init {
    init = Command::new("/bin/bash")
      .arg("-c")
      .arg("source ".to_owned() + &ros_setup + "&& rostopic list")
      .stdout(Stdio::null())
      .output()
      .expect("test a ROS command")
      .status
      .success();
  }

  //initiate rosrust
  rosrust::init("test");
  while !rosrust::is_initialized() {}

  None
  //Some(TestData {
  //  roscore_process: ros_launch,
  //  event_queue: VecDeque::new(),
  //})
}

/// Sets up a ROS environment and runs a test.
fn run_test(test: &dyn Fn()) {
  let data = setup().expect("setup test");
  test();
}

#[cfg(test)]
mod tests {
  use super::*;
  use std::sync::{Arc, Mutex};
  use std::thread;
  use std::thread::Thread;
  use std::time::{Duration, SystemTime};

  #[test]
  fn it_works() {
    run_test(&it_works_fn);
  }

  fn it_works_fn() {
    let results = Arc::new(Mutex::from(vec![]));

    //instantiate the nodes in a separate thread
    let _ = thread::spawn(|| instantiate_examples());

    //create a subscriber that subscribes to /test/lengths
    let results_copy = results.clone();
    let _subscriber = rosrust::subscribe(
      "/test/lengths",
      1,
      move |msg: rosrust_msg::std_msgs::UInt32| {
        results_copy.lock().unwrap().push(msg.data);
      },
    )
    .unwrap();

    //create a publisher that publishes to /test/strings
    let publisher = rosrust::publish("/test/strings", 1).unwrap();
    while publisher.subscriber_count() == 0 {} //wait until a subscriber registers to ROS
    publisher
      .wait_for_subscribers(Some(Duration::from_secs(30)))
      .expect("Wait for subscribers on /test/strings");

    //publish a message to /test/strings
    let message = rosrust_msg::std_msgs::String {
      data: "Hello World".to_string(),
    };
    publisher.send(message).unwrap();
    thread::sleep(Duration::from_secs(1)); //small delay to get the response
    assert_eq!(results.lock().unwrap().len(), 1);
  }
}
