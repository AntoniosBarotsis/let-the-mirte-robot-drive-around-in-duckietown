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

/// Handles sending/receiving messages of type T
struct TopicAgent<T: Message> {
  topic: String,
  publisher: Publisher<T>,
  subscriber: Subscriber,
  received_messages: Arc<Mutex<VecDeque<T>>>,
}

trait Agent<T: Message> {
  fn new(topic: String) -> TopicAgent<T>;
}

impl<T: Message> Agent<T> for TopicAgent<T> {
  fn new(topic: String) -> TopicAgent<T> {
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
    TopicAgent {
      topic,
      publisher,
      subscriber,
      received_messages,
    }
  }
}

/// publishes a message to the given topic
fn ros_publish<T: Message>(test: &mut TestData, topic: &str, message: T) {
  let agent = test.search_agent(topic.to_string());
  agent
    .publisher
    .send(message)
    .expect("Could not send message to topic");
}

/// asserts that a message was published to the given topic
fn assert_message<T: Message>(test: &mut TestData, topic: &str, message: T) {
  let agent = test.search_agent(topic.to_string());
  let received = agent
    .received_messages
    .lock()
    .expect("Writing thread panicked while holding the lock on the message queue")
    .pop_front();
  let actual = match received {
    Some(msg) => msg,
    None => panic!(
      "Tried to read on topic {}, but there was no message to read!",
      topic
    ),
  };
  assert_eq!(message, actual);
}

/// Ensures that ROS forwards messages to a topic
fn instantiate_subscriber<T: Message>(test: &mut TestData, topic: &str) {
  test.search_agent::<T>(topic.to_string());
}

/// Keeps track of the state of the test
struct TestData {
  pub roscore_process: Child,
  pub agents: std::collections::HashMap<String, anymap::Map>,
}

trait SearchAgent {
  fn search_agent<T: Message>(&mut self, topic: String) -> &TopicAgent<T>;
}

impl SearchAgent for TestData {
  fn search_agent<T: Message>(&mut self, topic: String) -> &TopicAgent<T> {
    if !self.agents.contains_key(&topic) {
      self.agents.insert(topic.clone(), anymap::Map::new());
    }
    let mut agent_map = self.agents.get_mut(&topic).unwrap(); //should never fail
    if !agent_map.contains::<T>() {
      let agent: TopicAgent<T> = TopicAgent::new(topic.clone());
      agent_map.insert(agent);
    }
    agent_map.get::<TopicAgent<T>>().unwrap() //should never fail
  }
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

  Some(TestData {
    roscore_process: ros_launch,
    agents: std::collections::HashMap::new(),
  })
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
    let mut data = setup().expect("setup test");
    instantiate_subscriber::<rosrust_msg::std_msgs::UInt32>(&mut data, "/test/lengths");
    instantiate_subscriber::<rosrust_msg::std_msgs::String>(&mut data, "/test/strings");
    //instantiate the nodes in a separate thread
    let _ = thread::spawn(|| instantiate_examples());

    thread::sleep(Duration::from_secs(3));

    //publish a message to /test/strings
    let message = rosrust_msg::std_msgs::String {
      data: "Hello World".to_string(),
    };
    ros_publish(&mut data, "/test/strings", message);
    thread::sleep(Duration::from_secs(1)); //small delay to get the response
    let response = rosrust_msg::std_msgs::UInt32 { data: 11 };
    assert_message(&mut data, "/test/lengths", response);
  }
}
