mod example_nodes;

use rostest::rostest;
use std::any::Any;
use std::env;
use std::io::Read;
use std::panic::AssertUnwindSafe;
use std::path::PathBuf;
use std::process::{Child, Command, Stdio};

struct TestData {
  pub roscore_process: Child,
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
  Some(TestData {
    roscore_process: ros_launch,
  })
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
  use std::time::Duration;

  #[test]
  fn it_works() {
    run_test(&it_works_fn);
  }

  fn it_works_fn() {
    let results = Arc::new(Mutex::from(vec![]));

    //instantiate the nodes in a separate thread
    let _ = thread::spawn(|| instantiate_examples());
    while !rosrust::is_initialized() {}

    //create a publisher that publishes to /test/strings
    let publisher = rosrust::publish("/test/strings", 1).unwrap();

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
    rosrust::shutdown();
  }
}
