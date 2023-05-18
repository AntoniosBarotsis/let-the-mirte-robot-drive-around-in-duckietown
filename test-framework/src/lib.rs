mod example_nodes;
use rostest::rostest;

fn instantiate_examples() {
  example_nodes::strlen();
}

#[cfg(test)]
mod tests {
  use super::*;
  use std::sync::{Arc, Mutex};
  use std::thread;
  use std::time::Duration;

  #[rostest]
  fn it_works() {
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
