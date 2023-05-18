mod example_nodes;

fn instantiate_example_nodes() {
  example_nodes::strlen_node();
}

#[cfg(test)]
mod tests {
  use super::*;
  use std::thread;
  use std::time::Duration;

  #[test]
  fn it_works() {
    //instantiate the nodes in a separate thread
    thread::spawn(|| instantiate_example_nodes());
    thread::sleep(Duration::from_millis(2000));
    //create a dummy publisher node that publishes to /test/strings
    let publisher = rosrust::publish("/test/strings", 1).unwrap();
    //create a dummy subscriber node that subscribes to /test/lengths
    let _subscriber =
      rosrust::subscribe("/test/lengths", 1, |msg: rosrust_msg::std_msgs::Int32| {
        rosrust::ros_info!("Subscriber received {}", msg.data);
      })
      .unwrap();
    let mut var = "A".to_owned();
    loop {
      //publish a message to /test/strings
      let message = rosrust_msg::std_msgs::String {
        data: var.clone().to_owned(),
      };
      publisher.send(message).unwrap();
      println!("sent message");
      var = var.to_owned() + "A";
      thread::sleep(Duration::from_millis(1000));
    }
  }
}
