/// A node that listens to the '/test/strings' topic and gives the length of the string to '/test/lengths'
pub fn strlen_node() {
  rosrust::init("strlen");

  let _subscriber_raii =
    rosrust::subscribe("/test/strings", 1, |msg: rosrust_msg::std_msgs::String| {
      let str: String = msg.data;
      println!("Received {}", str);
      let len = str.len();
      println!("Publishing {}", len);
      let _ = rosrust::publish::<rosrust_msg::std_msgs::Int32>("/test/lengths", len);
    })
    .expect("Create subscriber");

  rosrust::spin();
}
