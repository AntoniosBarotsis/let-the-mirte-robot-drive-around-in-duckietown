#![allow(unused)]

use std::{
  sync::{atomic::AtomicU8, mpsc::channel, Once},
  thread::{self, JoinHandle},
};

use once_cell::sync::OnceCell;
use rosrust::Publisher;

use crate::{init, RosError};

use bridge::mirte_msgs::{Lane, LineSegmentList};

static THREAD_COUNT: AtomicU8 = AtomicU8::new(0);

/// Publishes ROS messages to topics using background threads.
#[allow(missing_debug_implementations)]
pub struct RosBgPublisher {
  line_segment_sender: std::sync::mpsc::Sender<LineSegmentList>,
  lane_sender: std::sync::mpsc::Sender<Lane>,
  line_segment_publisher: Publisher<LineSegmentList>,
  lane_publisher: Publisher<Lane>,
}

impl Default for RosBgPublisher {
  fn default() -> Self {
    Self::new()
  }
}

impl RosBgPublisher {
  /// Creates a new instance of [`RosBgPublisher`].
  ///
  /// # Panics
  ///
  /// Note that each instance creates its own long-lived threads for managing the topic publishing.
  /// It's a good idea to *not* create *a lot* of instances of this struct as that would also
  /// create a lot of threads for no reason. **This is why this method will panic if there are
  /// more than 0 background threads initialized by this struct which as of now, means a second
  /// instance is about to be created.**
  ///
  /// This was not made into a singleton as the way I usually do that is by hiding the instance
  /// behind a static variable. In this case however, some of the fields are not [`Sync`] which is
  /// a prerequisite for static variables.
  #[allow(clippy::expect_used)]
  pub fn new() -> Self {
    let new_count = THREAD_COUNT.fetch_add(2, std::sync::atomic::Ordering::Relaxed);
    assert!(
      new_count == 0,
      "Tried creating more than one instance of RosBgPublisher."
    );

    init();

    let (line_segment_sender, line_segment_receiver) = channel();
    let (lane_sender, lane_receiver) = channel();

    let line_segment_publisher = rosrust::publish::<LineSegmentList>("line_segments", 1)
      .expect("Create LINE_SEGMENT_PUBLISHER");
    let line_segment_publisher_clone = line_segment_publisher.clone();

    let lane_publisher = rosrust::publish::<Lane>("lanes", 10).expect("Create LANE_PUBLISHER");
    let lane_publisher_clone = lane_publisher.clone();

    let _line_segment_thread = thread::spawn(move || loop {
      if let Ok(line_segment_list) = line_segment_receiver.recv() {
        Self::publish_work(
          "line_segments".to_owned(),
          line_segment_list,
          &line_segment_publisher_clone,
        );
      }
    });

    let _lane_thread = thread::spawn(move || loop {
      if let Ok(lane) = lane_receiver.recv() {
        Self::publish_work("line_segments".to_owned(), lane, &lane_publisher_clone);
      }
    });

    Self {
      line_segment_sender,
      lane_sender,
      line_segment_publisher,
      lane_publisher,
    }
  }

  pub fn publish_lane(&self, msg: impl Into<Lane>) -> Result<(), RosError> {
    Self::publish_work("lanes".to_string(), msg, &self.lane_publisher)
  }

  pub fn publish_line_segment(&self, msg: impl Into<LineSegmentList>) -> Result<(), RosError> {
    Self::publish_work(
      "line_segments".to_string(),
      msg,
      &self.line_segment_publisher,
    )
  }

  fn publish_work<T>(
    topic: String,
    msg: impl Into<T>,
    publisher: &Publisher<T>,
  ) -> Result<(), RosError>
  where
    T: rosrust::Message,
  {
    publisher
      .send(msg.into())
      .map_err(|_e| RosError::PublisherCreation { topic })
  }
}
