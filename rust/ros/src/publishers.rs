//! Utility for interacting with ROS topic publishers.
#![allow(unused)]

use std::{
  marker,
  sync::{atomic::AtomicU8, mpsc::channel, Once},
  thread::{self, JoinHandle, Thread},
};

use once_cell::sync::OnceCell;
use rayon::{ThreadPool, ThreadPoolBuilder};
use rosrust::Publisher;

use crate::{init, RosError};

use common::mirte_duckietown_msgs::{Lane, Line, LineSegmentList, ObstacleList};

static THREAD_COUNT: AtomicU8 = AtomicU8::new(0);
static INSTANCE: OnceCell<RosBgPublisher> = OnceCell::new();
pub const LINE_SEGMENTS_TOPIC_NAME: &str = "line_segments";
pub const LANE_TOPIC_NAME: &str = "lanes";
pub const OBSTACLE_TOPIC_NAME: &str = "obstacles";
pub const STOP_LINE_TOPIC_NAME: &str = "stop_line";

/// Publishes ROS messages to topics using background threads.
///
/// # Notes
///
/// This is a singleton so as to not spawn a needless amount of threads. The
/// [`RosBgPublisher::get_or_create`] method takes care of that.
#[allow(missing_debug_implementations)]
pub struct RosBgPublisher {
  thread_pool: ThreadPool,
  line_segment_publisher: Publisher<LineSegmentList>,
  lane_publisher: Publisher<Lane>,
  obstacle_publisher: Publisher<ObstacleList>,
  stop_line_publisher: Publisher<Line>,
}

#[allow(clippy::expect_used)]
impl RosBgPublisher {
  /// Initializes or gets the existing instance of [`RosBgPublisher`].
  pub fn get_or_create() -> &'static Self {
    INSTANCE.get_or_init(|| {
      init();

      // Init publishers
      let line_segment_publisher = rosrust::publish::<LineSegmentList>(LINE_SEGMENTS_TOPIC_NAME, 100)
        .expect("Create LINE_SEGMENT_PUBLISHER");
      let lane_publisher =
        rosrust::publish::<Lane>(LANE_TOPIC_NAME, 100).expect("Create LANE_PUBLISHER");
      let obstacle_publisher = rosrust::publish::<ObstacleList>(OBSTACLE_TOPIC_NAME, 100)
        .expect("Create OBSTACLE_PUBLISHER");
      let stop_line_publisher =
        rosrust::publish::<Line>(STOP_LINE_TOPIC_NAME, 100).expect("Create STOP_LINE_PUBLISHER");

      // Use 2 threads in the thread pool since *in theory* we shouldn't need more for the 2
      // topics we have now.
      let thread_pool = ThreadPoolBuilder::new()
        .num_threads(2)
        .build()
        .expect("Could not initialize thread pool.");

      Self {
        thread_pool,
        line_segment_publisher,
        lane_publisher,
        obstacle_publisher,
        stop_line_publisher,
      }
    })
  }

  /// Publishes a line segment to the [`LINE_SEGMENTS_TOPIC_NAME`] ROS topic.
  pub fn publish_line_segment(&self, msg: impl Into<LineSegmentList> + Send + 'static) {
    // Clone is required as the thread might outlive &self.
    let publisher_clone = self.line_segment_publisher.clone();

    self.publish_work(LINE_SEGMENTS_TOPIC_NAME.to_string(), msg, publisher_clone);
  }

  /// Publishes a lane to the [`LANE_TOPIC_NAME`] ROS topic.
  pub fn publish_lane(&self, msg: impl Into<Lane> + Send + 'static) {
    // Clone is required as the thread might outlive &self.
    let publisher_clone = self.lane_publisher.clone();

    self.publish_work(LANE_TOPIC_NAME.to_string(), msg, publisher_clone);
  }

  /// Publishes an obstacle to the [`OBSTACLE_TOPIC_NAME`] ROS topic.
  pub fn publish_obstacle(&self, msg: impl Into<ObstacleList> + Send + 'static) {
    let publisher_clone = self.obstacle_publisher.clone();

    self.publish_work(OBSTACLE_TOPIC_NAME.to_string(), msg, publisher_clone);
  }

  /// Publishes a lane to the [`LANE_TOPIC_NAME`] ROS topic.
  pub fn publish_stop_line(&self, msg: impl Into<Line> + Send + 'static) {
    // Clone is required as the thread might outlive &self.
    let publisher_clone = self.stop_line_publisher.clone();

    self.publish_work(STOP_LINE_TOPIC_NAME.to_string(), msg, publisher_clone);
  }

  /// Internal method to remove some of the boilerplate of publishing to the topics in the
  /// background.
  fn publish_work<T>(
    &self,
    topic: String,
    msg: impl Into<T> + Send + 'static,
    publisher: Publisher<T>,
  ) where
    T: rosrust::Message,
  {
    self.thread_pool.spawn(move || {
      let res = publisher
        .send(msg.into())
        .map_err(|_e| RosError::Publisher { topic });

      // Log any errors
      if let Err(e) = res {
        rosrust::ros_err!("{}", e);
      }
    });
  }
}
