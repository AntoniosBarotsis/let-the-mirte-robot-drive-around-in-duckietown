use mirte_msgs::{
  Colour, Lane, Line, LineSegment, LineSegmentList, Object, Obstacle, ObstacleList, Point, Vector,
};

rosrust::rosmsg_include!(
  mirte_msgs / Point,
  mirte_msgs / Colour,
  mirte_msgs / Vector,
  mirte_msgs / Line,
  mirte_msgs / Lane,
  mirte_msgs / LineSegment,
  mirte_msgs / LineSegmentList,
  mirte_msgs / Object,
  mirte_msgs / Obstacle,
  mirte_msgs / ObstacleList,
);

impl From<cv::object::Object> for Object {
  fn from(value: cv::object::Object) -> Self {
    match value {
      cv::object::Object::Duck => Object {
        type_: Object::DUCK,
      },
      cv::object::Object::Mirte => Object {
        type_: Object::MIRTE,
      },
    }
  }
}

impl From<cv::object::Obstacle> for Obstacle {
  fn from(value: cv::object::Obstacle) -> Self {
    let location = value.location.into();
    let diameter = value.diameter;
    let object = value.object.into();

    Obstacle {
      location,
      diameter,
      object,
    }
  }
}

impl From<Vec<cv::object::Obstacle>> for ObstacleList {
  fn from(value: Vec<cv::object::Obstacle>) -> Self {
    let obstacles = value.into_iter().map(Obstacle::from).collect::<Vec<_>>();

    ObstacleList { obstacles }
  }
}

impl From<cv::line::Point> for Point {
  fn from(value: cv::line::Point) -> Self {
    let x = value.x;
    let y = value.y;

    Point { x, y }
  }
}

impl From<cv::line::Colour> for Colour {
  fn from(value: cv::line::Colour) -> Self {
    match value {
      cv::line::Colour::Red => Colour { type_: Colour::RED },
      cv::line::Colour::Orange => Colour {
        type_: Colour::ORANGE,
      },
      cv::line::Colour::Yellow => Colour {
        type_: Colour::YELLOW,
      },
      cv::line::Colour::Green => Colour {
        type_: Colour::GREEN,
      },
      cv::line::Colour::Blue => Colour {
        type_: Colour::BLUE,
      },
      cv::line::Colour::Purple => Colour {
        type_: Colour::PURPLE,
      },
      cv::line::Colour::Black => Colour {
        type_: Colour::BLACK,
      },
      cv::line::Colour::White => Colour {
        type_: Colour::WHITE,
      },
    }
  }
}

impl From<cv::line::Vector> for Vector {
  fn from(value: cv::line::Vector) -> Self {
    let x = value.x;
    let y = value.y;

    Vector { x, y }
  }
}

impl From<cv::line::Line> for Line {
  fn from(value: cv::line::Line) -> Self {
    let origin = value.origin.into();
    let direction = value.dir.into();

    Line { origin, direction }
  }
}

impl From<cv::line::LineSegment> for LineSegment {
  fn from(value: cv::line::LineSegment) -> Self {
    let colour = value.colour.into();
    let start = value.start.into();
    let end = value.end.into();

    LineSegment { colour, start, end }
  }
}

impl From<Vec<cv::line::LineSegment>> for LineSegmentList {
  fn from(value: Vec<cv::line::LineSegment>) -> Self {
    let segments = value.into_iter().map(LineSegment::from).collect::<Vec<_>>();

    LineSegmentList { segments }
  }
}

impl From<cv::line::Lane> for Lane {
  fn from(value: cv::line::Lane) -> Self {
    let centre = value.centre.into();
    let left = value.left.into();
    let right = value.right.into();

    Lane {
      centre,
      left,
      right,
    }
  }
}
