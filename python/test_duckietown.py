import unittest
from unittest.mock import MagicMock
from datetime import datetime

from mirte_duckietown.duckietown import Camera
from mirte_duckietown._topic import Subscriber
from mirte_duckietown._common import (
    Point,
    LineSegment,
    Colour,
    Lane,
    Line,
    Vector,
    AprilTag,
    Obstacle,
)
from mirte_duckietown.sign import Sign
from mirte_duckietown.object import Object


class TestCamera(unittest.TestCase):
    """Test the Camera class"""

    def testGetLines(self):
        """Test the getLines method"""
        subscriber = MagicMock(spec=Subscriber)
        camera = Camera(subscriber=subscriber)

        # No lines are visible
        subscriber.getLines = MagicMock(return_value=[])
        self.assertEqual(camera.getLines(), [])

        # Two lines are visible
        subscriber.getLines = MagicMock(
            return_value=[
                LineSegment(Colour(0), Point(0, 0), Point(1, 1)),
                LineSegment(Colour(1), Point(1, 1), Point(2, 2)),
            ]
        )
        self.assertEqual(
            camera.getLines(),
            [
                LineSegment(Colour(0), Point(0, 0), Point(1, 1)),
                LineSegment(Colour(1), Point(1, 1), Point(2, 2)),
            ],
        )

    def testGetStopLine(self):
        """Test the getStopLine method"""
        subscriber = MagicMock(spec=Subscriber)
        camera = Camera(subscriber=subscriber)

        # Stop line is not visible
        subscriber.getStopLine = MagicMock(return_value=None)
        self.assertEqual(camera.getStopLine(), None)

        # Stop line is visible
        subscriber.getStopLine = MagicMock(
            return_value=Line(Point(0, 0), Vector(1, 1), 0, 0)
        )
        self.assertEqual(camera.getStopLine(), Line(Point(0, 0), Vector(1, 1), 0, 0))

    def testGetStopLineHeight(self):
        """Test the getStopLineHeight method"""
        subscriber = MagicMock(spec=Subscriber)
        camera = Camera(subscriber=subscriber)

        # Stop line is not visible
        subscriber.getStopLine = MagicMock(return_value=None)
        self.assertEqual(camera.getStopLineHeight(), None)

        # Stop line is vertical
        subscriber.getStopLine = MagicMock(
            return_value=Line(Point(0, 0), Vector(0, 1), 0, 0)
        )
        self.assertEqual(camera.getStopLineHeight(), None)

        # Instersecting of stop line with x=0.5 lies at y=0.25
        subscriber.getStopLine = MagicMock(
            return_value=Line(Point(0, 0), Vector(1, 0.5), 0, 0)
        )
        self.assertEqual(camera.getStopLineHeight(), 0.25)

        # Instersecting of stop line with x=0.5 lies below 0.0 (not visible)
        subscriber.getStopLine = MagicMock(
            return_value=Line(Point(0, 0), Vector(1, -0.5), 0, 0)
        )
        self.assertEqual(camera.getStopLineHeight(), 0.0)

        # Instersecting of stop line with x=0.5 lies above 1.0 (not visible)
        subscriber.getStopLine = MagicMock(
            return_value=Line(Point(0, 0), Vector(1, 4), 0, 0)
        )
        self.assertEqual(camera.getStopLineHeight(), 1.0)

    def testSeesStopLine(self):
        """Test the seesStopLine method"""
        subscriber = MagicMock(spec=Subscriber)
        camera = Camera(subscriber=subscriber, stop_line_threshold_height=0.75)

        # Stop line is not visible
        subscriber.getStopLine = MagicMock(return_value=None)
        self.assertEqual(camera.seesStopLine(), False)

        # Stop line lies close enough to the bottom of the image
        subscriber.getStopLine = MagicMock(
            return_value=Line(Point(0, 0), Vector(1, 1.51), 0, 0)
        )
        self.assertEqual(camera.seesStopLine(), True)

        # Stop line lies on threshold
        subscriber.getStopLine = MagicMock(
            return_value=Line(Point(0, 0), Vector(1, 1.50), 0, 0)
        )
        self.assertEqual(camera.seesStopLine(), True)

        # Stop line lies to high
        subscriber.getStopLine = MagicMock(
            return_value=Line(Point(0, 0), Vector(1, 1.49), 0, 0)
        )
        self.assertEqual(camera.seesStopLine(), False)

    def testGetAprilTags(self):
        """Test the getAprilTags method"""
        subscriber = MagicMock(spec=Subscriber)
        camera = Camera(subscriber=subscriber)

        # No AprilTags are visible
        subscriber.getAprilTags = MagicMock(return_value=[])
        self.assertEqual(camera.getAprilTags(), [])

        # Two AprilTags are visible
        subscriber.getAprilTags = MagicMock(
            return_value=[AprilTag(13, datetime.now()), AprilTag(531, datetime.now())]
        )
        self.assertEqual(
            camera.getAprilTags(),
            [AprilTag(13, datetime.now()), AprilTag(531, datetime.now())],
        )

    def testSeesSign(self):
        """Test the seesSign method"""
        subscriber = MagicMock(spec=Subscriber)
        camera = Camera(subscriber=subscriber)

        # No AprilTags are visible
        subscriber.getAprilTags = MagicMock(return_value=[])
        self.assertFalse(camera.seesSign(Sign.FOUR_WAY_INTERSECT))

        # AprilTag is visible, but not the one we are looking for
        subscriber.getAprilTags = MagicMock(
            return_value=[AprilTag(531, datetime.now())]
        )
        self.assertFalse(camera.seesSign(Sign.FOUR_WAY_INTERSECT))

        # AprilTag is visible and the one we are looking for
        subscriber.getAprilTags = MagicMock(
            return_value=[AprilTag(13, datetime.now()), AprilTag(531, datetime.now())]
        )
        self.assertTrue(camera.seesSign(Sign.FOUR_WAY_INTERSECT))

    def testSeesStreet(self):
        """Test the seesStreet method"""
        subscriber = MagicMock(spec=Subscriber)
        camera = Camera(subscriber=subscriber)

        # No AprilTags are visible
        subscriber.getAprilTags = MagicMock(return_value=[])
        self.assertFalse(camera.seesStreet("DUDEK ST"))

        # AprilTag is visible, but not the one we are looking for
        subscriber.getAprilTags = MagicMock(
            return_value=[AprilTag(13, datetime.now()), AprilTag(532, datetime.now())]
        )
        self.assertFalse(camera.seesStreet("DUDEK ST"))

        # AprilTag is visible and the one we are looking for
        subscriber.getAprilTags = MagicMock(
            return_value=[AprilTag(13, datetime.now()), AprilTag(531, datetime.now())]
        )
        self.assertTrue(camera.seesStreet("DUDEK ST"))
        self.assertTrue(camera.seesStreet("dudek st."))
        self.assertTrue(camera.seesStreet("dUdEk St"))

    def testSeesObstacleOnLane(self):
        """Test the seesObstacleOnLane method"""
        subscriber = MagicMock(spec=Subscriber)
        camera = Camera(subscriber=subscriber)

        # No Obstacles are visible
        subscriber.getObstacles = MagicMock(return_value=[])
        self.assertFalse(camera.seesObstacleOnLane(Object.MIRTE))

        # Obstacle is visible, but not on the lane
        subscriber.getObstacles = MagicMock(
            return_value=[Obstacle(15, Point(0, 0), Object.MIRTE)]
        )
        subscriber.getLane = MagicMock(
            return_value=Lane(
                Line(Point(0, 1), Vector(0.5, -1), 0, 0),
                Line(Point(0.5, 1), Vector(0, -1), 0, 0),
                Line(Point(1, 1), Vector(-0.5, -1), 0, 0),
            )
        )
        self.assertFalse(camera.seesObstacleOnLane(Object.MIRTE))

        # Obstacle is visible on the lane
        subscriber.getObstacles = MagicMock(
            return_value=[Obstacle(15, Point(0.5, 0.5), Object.MIRTE)]
        )
        subscriber.getLane = MagicMock(
            return_value=Lane(
                Line(Point(0, 1), Vector(0.5, -1), 0, 0),
                Line(Point(0.5, 1), Vector(0, -1), 0, 0),
                Line(Point(1, 1), Vector(-0.5, -1), 0, 0),
            )
        )
        self.assertFalse(camera.seesObstacleOnLane(Object.MIRTE))

    def testSeesObstacleOnLeft(self):
        """Test the seesObstacleOnLeft method"""
        subscriber = MagicMock(spec=Subscriber)
        camera = Camera(subscriber=subscriber)

        # No Obstacles are visible
        subscriber.getObstacles = MagicMock(return_value=[])
        self.assertFalse(camera.seesObstacleOnLeft(Object.MIRTE))

        # Obstacle is visible, but not on the left
        subscriber.getObstacles = MagicMock(
            return_value=[Obstacle(15, Point(1, 0), Object.MIRTE)]
        )
        self.assertFalse(camera.seesObstacleOnLeft(Object.MIRTE))

        # Obstacle is visible on the left
        subscriber.getObstacles = MagicMock(
            return_value=[Obstacle(15, Point(0, 0), Object.MIRTE)]
        )
        self.assertTrue(camera.seesObstacleOnLeft(Object.MIRTE))

    def testSeesObstacleOnRight(self):
        """Test the seesObstacleOnRight method"""
        subscriber = MagicMock(spec=Subscriber)
        camera = Camera(subscriber=subscriber)

        # No Obstacles are visible
        subscriber.getObstacles = MagicMock(return_value=[])
        self.assertFalse(camera.seesObstacleOnRight(Object.MIRTE))

        # Obstacle is visible, but not on the right
        subscriber.getObstacles = MagicMock(
            return_value=[Obstacle(15, Point(0, 0), Object.MIRTE)]
        )
        self.assertFalse(camera.seesObstacleOnRight(Object.MIRTE))

        # Obstacle is visible on the right
        subscriber.getObstacles = MagicMock(
            return_value=[Obstacle(15, Point(1, 0), Object.MIRTE)]
        )
        self.assertTrue(camera.seesObstacleOnRight(Object.MIRTE))


if __name__ == "__main__":
    unittest.main()
