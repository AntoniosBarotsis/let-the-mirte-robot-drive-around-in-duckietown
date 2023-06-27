import unittest
from datetime import datetime, timedelta
from freezegun import freeze_time

from mirte_duckietown._common import (
    LineSegment,
    Colour,
    Line,
    AprilTag,
    Obstacle,
)
from mirte_duckietown.sign import Sign
from mirte_duckietown_msgs.msg import (
    LineSegment as LineSegmentMsg,
    Line as LineMsg,
    Obstacle as ObstacleMsg,
)
from mirte_duckietown.object import Object


class TestCommonClasses(unittest.TestCase):
    """Test common classes"""

    def testLineSegmentFromMessage(self):
        """Test the LineSegment class"""
        message = LineSegmentMsg()
        message.colour.type = 0
        message.start.x = 1
        message.start.y = 2
        message.end.x = 3
        message.end.y = 4

        line_segment = LineSegment.fromMessage(message)
        self.assertEqual(line_segment.colour, Colour(0))
        self.assertEqual(line_segment.start.x_coord, 1)
        self.assertEqual(line_segment.start.y_coord, 2)
        self.assertEqual(line_segment.end.x_coord, 3)
        self.assertEqual(line_segment.end.y_coord, 4)

    def testLineFromMessage(self):
        """Test the Line class"""
        message = LineMsg()
        message.origin.x = 0
        message.origin.y = 1
        message.direction.x = 2
        message.direction.y = 3

        line = Line.fromMessage(message)
        self.assertEqual(line.origin.x_coord, 0)
        self.assertEqual(line.origin.y_coord, 1)
        self.assertEqual(line.direction.x_coord, 2)
        self.assertEqual(line.direction.y_coord, 3)

    def testAprilTagEQ(self):
        """Test the __eq__ method of the AprilTag class"""
        today = datetime.now()
        yesterday = today - timedelta(days=1)

        # Only values have to be equal
        tag1 = AprilTag(42, today)
        tag2 = AprilTag(42, yesterday - timedelta(seconds=1))
        self.assertEqual(tag1, tag2)

        # Dates have no effect on equality
        tag1 = AprilTag(42, today)
        tag2 = AprilTag(43, today)
        self.assertNotEqual(tag1, tag2)

    @freeze_time("2012-01-14")
    def testAprilTagHasExpired(self):
        """Test the hasExpired method of the AprilTag class"""
        now = datetime.now()
        tag = AprilTag(42, now - timedelta(seconds=1))
        # Tag has almost expired, but not yet
        self.assertFalse(tag.hasExpired(1000))
        # Tag has expired
        self.assertTrue(tag.hasExpired(999))

    def testAprilTagToSign(self):
        """Test the toSign method of the AprilTag class"""
        tag = AprilTag(13, datetime.now())
        self.assertEqual(tag.toSign(), Sign.FOUR_WAY_INTERSECT)

    def testAprilTagGetStreetName(self):
        """Test the getStreetName method of the AprilTag class"""
        tag = AprilTag(530, datetime.now())
        self.assertEqual(tag.getStreetName(), "BARFOOT ST")

    def testObstacleFromMessage(self):
        """Test the Obstacle class"""
        message = ObstacleMsg()
        message.object.type = 0
        message.location.x = 1
        message.location.y = 2
        message.diameter = 3

        obstacle = Obstacle.fromMessage(message)
        self.assertEqual(obstacle.object, Object.MIRTE)
        self.assertEqual(obstacle.location.x_coord, 1)
        self.assertEqual(obstacle.location.y_coord, 2)
        self.assertEqual(obstacle.diameter, 3)


if __name__ == "__main__":
    unittest.main()
