import unittest

from mirte_duckietown._common import LineSegment, Colour, Line
from mirte_msgs.msg import LineSegment as LineSegmentMsg, Line as LineMsg


class TestCommonClasses(unittest.TestCase):
    """Test common classes"""

    def testLineSegment(self):
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

    def testLine(self):
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


if __name__ == "__main__":
    unittest.main()
