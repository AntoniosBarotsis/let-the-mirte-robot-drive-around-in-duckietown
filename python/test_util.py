import unittest

from mirte_duckietown._util import intersectWithHorizontalLine
from mirte_duckietown._common import Point, Line, Vector


class TestUtil(unittest.TestCase):
    """Test the Camera class"""

    def testIntersectWithHorizontalLine(self):
        """Test the intersectWithHorizontalLine method"""
        line = Line(Point(1, 0.5), Vector(1, 2), 0, 0)

        # Check if intersection with y=1 is calculated correctly
        self.assertEqual(intersectWithHorizontalLine(line, 1), 1.25)

        # Test with a horizontal line
        line = Line(Point(0, 0), Vector(1, 0), 0, 0)
        self.assertEqual(intersectWithHorizontalLine(line, 1), None)


if __name__ == "__main__":
    unittest.main()
