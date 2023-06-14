import math
import time
import unittest
import lane_follower

from testing.camera.renderer import ImagePublisher


class TestLaneFollower(unittest.TestCase):
    def test_radians_from_vector(self):
        # 1, 1 -> 45 degrees -> pi/4 radians
        res = lane_follower.calculate_radians(1, 1)
        self.assertEqual(res, math.pi/4)

        # 1, 0 -> 0 degrees -> 0 radians
        res = lane_follower.calculate_radians(1, 0)
        self.assertEqual(res, 0)

        # 0, 1 -> 90 degrees -> pi/2 radians
        res = lane_follower.calculate_radians(0, 1)
        self.assertEqual(res, math.pi / 2)

        # -1, 0 -> 180 degrees -> pi radians
        res = lane_follower.calculate_radians(-1, 0)
        self.assertEqual(res, math.pi)

    def test_convert_radians(self):
        # 0 radians -> point to right -> +90 degrees
        res = lane_follower.convert_angle_to_degrees(0)
        self.assertEqual(res, 90)

        # pi/2 radians -> point up -> 0 degrees
        res = lane_follower.convert_angle_to_degrees(math.pi/2)
        self.assertEqual(res, 0)

        # pi radians -> point left -> -90 degrees
        res = lane_follower.convert_angle_to_degrees(math.pi)
        self.assertEqual(res, -90)

        # pi/4 radians -> point up-right -> +45 degrees
        res = lane_follower.convert_angle_to_degrees(math.pi/4)
        self.assertEqual(res, 45)

        # 3pi/4 radians -> point up-left -> -45 degrees
        res = lane_follower.convert_angle_to_degrees(3*math.pi/4)
        self.assertEqual(res, -45)

    def test_calculate_y_intercept(self):
        # (0, 0) to (0, 1) -> y1-intercept is 0
        res = lane_follower.calculate_y1_intercept(0, 0, 0, 1)
        self.assertEqual(res, 0)

        # (0, 10) to (2.5, 5) -> y1-intercept is 4.5
        res = lane_follower.calculate_y1_intercept(0, 10, 2.5, 5)
        self.assertEqual(res, 4.5)

        # (2, 9) to (-2, 2) -> y1-intercept is -2.571
        res = lane_follower.calculate_y1_intercept(2, 9, -2, 2)
        self.assertAlmostEqual(res, -2.571, places=3)

    def test_mock_setup(self):
        publisher = ImagePublisher()
        self.speed_l = 0
        self.speed_r = 0

        def speed_mock(motor, value):
            if motor == 'left':
                self.speed_l = value
            else:
                self.speed_r = value
        follower = lane_follower.Follower(speed_mock)
        follower.start_following()
        while True:
            publisher.step(self.speed_l, self.speed_r, 0.01)
            publisher.publish()
            time.sleep(0.25)


if __name__ == '__main__':
    #unittest.main()
    t = TestLaneFollower()
    t.test_mock_setup()

