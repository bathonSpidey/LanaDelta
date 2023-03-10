import unittest
from Robot.Lana import Lana

class LanaTestCases(unittest.TestCase):
    def test_robots_should_be_initialised(self):
        robot  = Lana(206,150,300,46.5)
        self.assertEqual(robot.arm_length, 150)

    def test_forward(self):
        robot = Lana(206,150,300,40)
        previous_angle = robot.current_angle
        print(robot.current_position)
        robot.forward(10,10,10)
        self.assertNotEqual(previous_angle,robot.current_angle)


if __name__ == '__main__':
    unittest.main()
