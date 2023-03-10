import unittest
from Robot.Lana import Lana

class LanaTestCases(unittest.TestCase):

    def setUp(self):
        self.robot = Lana(206, 150, 300, 40)
    def test_robots_should_be_initialised(self):
        robot  = Lana(206,150,300,46.5)
        self.assertEqual(robot.arm_length, 150)

    def test_forward(self):
        result = self.robot.forward(10, 10, 10)
        self.assertEqual(round(result[2],3), -253.477)

    def test_inverse(self):
        result = self.robot.inverse(0,0,-253.477)
        self.assertEqual(round(result[0],3), 10.)



if __name__ == '__main__':
    unittest.main()
