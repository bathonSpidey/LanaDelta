import unittest
import matplotlib.pyplot as plt
import numpy as np

from Robot.Lana import Lana


class MyTestCase(unittest.TestCase):

    def setUp(self):
        self.robot = Lana(206, 150, 300, 40)
        motor_1 = np.linspace(self.robot.min_angle, self.robot.max_angle, 5)
        motor_2 = motor_1
        motor_3 = motor_2
        self.Rx, self.Ry, self.Rz = [], [], []
        for first_angle in motor_1:
            for second_angle in motor_2:
                for third_angle in motor_3:
                    x, y, z = self.robot.forward(first_angle, second_angle, third_angle)
                    self.Rx.append(x)
                    self.Ry.append(y)
                    self.Rz.append(z)

    def test_workspace_should_make_scatter_plot(self):
        ax, cm = self.get_plot()
        ax.scatter(self.Rx, self.Ry, self.Rz, c=self.Rz, marker='o', cmap=cm)
        plt.show()

    def get_plot(self):
        fig = plt.figure()
        ax = plt.axes(projection='3d')
        cm = plt.cm.get_cmap('RdYlBu')
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')
        return ax, cm

    def test_workspace_surface(self):
        ax, cm = self.get_plot()
        X, Y = np.meshgrid(self.Rx, self.Ry)
        Z = X ** 2 + Y ** 2
        ax.plot_surface(X, Y, Z, cmap='plasma')
        plt.show()




if __name__ == '__main__':
    unittest.main()
