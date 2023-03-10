import numpy as np
import math

class Lana:
    def __init__(self, platform_length, arm_length, rod_length, base_length):
        self.platform_length = platform_length
        self.arm_length = arm_length
        self. rod_length = rod_length
        self.base_length = base_length
        self.current_angle = [0.0, 0.0, 0.0]
        self.degree_to_radians = np.pi / 180.0
        self.robot_orientation = (platform_length - base_length) * math.tan(30*self.degree_to_radians) / 2.0
        self.current_position = self.forward(self.current_angle[0], self.current_angle[1], self.current_angle[2])
        self.min_angle = -5
        self.max_angle = 60

    def forward(self, theta_a, theta_b, theta_c):
        self.current_angle = [theta_a, theta_b, theta_c]
        theta_a *= self.degree_to_radians
        theta_b *= self.degree_to_radians
        theta_c *= self.degree_to_radians
        y1 = -(self.robot_orientation + self.arm_length * math.cos(theta_a))
        z1 = -self.arm_length* math.sin(theta_a);
        y2 = (self.robot_orientation + self.arm_length*math.cos(theta_b))*math.sin(30*self.degree_to_radians);
        x2 = y2*math.tan(60*self.degree_to_radians)
        z2 = - self.arm_length*math.sin(theta_b)
        y3 = (self.robot_orientation + self.arm_length*math.cos(theta_c))*math.sin(30*self.degree_to_radians)
        x3 = -y3 * math.tan(60*self.degree_to_radians)
        z3 = -self.arm_length * math.sin(theta_c)
        distance_difference = (y2-y1) * x3 - (y3-y1) * x2
        w1 = y1*y1 + z1*z1
        w2 = x2*x2 + y2*y2 + z2*z2
        w3 = x3*x3 + y3*y3 + z3*z3
        a1 = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1)
        b1 = -((w2-w1)*(y3-y1)-(w3-w1)*(y2-y1))/2.0
        a2 = -(z2 - z1) * x3 + (z3 - z1) * x2
        b2 = ((w2 - w1) * x3 - (w3 - w1) * x2) / 2.0
        aV = a1 * a1 + a2 * a2 + distance_difference * distance_difference
        bV = 2.0 * (a1 * b1 + a2 * (b2 - y1 * distance_difference) - z1 * distance_difference * distance_difference)
        cV = (b2 - y1 * distance_difference) * (b2 - y1 * distance_difference) +\
             b1 * b1 + distance_difference * distance_difference * (z1 * z1 - self.rod_length * self.rod_length)
        dV = bV * bV - 4.0 * aV * cV
        if dV <0:
            return "Error"
        z = -0.5 * (bV + math.sqrt(dV)) / aV;
        x = (a1 * z + b1) / distance_difference;
        y = (a2 * z + b2) / distance_difference;
        return x, y, z

    def inverse(self, x,y,z):
        theta_a, theta_b, theta_c = 0, 0, 0
        theta_a = self.get_angle(x,y,z)
        if (theta_a != "error"):
            theta_b = self.get_angle(x*math.cos(120*self.degree_to_radians)+y*math.sin(120*self.degree_to_radians),
                                y*math.cos(120*self.degree_to_radians)-x*math.sin(120*self.degree_to_radians),z)
            if theta_b != "error":
                theta_c = self.get_angle(x*math.cos(120*self.degree_to_radians)-y*math.sin(120*self.degree_to_radians),
                                y*math.cos(120*self.degree_to_radians)+x*math.sin(120*self.degree_to_radians),z)
        return theta_a, theta_b, theta_c


    def get_angle(self, x, y,z):
        y1 = -0.5 * math.tan(30* self.degree_to_radians) * self.platform_length
        y -= 0.5 *math.tan(30* self.degree_to_radians) * self.base_length
        aV = (x*x + y*y + z*z + self.arm_length**2- self.rod_length**2-y1**2 )/(2.0*z)
        bV = (y1-y)/z
        dV = -(aV+bV*y1) * (aV+bV*y1)+self.arm_length*(bV*bV*self.arm_length+self.arm_length)
        if (dV < 0):
            return "error"
        yj =  (y1 - aV*bV - math.sqrt(dV))/(bV*bV + 1)
        zj = aV + bV * yj
        return math.atan2(-zj,(y1-yj)) * 180.0/math.pi







