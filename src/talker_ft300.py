#!/usr/bin/env python
import socket
import rospy
import math
import sys
from geometry_msgs.msg import WrenchStamped
    

class RobotiqFTsensor(object):
    def __init__(self):
        self.HOST = '192.168.11.3'
        self.PORT = 63351
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((self.HOST, self.PORT))
        rospy.init_node('robotiq_ft300', anonymous=True)
        self.rate = rospy.Rate(100)
        self.pub = rospy.Publisher('robotiq_ft_wrench', WrenchStamped, queue_size=10)
        self.FT = WrenchStamped()
        self.zero_sensor_flag = False
        self.main()

    def reset_socket(self):
        self.s.close()
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((self.HOST, self.PORT))

    def zero_sensor(self):
        HOST = '192.168.11.3'
        PORT = 63350
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((HOST, PORT))
        s.send('SET ZRO\n')
        print "set zero sensor"
        s.close()

    def main(self):
        '''ForceTorque sensor data
            data[0]: x-axis Force
            data[1]: y-axis Force
            data[2]: z-axis Force
            data[3]: x-axis Moment
            data[4]: y-axis Moment
            data[5]: z-axis Moment
        '''
        print "main"
        while not rospy.is_shutdown():
            # Reset FT300
            if self.zero_sensor_flag:
                self.zero_sensor()
                rospy.sleep(0.1)
                self.zero_sensor_flag = False

            # Sampling FT300 data
            amount_received = 0
            amount_expected = 2
            while amount_received < amount_expected:
                data = self.s.recv(1024)
                amount_received += len(data)

            l = []

            for t in data.split():
                try:
                    l.append(float(t))
                except ValueError:
                    pass
            data = data.replace("(", " ")
            data = data.replace(")", " ")
            data = data.replace(",", " ")

            da = data.split()

            # print data
            self.FT.wrench.force.x = float(da.pop(0))
            self.FT.wrench.force.y = float(da.pop(0))
            self.FT.wrench.force.z = float(da.pop(0))
            self.FT.wrench.torque.x = float(da.pop(0))
            self.FT.wrench.torque.y = float(da.pop(0))
            self.FT.wrench.torque.z = float(da.pop(0))
            self.pub.publish(self.FT)
            
            data = 0
            self.reset_socket()
            self.rate.sleep()


if __name__ == '__main__':
    FT300 = RobotiqFTsensor()
