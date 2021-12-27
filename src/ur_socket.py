#!/usr/bin/env python
import socket
import rospy
import math
import sys

class socketController(object):
    def __init__(self):
        self.HOST = '192.168.11.3'
        self.PORT = 30001
        self.angle_reset = [math.pi/4, -math.pi/2, math.pi/2, -math.pi,   -math.pi/2, 0]
        self.angle_init  = [math.pi/4, -math.pi/2, math.pi/2, -math.pi/2, -math.pi/2, 0]
        # self.angle_table  = [math.pi/4, -math.pi/180*70, math.pi/180*100, -math.pi/180*110, -math.pi/2, 0]
        self.angle_table  = [math.pi/4, -math.pi/180*70, math.pi/180*100, -math.pi/180*120, -math.pi/2, 0]
        # self.angle_table  = [math.pi/4, -math.pi/180*90, math.pi/180*120, -math.pi/180*120, -math.pi/2, 0]
        self.sum_z = None
        self.lim_z = -0.02

    def initConnection(self):
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((self.HOST, self.PORT))

    def setJointAngle(self, angle):
        self.initConnection()
        cmd = "movel([{}, {}, {}, {}, {}, {}], a=0.5, v=1.0)\n".format(angle[0], angle[1], angle[2], angle[3], angle[4], angle[5])
        self.s.send(cmd)
        self.s.close()

    def toBytes(self, str):
        return bytes(str.encode())

    def checkCollision(self, z):
        self.sum_z += z
        if self.sum_z < self.lim_z:
            print "WARNING!!!"
            sys.exit()
        else:
            pass

    def moveCartesianSpace(self, x, y, z):
        print "move"
        xx = x / math.sqrt(2)
        yy = x / math.sqrt(2)
        xx += y / math.sqrt(2)
        yy -= y / math.sqrt(2)
        zz = z
        self.checkCollision(zz)
        self.initConnection()
        self.s.send(self.toBytes(
            "def myProg():"+"\n"
            +"begin_pos = get_actual_tcp_pose()" +"\n"
            +"pos_end = pose_add(begin_pos, p[{}, {}, {}, 0.0, 0.0, 0.0])".format(xx, yy, zz) +"\n"
            +"movel(pos_end , a=0.5, v=0.1)" + "\n"
            +"end" +"\n"))
        self.s.close()

    def trace(self, vel):
        print "trace"
        xx = 0.1 / math.sqrt(2)
        yy = 0.1 / math.sqrt(2)
        self.initConnection()
        self.s.send(self.toBytes(
            "def myProg():"+"\n"
            +"begin_pos = get_actual_tcp_pose()" +"\n"
            +"pos_end = pose_add(begin_pos, p[{}, {}, 0.0, 0.0, 0.0, 0.0])".format(xx, yy) +"\n"
            +"movel(pos_end , a=0.1, v={})".format(vel) + "\n"
            +"end" +"\n"))
        self.s.close()
        rospy.sleep(1)

    def touch(self, z):
        print "touch"
        zz = -z
        self.checkCollision(zz)
        self.initConnection()
        self.s.send(self.toBytes(
            "def myProg():"+"\n"
            +"begin_pos = get_actual_tcp_pose()" +"\n"
            +"pos_end = pose_add(begin_pos, p[0.0, 0.0, {}, 0.0, 0.0, 0.0])".format(zz) +"\n"
            +"movel(pos_end , a=0.1, v=0.1)" + "\n"
            +"end" +"\n"))
        self.s.close()
        rospy.sleep(2)

    def halt(self):
        print "halt"
        self.initConnection()
        self.s.send(self.toBytes(
            "stopl(a=0.5)" +"\n"))
        self.s.close()
        # rospy.sleep(3)
        
    def resetPose(self):
        print "reset"
        self.setJointAngle(self.angle_reset)
        self.sum_z = None
        rospy.sleep(5)

    def initPose(self):
        print "init"
        self.setJointAngle(self.angle_init)
        self.sum_z = 0
        rospy.sleep(3)

    def tablePose(self):
        print "table"
        self.setJointAngle(self.angle_table)
        self.sum_z = 0
        rospy.sleep(2)

    def initGritPose(self, angle):
        print "init grit"
        self.setJointAngle(angle)
        self.sum_z = -0.3
        rospy.sleep(3)

    def changePlace(self, samples, trial):
        self.moveCartesianSpace(0, 0.02/(samples/2)*(trial-samples/2), 0)
        rospy.sleep(2)

    def inputPosition(self):
        x = raw_input("x: ")
        y = raw_input("y: ")
        z = raw_input("z: ")
        self.moveCartesianSpace(float(x), float(y), float(z))
        print "z = {}".format(self.sum_z)
        rospy.sleep(3)


if __name__ == '__main__':
    UR5 = socketController()

    UR5.resetPose()
    UR5.tablePose()
    while not rospy.is_shutdown():
        UR5.inputPosition()

    """
    cmd = "get_actual_tcp_pose()\n"
    socket.send(cmd)
    a = socket.recv(1024)
    socket.close()
    print(repr(self.s.recv(1024)))
    sys.exit()
    """
