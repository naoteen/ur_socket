#!/usr/bin/env python
import URsocket
import sys
import time

class RobotiqGripper(URsocket.URSocket):
    def __init__(self):
        URsocket.URSocket.__init__(self, ip='192.168.11.3', port=63352)
        # Grippers Settings
        self.SPEED = 200  #
        # 0 - deformable, very fragile obj., 255 - solid & strong obj. (if FORCE == 0, Re-grasp is off)
        self.FORCE = 0

        self.min_pos = 0   # mm
        self.init_gripper()

    def hard_grasp(self):
        self.set_pos(10)
        while self.get_obj_detect() == 0:
            time.sleep(0.1)

    def release(self):
        self.set_pos(49)
        while self.get_obj_detect() == 0:
            time.sleep(0.1)

    def command(self, pos):
        if pos < self.min_pos:
            pos = self.min_pos

        self.set_pos(pos)
        while self.get_obj_detect() == 0:
            time.sleep(0.1)

    def get_pos(self):
        res = self.__get('POS')
        print self.__res_to_value(res)

    def set_pos(self, mm):
        value = self.__convert_to_msg(mm)
        self.__set('POS', int(value))

    def __convert_to_msg(self, mm):
        """
        mm(0mm - 50mm) -> range(0 - 255)
        """
        msg = int(self.min_pos_range - mm / (40.0/255))
        if msg < 0:
            msg = 0
        return msg

    def get_force(self):
        res = self.__get('FOR')
        return self.__res_to_value(res)

    def set_force(self, value, echo_fl=True):
        if echo_fl:
            print 'Gripper Force is changed ({} -> {})'.format(
                self.get_force(), value)
        self.FORCE = value
        self.__set('FOR', value)

    def get_speed(self):
        res = self.__get('SPE')
        return self.__res_to_value(res)

    def set_speed(self, value, echo_fl=True):
        if echo_fl:
            print 'Gripper Speed is changed ({} -> {})'.format(
                self.get_speed(), value)
        self.SPEED = value
        self.__set('SPE', value)

    def get_obj_detect(self):
        """
        Object detection status
        :return:
            0 : fingers are in motion towards requested position.
            1 : fingers have stopped due to a contact while opening.
            2 : fingers have stopped due to a contact while closing.
            3 : fingers are at requested position.
        """
        res = self.__get('OBJ')
        return self.__res_to_value(res)

    def __set(self, cmd, value):
        self.client.send('SET {} {}\n'.format(cmd, value))
        res, _ = self.client.recvfrom(1024)
        if res == 'ack':
            return True
        else:
            return False

    def __get(self, cmd):
        self.client.send('GET {}\n'.format(cmd))
        res, _ = self.client.recvfrom(1024)
        return res

    @staticmethod
    def __res_to_value(res):
        return res.split(' ')[-1].rstrip()

    def __calibration(self):
        print 'Calibration now...'
        tmp_speed = self.SPEED
        self.set_speed(255)
        # min pos
        self.__set('POS', 255)
        time.sleep(2)
        self.get_pos()
        msg = self.__get('POS')
        self.min_pos_range = int(self.__res_to_value(msg).rstrip())
        # max pos
        self.__set('POS', 0)
        time.sleep(2)
        self.get_pos()
        msg = self.__get('POS')
        self.max_pos_range = int(self.__res_to_value(msg).rstrip())
        self.set_speed(tmp_speed)
        print 'Calibration Finished!!'

    def init_gripper(self):
        self.__set('SPE', self.SPEED)
        self.__set('FOR', self.FORCE)
        self.__calibration()


if __name__ == "__main__":
    gripper = RobotiqGripper()
    gripper.hard_grasp()
    time.sleep(2)
    gripper.release()
