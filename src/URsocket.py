import socket
import struct
import time

class URSocket(object):
    def __init__(self, ip='192.168.11.3', port=30003):
        '''URSocket: Socket communicator with UR
            ip  : robot_IP address
                - UR5 : 192.168.11.3
            port: serior port
                - UR5                : 30001(primary), 30002 (Secondary), 30003 (Real-time)
                - robotiqGripper     : 63352
                - robotiqFT300 sensor: 63351
            '''
        self.HOST = ip
        self.PORT = port
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client.connect((self.HOST, self.PORT))

    def reset_socket(self):
        self.client.close()
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client.connect((self.HOST, self.PORT))


if __name__ == "__main__":
    def toBytes(s):
        return bytes(s.encode())
    try:
        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client.connect(('192.168.11.3', 30002))
        client.send(toBytes("get_actual_tcp_pose\n"))
        time.sleep(3)
        res = client.recv(1024)
        print(repr(res))
        print("hoge")
    except Exception as e:
        print(e)
    finally:
        client.close()
