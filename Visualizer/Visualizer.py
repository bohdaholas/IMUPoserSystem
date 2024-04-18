import bge
import bpy
from mathutils import Quaternion
import socket
import threading
import math
from enum import Enum
class Axis(Enum):
    X = 1
    Y = 2
    Z = 3

class Human:
    def __init__(self):
        self.bones = bpy.data.objects['SMPLX-male'].pose.bones
        self.joints_quaternion_orientation = {
            'pelvis': (1, 0, 0, 0),
            'spine1': (1, 0, 0, 0),
            'head': (1, 0, 0, 0),
            'left_hip': (1, 0, 0, 0),
            'right_hip': (1, 0, 0, 0),
            'left_knee': (1, 0, 0, 0),
            'right_knee': (1, 0, 0, 0),
            'left_shoulder': (1, 0, 0, 0),
            'right_shoulder': (1, 0, 0, 0),
            'left_elbow': (1, 0, 0, 0),
            'right_elbow': (1, 0, 0, 0)
        }
    def apply_quaternion_rotation(self, joint: str, orientation: tuple):
        self.bones[joint].rotation_quaternion = Quaternion(orientation)

    def apply_around_axis_quaternion_rotation(self, joint:str, axis: Axis, angle: int):
        current_orientation = self.joints_quaternion_orientation[joint]
        q = Quaternion(current_orientation)
        axis_tuple = (0, 0, 0)
        match axis:
            case Axis.X:
                axis_tuple = (1, 0, 0)
            case Axis.Y:
                axis_tuple = (0, 1, 0)
            case Axis.Z:
                axis_tuple = (0, 0, 1)
        q_axis_rotation = Quaternion(axis_tuple, math.radians(angle))
        q @= q_axis_rotation
        self.bones[joint].rotation_quaternion = q
        self.joints_quaternion_orientation[joint] = (q.w, q.x, q.y, q.z)

human = Human()

def socket_listener():
    PORT = 1234
    BUFFER_SIZE = 1024

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as server_socket:
        server_socket.bind(('', PORT))
        print(f'Listening on port {PORT}')

        while True:
            data, addr = server_socket.recvfrom(BUFFER_SIZE)
            data = data.decode().split('\n')[-2].split(',')
            joint, *orientation = data
            w, rx, ry, rz = [float(angle) for angle in orientation]
            if human.joints_quaternion_orientation[joint] != (w, ry, rz, rx):
                print(f"Orientation: {orientation}, Body Segment: {joint}")
                human.joints_quaternion_orientation[joint] = (w, ry, rz, rx)


class HumanVisualizer(bge.types.KX_PythonComponent):
    def start(self, args):
        for joint in human.joints_quaternion_orientation.keys():
            human.bones[joint].rotation_mode = 'QUATERNION'
        thread = threading.Thread(target=socket_listener, daemon=True)
        thread.start() 

    def update(self):
        for joint, orientation in human.joints_quaternion_orientation.items():
            human.apply_quaternion_rotation(joint, orientation)

