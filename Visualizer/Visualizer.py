import bge
from collections import OrderedDict
import socket
from random import randint
import threading
import bpy
from mathutils import *
from math import *
import time

HOST = 'localhost'
PORT = 1234
BUFFER_SIZE = 1024
ax = ay = az = 0

bones = bpy.data.objects['SMPLX-male'].pose.bones

pose = {
    'pelvis': (0, 0, 0),
    'spine1': (0, 0, 0), 
    'head': (0, 0, 0), 
    'left_hip': (0, 0, 0), 
    'right_hip': (0, 0, 0), 
    'left_knee': (0, 0, 0), 
    'right_knee': (0, 0, 0), 
    'left_shoulder': (0, 0, 0), 
    'right_shoulder': (0, 0, 0), 
    'left_elbow': (0, 0, 0), 
    'right_elbow': (0, 0, 0)
}

def socket_listener():
    global ax, ay, az

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as server_socket:
        server_socket.bind(('', PORT))
        print(f'Listening on port {PORT}')

        while True:
            data, addr = server_socket.recvfrom(BUFFER_SIZE)
            data = data.decode().split('\n')[-2].split(',')
            body_segment, *orientation = data
            ax, ay, az = [float(angle) for angle in orientation]
            if body_segment not in pose:
                print("such body segment either doesn't exist or it's not supported")
                continue
            pose[body_segment] = (ax, ay, az)

        
def rotate_body_segment(body_segment, orientation):
    if body_segment not in pose:
        print("such body segment either doesn't exist or it's not supported")
        return
    orientation_radians = tuple(radians(angle) for angle in orientation_degrees)
    bones[body_segment].rotation_euler = Euler(orientation_radians, 'XYZ')
    print(f"{body_segment} orient {orientation_radians}")
    print(f"Orientation: {orientation_radians}, Body Segment: {body_segment}")


class HumanVisualizer(bge.types.KX_PythonComponent):
    args = OrderedDict([
    ])

    def start(self, args):
        for body_segment in pose.keys():  
            bones[body_segment].rotation_mode = 'XYZ'
        thread = threading.Thread(target=socket_listener, daemon=True)
        thread.start()
        
    def update(self):
        for body_segment, orientation in pose.items():  
            rotate_body_segment(body_segment, orientation)
