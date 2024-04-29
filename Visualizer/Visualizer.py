import bge
import bpy
from mathutils import Quaternion
import math
import time
import socket
import threading
import paho.mqtt.client as mqtt
from paho.mqtt.enums import CallbackAPIVersion

TOPIC = "tpose"
tpose_req_time = None
prev_root_yaw_offset = None

def average_lists(list_of_lists):
    if not list_of_lists:
        return []
    sum_list = [0] * len(list_of_lists[0])
    for inner_list in list_of_lists:
        for index, item in enumerate(inner_list):
            sum_list[index] += item
    average_list = [total / len(list_of_lists) for total in sum_list]
    return average_list

class Human:
    def __init__(self):
        self.bones = bpy.data.objects['SMPLX-male'].pose.bones
        self.joints = ('root', 'spine1', 'head', 'left_hip', 'right_hip', 'left_knee', 'right_knee',
                       'left_shoulder', 'right_shoulder', 'left_elbow', 'right_elbow')
        self.joints_quaternion_orientation = {key: Quaternion((1, 0, 0, 0)) for key in self.joints}
        self.measured_tpose = {key: [] for key in self.joints}
        self.corrective_quaternions = {key: Quaternion((1, 0, 0, 0)) for key in self.joints}

    def apply_quaternion_rotation(self, joint: str, joint_orientation_q: Quaternion):
        self.bones[joint].rotation_quaternion = self.corrective_quaternions[joint] @ joint_orientation_q

    def get_corrective_quaternion(self, q):
        w, rx, ry, rz = q
        q_conj = Quaternion((w, -rx, -ry, -rz))
        return q_conj
    def angle_distance(self, angle1, angle2):
        # Normalizing angles to ensure they are within the range [-180, 180]
        angle1 = (angle1 + 180) % 360 - 180
        angle2 = (angle2 + 180) % 360 - 180

        # Calculate the differences both ways due to circular nature of angles
        diff = (angle2 - angle1 + 360) % 360
        # Since angles are circular, we take the smallest distance
        distance = min(diff, 360 - diff)
        return distance

    def quaternion_to_euler(self, q):
        w, x, y, z = q

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw


    def calc_corrective_quaternions(self):
        global prev_root_yaw_offset
        print(f'***********************************************')
        print('Calculation of yaw offsets and corrective quaternions:')
        for joint in self.joints:
            if not self.measured_tpose[joint]:
                continue
            measured_tpose = average_lists(self.measured_tpose[joint])
            q_corrective = self.get_corrective_quaternion(measured_tpose)

            if joint == 'root':
                euler = self.quaternion_to_euler(measured_tpose)
                roll, pitch, yaw = [math.degrees(x) for x in euler]
                print(f'pitch={pitch:.2f}, roll={roll:.2f}, yaw={yaw:.2f}')
                if prev_root_yaw_offset is not None:
                    # Calc angle distance
                    print(f"Angle distance {self.angle_distance(yaw, prev_root_yaw_offset)}")
                prev_root_yaw_offset = yaw


            self.corrective_quaternions[joint] = q_corrective
            print(f'-----> {joint} {self.corrective_quaternions[joint]}')
        print(f'***********************************************')

    def remap_axis(self, joint, w, rx, ry, rz):
        if joint in ("root", "spine1"):
            return w, -rx, ry, -rz
        if joint == "head":
            return -w, ry, rx, rz
        if joint in ('left_hip', 'right_hip', 'left_knee', 'right_knee'):
            return w, ry, -rx, rz
        if joint in ('left_shoulder', 'left_elbow'):
            return w, rx, rz, -ry
        if joint in ('right_shoulder', 'right_elbow'):
            return w, -rx, rz, ry
        

human = Human()

def socket_listener():
    global tpose_req_time
    UDP_PORT = 1234
    BUFFER_SIZE = 1024
    TPOSE_REQ_DELAY = 2
    TPOSE_STAND_DURATION = 3

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as server_socket:
        server_socket.bind(('', UDP_PORT))
        print(f'Listening on port {UDP_PORT}')

        while True:
            data, addr = server_socket.recvfrom(BUFFER_SIZE)
            data = data.decode().split('\n')[-2].split(',')
            joint, *orientation = data
            orientation = [float(angle) for angle in orientation]
            orientation = human.remap_axis(joint, *orientation)
            orientation_q = Quaternion(orientation)
            if tpose_req_time:
                if time.time() <= tpose_req_time + TPOSE_REQ_DELAY:
                    pass
                elif time.time() <= tpose_req_time + TPOSE_REQ_DELAY + TPOSE_STAND_DURATION:
                    human.measured_tpose[joint].append(orientation)
                else:
                    tpose_req_time = None
                    human.calc_corrective_quaternions()
                    human.measured_tpose = {key: [] for key in human.joints}
            if human.joints_quaternion_orientation[joint] != orientation_q:
                # print(f"{time.time()} Orientation: {orientation}, Body Segment: {joint}")
                angles = human.quaternion_to_euler(orientation_q)
                # print([math.degrees(x) for x in angles])
                human.joints_quaternion_orientation[joint] = orientation_q

def on_connect(client, userdata, conn_flags, rc, props):
    print("Connected with result code " + str(rc))
    print(f"Subsribing to topic: {TOPIC}")
    client.subscribe(TOPIC)

def on_message(client, userdata, msg):
    global tpose_req_time
    if tpose_req_time is None:
        print(f'***********************************************')
        print(f"{time.time()} Tpose request received")
        print(f'***********************************************')
        tpose_req_time = time.time()

def mqtt_communication():
    BROKER_ADDRESS = "localhost"
    MQTT_PORT = 1884

    ver = CallbackAPIVersion.VERSION2
    client = mqtt.Client(ver)
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(BROKER_ADDRESS, MQTT_PORT, 60)
    client.loop_forever()

class HumanVisualizer(bge.types.KX_PythonComponent):
    def start(self, args):
        for joint in human.joints_quaternion_orientation.keys():
            human.bones[joint].rotation_mode = 'QUATERNION'
        sock_listener_th = threading.Thread(target=socket_listener, daemon=True)
        sock_listener_th.start()
        mqtt_manager_th = threading.Thread(target=mqtt_communication, daemon=True)
        mqtt_manager_th.start()

    def update(self):
        for joint, orientation in human.joints_quaternion_orientation.items():
            human.apply_quaternion_rotation(joint, orientation)
        bpy.ops.object.smplx_set_poseshapes()


