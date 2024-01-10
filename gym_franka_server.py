import socket
import subprocess
import time
import numpy as np
import pyquaternion as pqt

import omegaconf
from networked_robotics_benchmarking.networks import ZMQ_Pair

SERVER_IP = '10.42.0.67'
SERVER_PORT = 8888
ROBOT_IP = '10.42.0.146'
ROS_INTERFACE_IP = 'localhost'
ROS_INTERFACE_PORT = 6666

BUFFER_SIZE = 1024


class GymFrankaServer:
    def __init__(self):
        self.translation = None
        self.rotation = None

        self.ros_socket = None
        self.ros_process = None

        self.action_history = []

        '''
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.settimeout(1)
        self.server_socket.bind((SERVER_IP, SERVER_PORT))
        self.server_socket.listen(1)
        while True:
            try:
                self.client_connection, self.client_address = self.server_socket.accept()
                print('[Gym Franka Server] Connected!')
                return
            except socket.timeout:
                pass
        '''
        self.net_config = omegaconf.OmegaConf.load("net_config.yaml")
        self.network = ZMQ_Pair("server", **self.net_config)

    def process_request(self):
        '''
        try:
            data = self.client_connection.recv(BUFFER_SIZE)
            if len(data) == 0:
                print('[Gym Franka Server] Connection lost.')
                return False

        except ConnectionResetError as e:
            print('[Gym Franka Server] Connection reset.')
            return False

        data_strings = data.decode('utf8').split(' ')
        '''
        data_strings = self.network.recv("client").split(' ')

        command = data_strings[0]
        try:
            params = np.array([float(s) for s in data_strings[1: -1]])
        except ValueError as e:
            print(f'Failed to parse command: {data_strings}')
            return True

        if command == '<Reset>':
            print('[Gym Franka Server] Reset.')
            self.reset()
            echo_message = f'<Success> {data_strings[1]}'

            #self.client_connection.send(echo_message.encode('utf8'))
            self.network.send("client", echo_message)

        elif command == '<Step-Wait>' or command == '<Step>':
            action = np.array(params[:8])
            print(f'[Gym Franka Server] Step: {list(action)}')
            if not self.is_ready():
                self.wait_for_ready()
            self.step(action)
            if command == '<Step-Wait>':
                time.sleep(0.5)
            reflex = False
            while not self.is_ready():
                reflex = True
                print(f'[Gym Franka Server] Attempting Reflex rollback...')
                self.wait_for_ready()
                self.action_history.pop()
                if len(self.action_history) > 0:
                    self.ros_socket.send(self.action_history[-1].encode('utf8'))
                    time.sleep(0.5)
            if reflex:
                echo_message = f'<Reflex> {data_strings[9]}'
            else:
                echo_message = f'<Success> {data_strings[9]}'
            
            #self.client_connection.send(echo_message.encode('utf8'))
            self.network.send("client", echo_message)

        elif command == '<Grasp>':
            print(f'[Gym Franka Server] Grasp.')
            self.ros_socket.send(b'<Gripper> -1')
        elif command == '<Open>':
            print(f'[Gym Franka Server] Open gripper.')
            self.ros_socket.send(b'<Gripper> 1')
        elif command == '<Close>':
            print('[Gym Franka Server] Shutdown.')
            return False
        
        return True

    def get_robot_mode(self):
        timestamp = time.time_ns()
        query = f'<Mode> {timestamp}'
        try:
            self.ros_socket.send(query.encode('utf8'))
            heard_back = False
            data_strings = []
            while not heard_back:
                response = self.ros_socket.recv(BUFFER_SIZE)
                if len(response) == 0:
                    self.hard_reset()
                    return 0
                data_strings = response.decode('utf8').split(' ')
                if int(data_strings[2]) == timestamp:
                    heard_back = True
            return int(data_strings[1])
        except BrokenPipeError:
            self.hard_reset()
            return 0

    def is_ready(self):
        robot_mode = self.get_robot_mode()
        return robot_mode == 2
    
    def wait_for_ready(self):
        time.sleep(0.1)
        message_printed = False
        while True:
            robot_mode = self.get_robot_mode()
            if robot_mode != 2:
                if not message_printed:
                    print('[Gym Franka Server] Waiting for robot...', end=' ', flush=True)
                    message_printed = True
                print(robot_mode, end=' ', flush=True)
                self.ros_socket.send(b'<Recovery>')
                time.sleep(0.1)
            else:
                if message_printed:
                    print(flush=True)
                return

    def establish_ros_connection(self):
        command = f'gnome-terminal --window --wait -x roslaunch gym_franka_server ci_socket.launch ' \
                  f'robot_ip:={ROBOT_IP} address:={ROS_INTERFACE_IP} port:={ROS_INTERFACE_PORT} client:=gym'
        self.ros_process = subprocess.Popen(command, shell=True, executable='/bin/bash',
                                            stdout=subprocess.DEVNULL,
                                            stderr=subprocess.DEVNULL)
        self.ros_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        attempts = 0
        while True:
            try:
                self.ros_socket.connect((ROS_INTERFACE_IP, ROS_INTERFACE_PORT))
                return True
            except ConnectionRefusedError as e:
                attempts += 1
                if attempts >= 10:
                    print('Connection to rospy node failed after 10 trials.')
                    self.ros_process.terminate()
                    self.ros_process = None
                    self.ros_socket.close()
                    self.ros_socket = None
                    return False
                time.sleep(1)

    def home(self):
        if self.ros_socket is not None:
            self.wait_for_ready()
            self.ros_socket.close()
            self.ros_process.wait()
            self.ros_socket = None
        command = f'gnome-terminal --window --wait -x roslaunch gym_franka_server home.launch robot_ip:={ROBOT_IP}'
        reset_process = subprocess.Popen(command, shell=True, executable='/bin/bash',
                                         stdout=subprocess.DEVNULL,
                                         stderr=subprocess.DEVNULL)
        reset_process.wait()

    def hard_reset(self):
        print('[Gym Franka Server] Fatal error! Hard reset...')
        if self.ros_socket is not None:
            self.ros_socket.close()
        if self.ros_process is not None:
            self.ros_process.kill()
        self.establish_ros_connection()
        self.wait_for_ready()
        self.reset()

    def reset(self):
        self.home()
        self.establish_ros_connection()
        if not self.is_ready():
            self.wait_for_ready()
        self.action_history = []
        self.translation = np.array([0.3, 0, 0.5])
        self.rotation = pqt.Quaternion(0, -1, 0, 0)
        self.ros_socket.send(b'<Start> 0.3 0 0.5 0 -1 0 0')

    def step(self, action):
        self.translation = action[:3]
        self.rotation = pqt.Quaternion(action[3:7])
        ros_socket_message = f'<Track> {action[0]} {action[1]} {action[2]} ' \
                             f'{action[3]} {action[4]} {action[5]} {action[6]} ' \
                             f'{action[7]}'
        self.action_history.append(ros_socket_message)
        self.ros_socket.send(ros_socket_message.encode('utf8'))


if __name__ == '__main__':
    server = GymFrankaServer()
    live = True
    while live:
        live = server.process_request()
