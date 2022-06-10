from pyexpat.errors import XML_ERROR_PARAM_ENTITY_REF
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from dynamixel_sdk import PortHandler, PacketHandler                    # Uses Dynamixel SDK library
from hiro_dynamixel.motor_controller import MotorController
from hiro_dynamixel.util import base_angle_encoder

import os
import yaml
import pdb
from copy import deepcopy

def main():
    with open('/home/dev/robotis_ws/src/hiro_dynamixel/hiro_dynamixel/motors.yaml', 'r') as motor_config_file:
        motors = yaml.safe_load(motor_config_file)

    base_motor = deepcopy(motors['AX'])
    # lowercase are values that I assign
    # uppercase are addresses
    base_motor['id'] = 1
    base_motor['port'] = None
    base_motor['baudrate'] = 1000000    
    base_motor['moving_speed'] = 32
    base_motor['encoder'] = base_angle_encoder
    base_motor['name'] = 'swing'
    
    rclpy.init()
    
    base_controller = MotorController([base_motor],'base_controller')

    rclpy.spin(base_controller)

    rclpy.shutdown()
