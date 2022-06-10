from pyexpat.errors import XML_ERROR_PARAM_ENTITY_REF
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from dynamixel_sdk_custom_interfaces.msg import SetPosition
from dynamixel_sdk_custom_interfaces.srv import GetPosition
from dynamixel_sdk_python import PortHandler, PacketHandler                    # Uses Dynamixel SDK library
from hiro_dynamixel.motor_controller import MotorController

import os
import yaml
import pdb
from copy import deepcopy
from hiro_dynamixel.util import shoulder_angle_encoder, elbow_angle_encoder

def main():
    with open('/home/dev/robotis_ws/src/hiro_dynamixel/hiro_dynamixel/motors.yaml', 'r') as motor_config_file:
        motors = yaml.safe_load(motor_config_file)

    shoulder_motor = deepcopy(motors['RX'])
    # lowercase are values that I assign
    # uppercase are addresses
    shoulder_motor['id'] = 2
    shoulder_motor['port'] = None
    shoulder_motor['baudrate'] = 1000000
    shoulder_motor['moving_speed'] = 25
    shoulder_motor['name'] = 'shoulder'
    shoulder_motor['encoder'] = shoulder_angle_encoder

    elbow_motor = deepcopy(motors['RX'])
    elbow_motor['id'] = 3
    elbow_motor['port'] = None
    elbow_motor['baudrate'] = 1000000
    elbow_motor['moving_speed'] = 14
    elbow_motor['name'] = 'elbow'
    elbow_motor['encoder'] = elbow_angle_encoder
    
    
    rclpy.init()
    
    arm_controller = MotorController([shoulder_motor, elbow_motor],'arm_controller')

    rclpy.spin(arm_controller)

    rclpy.shutdown()
