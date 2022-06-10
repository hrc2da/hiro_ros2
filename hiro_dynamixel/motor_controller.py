from pyexpat.errors import XML_ERROR_PARAM_ENTITY_REF
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from dynamixel_sdk_custom_interfaces.msg import SetPosition
from dynamixel_sdk_custom_interfaces.srv import GetPosition
from dynamixel_sdk_python import PortHandler, PacketHandler, GroupBulkRead                    # Uses Dynamixel SDK library
from hiro_msgs.msg import HiroPose, HiroSpeed
import os
import yaml
import serial.tools.list_ports
import pdb




# if os.name == 'nt':
#     import msvcrt
#     def getch():
#         return msvcrt.getch().decode()
# else:
#     import sys, tty, termios
#     fd = sys.stdin.fileno()
#     old_settings = termios.tcgetattr(fd)
#     def getch():
#         try:
#             tty.setraw(sys.stdin.fileno())
#             ch = sys.stdin.read(1)
#         finally:
#             termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
#         return ch



COMM_SUCCESS                = 0
TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 400           # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 500            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

class MotorController(Node):
    def __init__(self, motors, name):
        super().__init__('motor_controller')
        self.motors = motors
        self.motor_dict = {m['id']: m for m in self.motors}
        self.name = name
        self.v1_packet_handler = PacketHandler(1)
        self.v2_packet_handler = PacketHandler(2)
        self.packet_handlers = { 1: self.v1_packet_handler, 2: self.v2_packet_handler }
        self.protocols = list(set([m['protocol_version'] for m in motors]))
        if len(self.protocols) > 1:
            self.protocol = None
            self.packet_handler = None
        else:
            self.protocol = self.protocols[0]
            self.packet_handler = self.packet_handlers[self.protocol]

        
        # all the motors on this controller should use the same port. find it if it isn't specified.
        # if 'port' not in motor or motor['port'] is None:
        if not self.find_port():
            self.get_logger().info('Could not find port for controller %s' % name)
            return
        
        for motor in self.motors:
            self.set_speed(motor['id'],motor['moving_speed'])
        # so only MX motors support groupbulkread on protocol 1. I am going to ignore it for now.
        # if self.protocol:
        #     self.groupBulkRead = GroupBulkRead(self.port_handler,self.packet_handler)
        #     # Add parameter storage for each motor's current angle
        #     for motor in self.motors:
        #         dxl_addparam_result = self.groupBulkRead.addParam(motor['id'],motor['PRESENT_POSITION']['ADDR'],motor['PRESENT_POSITION']['LEN'])
        #         if dxl_addparam_result != True:
        #             print("[ID:%03d] groupBulkRead addparam failed" % motor['id'])
        #             return


        # self.setAngleSubscriber = self.create_subscription(SetPosition, f'set_{name}_position', self.set_angle_callback,10)
        # self.getAngleService = self.create_service(GetPosition, f'get_{name}_position', self.get_angle_callback)
        self.anglePublisher = self.create_publisher(SetPosition, f'hiro_joint_angles', 1) # queue size of 1

        self.motor_speed_subscriber = self.create_subscription(HiroSpeed, 'hiro_joint_speed', self.motor_speed_callback, 10)
        self.pose_subscriber = self.create_subscription(HiroPose, 'pose_target', self.set_angle_callback,1)
        self.timer_period = 0.05 # update joint angles at 20 hz
        self.create_timer(self.timer_period, self.publish_angles)
        # Open port
        # note that all communication over this port must happen in this node b/c I can't overload the USB connection
        # basically the most I can break down the logic is to have a separate node for each U2D2
        # and if all the motors are on a shared bus, I can only have one shared motor node
        # if self.port_handler.openPort():
        #     print("Succeeded to open the port")
        # else:
        #     print("Failed to open the port")
    def set_speed(self,m_id,speed):
        self.motor_dict[m_id]["moving_speed"]=speed
        target_motor = self.motor_dict[m_id]
        protocol = target_motor['protocol_version']
        packet_handler = self.packet_handlers[protocol]
        dxl_comm_result, dxl_error = packet_handler.write2ByteTxRx(self.port_handler, 
                                                                                m_id, target_motor['MOVING_SPEED']['ADDR'], 
                                                                                speed)
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().info(f'Failed to set angle for motor {m_id} on {self.name}')

    def find_port(self, port=None):
        # we assume all the motors are on the same port
        # so just look for the port with the first motor on it
        dxl_id = self.motors[0]['id']
        protocol = self.motors[0]['protocol_version']
        ports = list(serial.tools.list_ports.comports())
        if port is not None and port < len(ports):
            temp_port_handler = PortHandler(ports[port].device)
            if temp_port_handler.openPort():
                # check if the motor id is on this port; if it is, then stay
                model_number, comm_result, comm_error = self.packet_handlers[protocol].ping(temp_port_handler, dxl_id)
                if comm_result == COMM_SUCCESS:
                    self.port_handler = temp_port_handler
                    self.port = p.device
                    self.get_logger().info(f'Opening port {p.device} for controller {self.name}')
                    return True
            return False
        for p in ports:
            # open the port
            temp_port_handler = PortHandler(p.device)
            if temp_port_handler.openPort():
                # check if the motor id is on this port; if it is, then stay
                try:
                    model_number, comm_result, comm_error = self.packet_handlers[protocol].ping(temp_port_handler, dxl_id)
                except serial.SerialException:
                    continue
                if comm_result == COMM_SUCCESS:
                    self.port_handler = temp_port_handler
                    self.port = p.device
                    self.get_logger().info(f'Opening port {p.device} for controller {self.name}')
                    return True
        self.get_logger().info('Could not find port in %s' % ports)
        return False

    def set_angle_callback(self, msg):
        for motor in self.motors:
            try:
                target_angle = getattr(msg, motor['name'])
            except AttributeError:
                continue # the message doesn't have a target angle for this motor
            encoded_angle = motor['encoder'](target_angle)
            self.get_logger().info(f"Incoming set angle request for motor {motor['name']} on {self.name}: ({target_angle},{encoded_angle})")
            packet_handler = self.packet_handlers[motor['protocol_version']]
            dxl_comm_result, dxl_error = packet_handler.write2ByteTxRx(self.port_handler, 
                                                                                motor['id'], motor['GOAL_POSITION']['ADDR'], 
                                                                                encoded_angle)
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().info(f"Failed to set angle for motor {motor['name']} on {self.name}")
        
    def motor_speed_callback(self, msg):
        speed = msg.speed
        dxl_id = None
        if msg.name != '':
            for motor in self.motors:
                if motor['name'] == msg.name:
                    dxl_id = motor['id']
                    target_motor = motor
                    break
            if dxl_id is None:
                self.get_logger().info(f'Failed to find motor {dxl_id} on {self.name}')
                return
        else:
            dxl_id = msg.id
            try:
                target_motor = self.motor_dict[dxl_id]
            except KeyError:
                self.get_logger().info(f'Failed to find motor {dxl_id} on {self.name}')
                return
        packet_handler = self.packet_handlers[target_motor['protocol_version']]
        dxl_comm_result, dxl_error = packet_handler.write2ByteTxRx(self.port_handler, dxl_id, target_motor['MOVING_SPEED']['ADDR'], speed)
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().info(f"Failed to set speed for motor {target_motor['name']} on {self.name}")

    def get_angle_callback(self, request, response):
        dxl_id = request.id
        try:
            target_motor = self.motor_dict[dxl_id]
        except KeyError:
            self.get_logger().info(f'Failed to find motor {dxl_id} on {self.name}')
            response.position = None
            return response
        packet_handler = self.packet_handlers[target_motor['protocol_version']]
        dxl_present_position, dxl_comm_result, dxl_error = packet_handler.read2ByteTxRx(self.port_handler, dxl_id, target_motor['PRESENT_POSITION']['ADDR'])
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().info(f'Failed to get angle for motor {dxl_id} on {self.name}')
            response.position = None
            return response
        else:
            response.position = dxl_present_position
            self.get_logger().info('Incoming request\nbase_angle: %d' % response.position)
            return response

    def publish_angles(self):
        # here I want to use the batch read
        # assume that all motors are using the same protocol for now
        # in the future we can check which packet handler to use
        # dxl_comm_result = self.groupBulkRead.txRxPacket()
        # if dxl_comm_result != COMM_SUCCESS:
        #     print("%s" % self.packet_handler.getTxRxResult(dxl_comm_result))

        # Check if groupbulkread data of Dynamixel#1 is available
        for motor in self.motors:
            position, success, _ = self.packet_handler.read2ByteTxRx(self.port_handler, motor['id'], motor['PRESENT_POSITION']['ADDR'])
            # dxl_getdata_result = self.groupBulkRead.isAvailable(motor['id'], motor['PRESENT_POSITION']['ADDR'],motor['PRESENT_POSITION']['LEN'])
            if success != 0:
                self.get_logger().error(f'Position read failed for motor {motor["id"]}')
            else:
                msg = SetPosition()
                msg.id = motor['id']
                msg.position = position
                self.anglePublisher.publish(msg)


                

