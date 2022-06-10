from pyexpat.errors import XML_ERROR_PARAM_ENTITY_REF
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from hiro_dynamixel.solvers import IKSolver
from hiro_dynamixel.litearm import ArmConfig
from hiro_dynamixel.pid import PIDControl

from dynamixel_sdk_custom_interfaces.msg import SetPosition
from dynamixel_sdk_custom_interfaces.srv import GetPosition


# def setup_robot(config):
#     print("Loading robot from json...")
#     robot = pypot.robot.from_json(config)
#     robot.servos = [robot.swing, robot.shoulder, robot.elbow, robot.wrist]
#     for s in robot.servos:
#         s.compliant = False
#         s.compliance_margin = (0,0)
#     robot.swing.compliance_slope = (63,63)
#     robot.shoulder.compliance_slope = (16,255)
#     robot.elbow.compliance_slope = (255,16)
#     robot.wrist.moving_speed = 1024
#     robot.wrist.compliant = False
#     robot.swing.moving_speed = 14
#     robot.shoulder.moving_speed = 14
#     robot.elbow.moving_speed = 14
#     print("done.")
#     return robot

def setup_solver():
    print("Initializing solver...")
    arm_config = ArmConfig()
    arm_config.main_length = 148.4
    arm_config.forearm_length = 160
    arm_config.linkage_length = 155
    arm_config.lower_actuator_length = 65
    arm_config.upper_actuator_length = 54.4
    arm_config.wrist_length = 90.52
    arm_config.shoulder_offset = [-9.7, 18.71]
    iksolver = IKSolver(
        arm_config.main_length,
        arm_config.forearm_length,
        arm_config.wrist_length,
        arm_config.shoulder_offset,
        arm_config
    )
    print("done.")
    return iksolver


class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        # create a subscriber for cartesian pose commands
        # self.pose_sub = self.create_subscription(Pose, 'set_robot_pose', self.set_pose_callback)
        self.ik_solver = setup_solver()
        self.base_pid = PIDControl(1, 0.0001, 0.0001)
        self.shoulder_pid = PIDControl(2.0, 0.03, 0.05)
        self.elbow_pid = PIDControl(1, 0.001, 0.001)
        self.controllers = {1:self.base_pid, 2:self.shoulder_pid, 3:self.elbow_pid}
        # create a publisher to set the base motor
        self.set_base_position_publisher = self.create_publisher(SetPosition, 'set_base_controller_position',1)
        # create a publisher to set the arm motors, one for the shoulder, and one for the elbow
        self.set_arm_position_publisher = self.create_publisher(SetPosition, 'set_arm_controller_position',1)

        self.set_joint_angle_subscriber = self.create_subscription(SetPosition, 'set_hiro_joint_angle', self.set_target,1)
        # create a client to get the current position of the base motor
        # self.get_base_position_service = self.create_client(GetPosition, 'get_base_controller_position')
        # create a client to get the current position of the arm motors
        # self.get_arm_position_service = self.create_client(GetPosition, 'get_arm_controller_position')
        self.pid_subscriber = self.create_subscription(SetPosition,'hiro_joint_angles', self.pid_step,1)


    def pid_step(self, motor_pose):
        '''
        When we get a new pose msg for a motor, try to update to target pose
        '''
        motor_id = motor_pose.id
        current_angle = motor_pose.position
        if not self.controllers[motor_id].initialized:
            self.controllers[motor_id].initialize(current_angle)
            return
        else:
            output = self.controllers[motor_id].stepOutput(current_angle)
            msg = SetPosition()
            msg.id = motor_id
            msg.position = int(output+current_angle)
            if motor_id == 1:
                self.set_base_position_publisher.publish(msg)
            else:
                self.set_arm_position_publisher.publish(msg)
            if msg.id == 2:
                self.get_logger().info(f'Motor {msg.id} current: {current_angle}, target: {self.controllers[motor_id].target}, control: {msg.position}')

    def set_target(self, position):
        motor_id = position.id
        if self.controllers[motor_id].initialized:
            self.controllers[motor_id].setTarget(position.position)
        else:
            self.get_logger().error(f"Tried to set angle on uninitialized motor {motor_id}.")

    def set_pose_callback(self, pose):
        goal = [pose.x, pose.y, pose.z]
        self.ik_solver.setGoal(goal)
        self.ik_solver.resolveIK()
        swing = self.ik_solver.getSwing()
        shoulder = self.ik_solver.getShoulder()
        elbow = self.ik_solver.getElbow()

        # update the PID target
        self.base_pid.setTarget(swing)
        self.shoulder_pid.setTarget(shoulder)
        self.elbow_pid.setTarget(elbow)

    def get_current_pose(self):
        self.base_position_future = self.get_base_position_service.call_async().position
        self.shoulder_position_future = self.get_arm_position_service.call_async().position
        self.elbow_position_future = self.get_arm_position_service.call_async().position
        self.futures = [self.base_position_future, self.shoulder_position_future, self.elbow_position_future]

def main():
    rclpy.init()
    robot_controller = RobotController()

    rclpy.spin(robot_controller)

    rclpy.shutdown()

    # def main_loop(self):

        # while true, 
        # get the current positions of the motors
        # update the PID using the current

        
        # self.get_current_pose()
        # while rclpy.ok():
        #     rclpy.spin_once(robot_controller)
        #     if all([f.done for f in robot_controller.base_position_futures]):
        #         try:
        #             response = robot_controller.future.result()
        #         except Exception as e:
        #             robot_controller.get_logger().info(
        #                 'Service call failed %r' % (e,))
        #         else:
        #             robot_controller.get_logger().info(
        #                                             # CHANGE
        #                 (robot_controller.req.a, robot_controller.req.b, robot_controller.req.c, response.sum))  # CHANGE
        #         break

        # robot_controller.destroy_node()
        # rclpy.shutdown()

    # get the current positions of the base and arm motors
    
    # futures = [base_position_future, shoulder_position_future, elbow_position_future]


    # def goto_pose(self,pose,pid=False):
    #     goal = [pose.x,pose.y,pose.z]
    #     self.ik_solver.setGoal(goal)
    #     self.ik_solver.resolveIK()
        
    #     swing = self.ik_solver.getSwing()
    #     if pid:

    #     # set base
    #     self.base_position_publisher.publish(SetPosition(1,self.ik_solver.swing))
    #     # set shoulder
    #     self.arm_position_publisher.publish(SetPosition(2,self.ik_solver.shoulder_angle))
    #     # set elbow
    #     self.arm_position_publisher.publish(SetPosition(3,self.ik_solver.elbow_angle))


    

    
