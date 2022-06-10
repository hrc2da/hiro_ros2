# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from hiro_dynamixel.litearm import ArmConfig
from hiro_dynamixel.util import radians, degrees, setup_params
from hiro_dynamixel.util import shoulder_servo_pose, elbow_servo_pose, swing_servo_pose, wrist_servo_pose
from hiro_dynamixel.solvers import IKSolver
from hiro_msgs.msg import HiroPose
from std_msgs.msg import String



import json




class HIROTargetSetter(Node):

    def __init__(self):
        super().__init__('cartesian_target_subscriber')
        params = self.declare_parameters('hiro_dynamixel',
            [('main_length', 148.4),
             ('forearm_length', 160.0),
             ('linkage_length', 155.0),
             ('lower_actuator_length', 65.0),
             ('upper_actuator_length', 54.4),
             ('wrist_length', 90.52),
             ('shoulder_offset', [-9.7, 18.71]),
             ('box_width_mm', 0.0),
             ('box_height_mm', 0.0),
             ('x_offset_mm', 0.0),
             ('y_offset_mm', 0.0)])
        setup_params(self,params)
        self.x_offset_mm -= 0.5 * self.box_width_mm
        self.solver = self.setup_solver()
        self.target_subscriber = self.create_subscription(
            String,
            'cartesian_target',
            self.cartesian_target_callback,
            10
        )
        self.pose_publisher = self.create_publisher(
            HiroPose,
            'pose_target',
            10
        )
    def setup_solver(self):
        print("Initializing solver...")
        self.arm_config = ArmConfig()
        self.arm_config.main_length = self.main_length
        self.arm_config.forearm_length = self.forearm_length
        self.arm_config.linkage_length = self.linkage_length
        self.arm_config.lower_actuator_length = self.lower_actuator_length
        self.arm_config.upper_actuator_length = self.upper_actuator_length
        self.arm_config.wrist_length = self.wrist_length
        self.arm_config.shoulder_offset = self.shoulder_offset
        iksolver = IKSolver(
            self.arm_config.main_length,
            self.arm_config.forearm_length,
            self.arm_config.wrist_length,
            self.arm_config.shoulder_offset,
            self.arm_config
        )
        print("done.")
        return iksolver

    def target2pose(self,x,y,z,theta):

        return

    def cartesian_target_callback(self, msg):
        target = json.loads(msg.data)
        x = (1-target['x'])*self.box_width_mm + self.x_offset_mm
        y = target['y']*self.box_height_mm + self.y_offset_mm
        z = target['z']
        self.get_logger().info(f'Got target ({x},{y},{z}) mm.')
        theta = target['theta']
        goal = (x,z,y) #y is height in solver convention
        self.solver.setGoal(goal)
        swing = swing_servo_pose(degrees(self.solver.swing))
        shoulder = shoulder_servo_pose(degrees(self.solver.shoulder_angle))
        elbow = elbow_servo_pose(degrees(self.solver.elbow_angle))
        wrist = wrist_servo_pose(theta)
        msg = HiroPose()
        msg.swing = float(swing)
        msg.shoulder = float(shoulder)
        msg.elbow = float(elbow)
        msg.wrist = float(wrist)
        self.get_logger().info(f'Raw pose swing:{degrees(self.solver.swing)},shoulder:{degrees(self.solver.shoulder_angle)},elbow:{degrees(self.solver.elbow_angle)}')
        
        self.get_logger().info('Publishing: "%s"' % msg)
        self.pose_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    target_setter = HIROTargetSetter()
    try:
        rclpy.spin(target_setter)
    except KeyboardInterrupt:
        pass
    
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        target_setter.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
