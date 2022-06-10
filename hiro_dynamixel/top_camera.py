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

from cv2 import VideoCapture
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import cv2
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from sensor_msgs.msg import Image
from tutorial_interfaces.msg import Num
from dynamixel_sdk_custom_interfaces.msg import SetPosition
from dynamixel_sdk import PortHandler, PacketHandler                    # Uses Dynamixel SDK library
import glob
from simplejpeg import encode_jpeg
import base64
import numpy as np
from hiro_dynamixel.util import setup_params

class TopCamera(Node):

    def __init__(self):
        super().__init__('top_camera')
        params = self.declare_parameters('hiro_dynamixel',
            [('box_xmin_px', 0),
             ('box_ymin_px', 0),
             ('box_xmax_px', 0),
             ('box_ymax_px', 0),
             ('box_width_mm', 0.0),
             ('box_height_mm', 0.0)])
        setup_params(self,params)
        self.box_width_px = self.box_xmax_px-self.box_xmin_px
        self.box_height_px = self.box_ymax_px-self.box_ymin_px
        self.px_to_mm = np.mean([self.box_width_mm / self.box_width_px, self.box_height_mm / self.box_height_px])
        self.get_logger().info(f'px to mm: {self.px_to_mm}')

        self.cvbridge = CvBridge()
        
        self.top_jpg_publisher = self.create_publisher(String, 'encoded_top_view', 10)
        self.cropped_top_jpg_publisher = self.create_publisher(String, 'encoded_top_view_cropped', 10)

        self.files = list(glob.glob('/home/dev/rock_garden_img/jpg/top*.jpg'))
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.setup_camera()
        self.i = 0

    def setup_camera(self):
        # open a cv2 VideoCapture object
        self.camera = VideoCapture(2)

    def timer_callback(self):
        # msg = SetPosition()
        # msg.id = self.i
        # msg.position = 0
        # self.publisher_.publish(msg)
        # img = cv2.imread(self.files[self.i%len(self.files)])
        ret, img = self.camera.read()
        # note that these are swapped x/y for numpy ordering
        cropped_img = np.asarray(img[self.box_ymin_px:self.box_ymax_px,self.box_xmin_px:self.box_xmax_px],order='C')
        
        img_encoded = encode_jpeg(img, colorspace='bgr')
        cropped_img_encoded = encode_jpeg(cropped_img, colorspace='bgr')
        #base64 encode the img
        img_encoded = base64.b64encode(img_encoded).decode('utf-8')
        cropped_img_encoded = base64.b64encode(cropped_img_encoded).decode('utf-8')
        # try:
        #     self.image_publisher.publish(self.cvbridge.cv2_to_imgmsg(img, "bgr8"))
        # except CvBridgeError as e:
        #     print(e)
        strmsg = String()
        strmsg.data = img_encoded
        self.top_jpg_publisher.publish(strmsg)

        strmsg.data = cropped_img_encoded
        self.cropped_top_jpg_publisher.publish(strmsg)
        # self.get_logger().info(f'Publishing: {msg.id}:{msg.position}')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    top_camera = TopCamera()
    try:
        rclpy.spin(top_camera)
    except KeyboardInterrupt:
        top_camera.get_logger().info("Shutting down top camera!")
        pass
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    finally:
        top_camera.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
