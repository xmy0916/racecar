#!/usr/bin/env python
# BEGIN ALL

import sys
sys.path.append("/home/xmy/.local/lib/python3.5/site-packages")
import rospy,cv2,cv_bridge, numpy
#import paddle as paddle
import paddle.fluid as fluid
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDriveStamped


class Follower:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    #cv2.namedWindow("window", 1)
    self.image_sub = rospy.Subscriber('/camera/zed/rgb/image_rect_color', 
                                      Image, self.image_callback)
    self.cmd_vel_pub = rospy.Publisher(
        "/vesc/low_level/ackermann_cmd_mux/input/teleop", AckermannDriveStamped, queue_size=1)
    self.err_pub = rospy.Publisher('err',
                                       Float64, queue_size=1)  
    self.ack = AckermannDriveStamped()
  def image_callback(self, msg):


      # END CONTROL
    
    cv2.waitKey(3)

rospy.init_node('light_contrl')
follower = Follower()
rospy.spin()
# END ALL
