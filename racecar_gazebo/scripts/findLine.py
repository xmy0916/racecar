#!/usr/bin/env python
# BEGIN ALL
import rospy,cv2,cv_bridge, numpy
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
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_yellow = numpy.array([ 0,  0, 100])
    upper_yellow = numpy.array([180, 30, 255])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    #cv2.imshow("window2", mask)
    #cv2.imshow("window", image)
    h, w, d = image.shape
    search_top = h/2 + 50
    search_bot = h - 20
    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0
    M = cv2.moments(mask)
    if M['m00'] > 0:
      cx = int(M['m10']/M['m00'])
      cy = int(M['m01']/M['m00'])
      cv2.circle(mask, (cx, cy), 20, (0,0,255), -1)
      # BEGIN CONTROL
      err = cx - w/2
      self.ack.drive.speed = 0.5
      angle = -float(err) / 25
      max_angle = 0.6
      if angle > max_angle :
	angle = max_angle
      elif angle < -max_angle:
	angle = -max_angle
      
      self.ack.drive.steering_angle = angle
      self.cmd_vel_pub.publish(self.ack)

      msg = Float64()
      msg.data = err
      self.err_pub.publish(msg)
      cv2.imshow("window2", mask)
      # END CONTROL
    
    cv2.waitKey(3)

rospy.init_node('follower')
follower = Follower()
rospy.spin()
# END ALL
