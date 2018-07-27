#!/usr/bin/env python
import rospy
import tf2_ros
import tf.transformations
from nav_msgs.msg          import Path
from geometry_msgs.msg     import PoseStamped
class plot_path_node:

  def __init__(self):
    self.params()
    self.pubs()
    self.tf_buffer   = tf2_ros.Buffer()
    self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

  def params(self):
    self.frame_from = rospy.get_param('~frame_from', 'map')
    self.frame_to   = rospy.get_param('~frame_to', 'base_link')

  def pubs(self):
    self.pub = rospy.Publisher('path', Path, queue_size=1)
    self.path = Path()
    self.path.header.frame_id = self.frame_from

  def loop(self):
    try:
      trans = self.tf_buffer.lookup_transform(self.frame_from,self.frame_to, rospy.Time())
      pose = PoseStamped();
      pose.header.frame_id = self.frame_from
      pose.header.stamp = rospy.get_rostime()
      pose.pose.position.x = trans.transform.translation.x
      pose.pose.position.y = trans.transform.translation.y
      pose.pose.orientation = trans.transform.rotation
      self.path.poses.append(pose)
      self.pub.publish(self.path)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
      return


if __name__ == '__main__':
  rospy.init_node('plot_path', anonymous=True)
  obj = plot_path_node()
  r = rospy.Rate(5)  #Hz
  while not rospy.is_shutdown():
    obj.loop()
    r.sleep()
  #rospy.spin()
