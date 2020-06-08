#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan

def callback(data):
    #print("{1}, {2}, {3}, {4}".format(data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w))
    #print(laser.ranges)
    print(data.pose.orientation.x)
def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    data = PoseStamped()
    laser = LaserScan()
    laser.ranges
    data.pose.orientation
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()