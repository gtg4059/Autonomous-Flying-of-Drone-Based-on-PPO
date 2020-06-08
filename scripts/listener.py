#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan

# setpoint = PoseStamped()
# setpioit.pose.orientation.x=10
# pub_setpoint = rospy.Publisher("mavros/setpoint_position/local",PoseStamped)

def callback(data):    
    #print("{}, {}, {}, {}".format(data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w))
    self.str = data.pose.orientation.x
    print(self.str)
    #asdasdsdasdsadsad

    
    #print(data.pose.orientation.x)
    #print(c.avg_resultx, c.avg_resulty)
    
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
    rospy.spin()


if __name__ == '__main__':  
    listener() 
            
            # define set point

            # pub_setpoint.publish(setpoint)
    #listener()
