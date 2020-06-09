#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import serial
import NGD

anc = [[0.01, 0.01, 1.6],
        [0.01, 7.2, 1.6],
        [4.05, 0.01, 1.6],
        [4.05, 7.2, 1.6],
        [6, 0.01, 1.6],
        [6, 7.2, 1.6]]
ser = serial.Serial(port='/dev/ttyUSB1',baudrate=115200)
c=NGD.CalculatePosition(anc)

def talker():
    pub = rospy.Publisher('UWBPosition', String, queue_size=24)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        if ser.readable():
            res = ser.readline()
            if len(res.decode().split(','))>=6:
                c.SumValues(res.decode()[1:len(res)-1])
        hello_str = "{:f},{:f}".format(c.avg_resultx, c.avg_resulty) 
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass