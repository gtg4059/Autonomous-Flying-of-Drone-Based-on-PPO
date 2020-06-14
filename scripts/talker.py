#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import serial
import NGD

anc = [[0.01, 0.01, 1.6],
        [0.01, 7.3, 1.6],
        [3.23, 0.01, 1.6],
        [3.23, 7.3, 1.6],
        [6, 0.01, 1.6],
        [6, 7.2, 1.6]]
ser = serial.Serial(
    port='/dev/ttyUWB',
    baudrate=115200)
ser.xonxoff=1
c=NGD.CalculatePosition(anc)

def talker():
    pub = rospy.Publisher('UWBPosition', String, queue_size=24)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        if ser.readable():
            res = ser.readline()
            if len(res.decode().split(','))>=6:
                c.SumValues(res.decode()[1:len(res)-2])
        hello_str = "{:f},{:f}".format(c.avg_resultx, c.avg_resulty) 
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass



# while True:
#     if ser.readable():
#         res = ser.readline()
#         if len(res.decode().split(','))>=6:
#             c.SumValues(res.decode()[1:len(res)-2])
#             print(c.avg_resultx, c.avg_resulty)