import os
import sys
import rospy
import roslib.message
import std_msgs
from io import BytesIO

if __name__ == '__main__':
    rospy.init_node('eedros_sub', anonymous=True)
    topic = '/eedros/sub/' + sys.argv[1]
    msg_class = roslib.message.get_message_class(sys.argv[2])
    pub = rospy.Publisher(topic, msg_class, queue_size=10)
    while True:
        size_str = bytearray()
        while len(size_str) != 2:
            size_str += bytearray(os.read(0, 2))

        size = (size_str[0] << 8) | size_str[1]
        
        message = bytes()
        while len(message) < size:
            message += os.read(0, size-len(message))

        msg = msg_class()
        msg.deserialize(message)
        pub.publish(msg)
        
