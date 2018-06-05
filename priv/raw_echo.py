import os
import sys
import rospy
from io import BytesIO

def echo_cb(msg):
    buff = BytesIO()
    msg.serialize(buff)
    # erlang expects high-byte then low byte for packet size
    high_byte = chr((len(buff.getvalue()) >> 8) & 0xff)
    low_byte = chr(len(buff.getvalue()) & 0xff)
    os.write(1, high_byte + low_byte + buff.getvalue())

if __name__ == '__main__':
    rospy.init_node('raw_echo', anonymous=True)
    topic = '/eedros/pub/' + sys.argv[1]
    sub = rospy.Subscriber(topic, rospy.msg.AnyMsg, echo_cb)
    rospy.spin()