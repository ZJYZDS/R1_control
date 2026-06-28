#!/usr/bin/env python3
"""简易测试发布: 按 A 发送 [id1, id2] 到 /R1_grap_pos."""

import rospy
from std_msgs.msg import Int32MultiArray
import sys
import termios
import tty


def get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    rospy.init_node("test_grap_pub")
    pub = rospy.Publisher("/R1_grap_pos", Int32MultiArray, queue_size=5)
    id1, id2 = 1, 3

    if id1 == id2 :
        rospy.logerr_once("idx1 can't is equal to idx2")
    elif id1 not in [3,4,5,6,8,9,11,12,14] or id2 not in [3,4,5,6,8,9,11,12,14]:
        rospy.logerr_once("error id")
    else:
        rospy.logdebug_once("ok id")

    while not rospy.is_shutdown():
        
        msg = Int32MultiArray(data=[id1, id2])
        pub.publish(msg)
        rospy.loginfo(f"Published: [id1={id1}, id2={id2}]")


if __name__ == "__main__":
    main()
