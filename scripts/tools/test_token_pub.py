#!/usr/bin/env python3
"""发布测试 /R1_token_id — Scene 1"""

import rospy
from std_msgs.msg import Int32

if __name__ == "__main__":
    rospy.init_node("test_token_pub")
    pub = rospy.Publisher("/R1_token_id", Int32, queue_size=1, latch=True)

    # Scene 1: 发送 ID=5 作为测试目标
    test_id = 5
    rospy.sleep(1.0)
    rospy.loginfo(f"Publishing /R1_token_id: {test_id}")
    pub.publish(test_id)
    rospy.spin()
