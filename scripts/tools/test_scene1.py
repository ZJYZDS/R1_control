#!/usr/bin/env python3
"""Scene 1 集成测试: 发布模拟 odom + /R1_token_id [id1, id2], 观察规划结果"""

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Point, Quaternion

# ── 步骤 1: 发布里程计(持续 + latch) ─────────────────────────────

rospy.init_node("test_scene1")
odom_pub = rospy.Publisher("/aft_mapped_to_init", Odometry, queue_size=10, latch=True)

msg = Odometry()
msg.header.frame_id = "camera_init"
msg.pose.pose.position = Point(0.0, 0.0, 0.0)
msg.pose.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

rospy.loginfo("Publishing odometry at (0,0) for 5s...")
rate = rospy.Rate(10)
for i in range(50):
    msg.header.stamp = rospy.Time.now()
    odom_pub.publish(msg)
    rate.sleep()

# ── 步骤 2: 等 mapper 收到 odom ─────────────────────────────────

rospy.sleep(1.0)

# ── 步骤 3: 发布 token pairs ────────────────────────────────────

token_pub = rospy.Publisher("/R1_token_id", Int32MultiArray, queue_size=1, latch=True)
rospy.sleep(0.5)

test_pairs = [
    [5, 12],
    [3, 14],
    [6, 11],
]
for pair in test_pairs:
    rospy.loginfo(f"Publishing /R1_token_id: {pair}")
    token_pub.publish(Int32MultiArray(data=pair))
    rospy.sleep(2.0)

rospy.loginfo("Scene 1 test done")
