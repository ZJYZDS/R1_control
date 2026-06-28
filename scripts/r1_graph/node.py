"""GraphMapperNode — ROS node subscribing to /R1_token_id for graph planning."""

import numpy as np

import rospy
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32MultiArray

from .config import Config
from .mapper import GraphMapper, supplement_ids


class GraphMapperNode:
    def __init__(self):
        rospy.init_node("r1_graph_mapper")
        self.mapper = GraphMapper()
        self.current_pose = None

        rospy.Subscriber("/aft_mapped_to_init", Odometry, self.cb_odom)
        rospy.Subscriber("/R1_token_id", Int32MultiArray, self.cb_token_id)

        self.pub_path = rospy.Publisher("/global_path", Path, queue_size=1, latch=True)
        self.pub_wp   = rospy.Publisher("/r1_waypoints_viz", Path, queue_size=1, latch=True)

        rospy.loginfo("GraphMapperNode ready, listening to /R1_token_id")

    def cb_odom(self, msg):
        pos = msg.pose.pose.position
        self.current_pose = np.array([pos.x, pos.y])

    def cb_token_id(self, msg):
        ids = list(msg.data)
        if len(ids) != 2:
            rospy.logwarn(f"/R1_token_id expects 2 ints, got {len(ids)}: {ids}")
            return
        rospy.loginfo(f"Received /R1_token_id: {ids}")

        if self.current_pose is None:
            rospy.logerr("No odometry yet, cannot plan")
            return

        required = [i for i in ids if i in self.mapper.id_map]
        id_num = len(required)

        id1, id2 = supplement_ids(self.mapper, required)
        if id1 is None:
            rospy.logerr(f"ID supplement failed, required={required}")
            return

        if id_num < 2:
            rospy.loginfo(f"Supplemented: required={required} → [{id1}, {id2}]")

        c1 = self.mapper.id_to_coord(id1)
        c2 = self.mapper.id_to_coord(id2)
        if c1 is None or c2 is None:
            return

        nodes, adj = self.mapper.build_graph([c1, c2])

        start_idx = Config.K_INDEX[Config.START_K] + Config.K_GRAPH_OFFSET
        end_idx   = Config.K_INDEX[Config.END_K]   + Config.K_GRAPH_OFFSET
        rospy.loginfo(f"Scenario: {Config.SCENARIO}, Start: {Config.START_K}, End: {Config.END_K}")

        route = self.mapper.find_path(nodes, adj, start_idx, end_idx)
        if not route:
            return

        cost = sum(np.linalg.norm(route[i][1][:2] - route[i + 1][1][:2])
                   for i in range(len(route) - 1))

        print("\n" + "=" * 50)
        print(f"  Targets: [{id1}, {id2}]  @  {c1[:2]}, {c2[:2]}")
        print(f"  Path cost: {cost:.2f} m")
        print(f"  Route: {' → '.join(n for n, _ in route)}")
        for name, coord in route:
            print(f"    {name:6s}  ({coord[0]:6.2f}, {coord[1]:6.2f})")
        print("=" * 50 + "\n")

        self._publish_paths(route, nodes)

    def _publish_paths(self, path, nodes):
        self._pub(self.pub_path, [(c[0], c[1]) for _, c in path])
        self._pub(self.pub_wp, [(c[0], c[1]) for _, c in nodes])

    def _pub(self, pub, pts):
        if not pts:
            return
        msg = Path()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "camera_init"
        for x, y in pts:
            p = PoseStamped()
            p.header = msg.header
            p.pose.position.x = x
            p.pose.position.y = y
            p.pose.position.z = 0.0
            p.pose.orientation.w = 1.0
            msg.poses.append(p)
        pub.publish(msg)


if __name__ == "__main__":
    try:
        GraphMapperNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
