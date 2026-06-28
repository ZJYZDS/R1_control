"""Re-export from r1_graph — launch-file compatibility wrapper.

r1_graph.mapper:  GraphMapper, supplement_ids
r1_graph.node:    GraphMapperNode (ROS)
"""

from r1_graph.mapper import GraphMapper, supplement_ids  # noqa: F401
from r1_graph.node import GraphMapperNode  # noqa: F401


if __name__ == "__main__":
    try:
        import rospy
        GraphMapperNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
