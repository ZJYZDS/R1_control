"""Re-export from r1_graph.receiver — launch-file compatibility wrapper."""
from r1_graph.receiver import MapSectionReceiver  # noqa: F401


if __name__ == "__main__":
    try:
        import rospy
        MapSectionReceiver()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
