#!/usr/bin/env python3
import os
import yaml
import rospy
from geometry_msgs.msg import PoseStamped

class PoiRecorder:
    def __init__(self):
        self.output_file = rospy.get_param(
            "~output_file",
            os.path.expanduser("~/catkin_ws/src/tb3_projet_ma/config/pois_demo.yaml")
        )
        self.prefix = rospy.get_param("~prefix", "poi")
        self.pois = []

        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_callback)
        rospy.on_shutdown(self.save_file)

        rospy.loginfo("Recorder pret.")
        rospy.loginfo("Clique sur 2D Nav Goal dans RViz pour enregistrer les POI.")
        rospy.loginfo("Ctrl+C pour sauvegarder et quitter.")

    def goal_callback(self, msg):
        idx = len(self.pois) + 1
        poi = {
            "name": f"{self.prefix}_{idx}",
            "x": float(msg.pose.position.x),
            "y": float(msg.pose.position.y),
            "z": float(msg.pose.orientation.z),
            "w": float(msg.pose.orientation.w),
        }
        self.pois.append(poi)
        rospy.loginfo(f"POI enregistre : {poi}")

    def save_file(self):
        os.makedirs(os.path.dirname(self.output_file), exist_ok=True)
        data = {"pois": self.pois}
        with open(self.output_file, "w") as f:
            yaml.safe_dump(data, f, sort_keys=False)
        rospy.loginfo(f"POI sauvegardes dans : {self.output_file}")

if __name__ == "__main__":
    rospy.init_node("record_pois")
    PoiRecorder()
    rospy.spin()
