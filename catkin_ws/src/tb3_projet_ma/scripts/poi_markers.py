#!/usr/bin/env python3
import os
import yaml
import rospy

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


class PoiMarkersTool:
    def __init__(self):
        self.pois_file = rospy.get_param(
            "~pois_file",
            os.path.expanduser("~/catkin_ws/src/tb3_projet_ma/config/pois_demo.yaml")
        )
        self.action = rospy.get_param("~action", "show")
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.cross_size = float(rospy.get_param("~cross_size", 0.25))
        self.line_width = float(rospy.get_param("~line_width", 0.04))
        self.text_height = float(rospy.get_param("~text_height", 0.18))

        self.pub = rospy.Publisher("/poi_markers", MarkerArray, queue_size=1, latch=True)

        rospy.sleep(1.0)

        if self.action == "hide":
            self.publish_delete_all()
            rospy.loginfo("Tous les POI ont ete caches.")
        else:
            self.publish_markers()
            rospy.loginfo("POI affiches sur /poi_markers")

    def load_pois(self):
        with open(self.pois_file, "r") as f:
            data = yaml.safe_load(f) or {}
        return data.get("pois", [])

    def make_cross_marker(self, marker_id, x, y):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "poi_crosses"
        marker.id = marker_id
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD

        marker.pose.orientation.w = 1.0
        marker.scale.x = self.line_width

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        half = self.cross_size / 2.0

        p1 = Point()
        p1.x = x - half
        p1.y = y - half
        p1.z = 0.05

        p2 = Point()
        p2.x = x + half
        p2.y = y + half
        p2.z = 0.05

        p3 = Point()
        p3.x = x - half
        p3.y = y + half
        p3.z = 0.05

        p4 = Point()
        p4.x = x + half
        p4.y = y - half
        p4.z = 0.05

        marker.points.append(p1)
        marker.points.append(p2)
        marker.points.append(p3)
        marker.points.append(p4)

        return marker

    def make_text_marker(self, marker_id, x, y, name):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "poi_labels"
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.25
        marker.pose.orientation.w = 1.0

        marker.scale.z = self.text_height

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.text = name
        return marker

    def publish_markers(self):
        pois = self.load_pois()
        markers = MarkerArray()

        if not pois:
            rospy.logwarn("Aucun POI trouve dans le YAML.")
            self.pub.publish(markers)
            return

        idx = 0
        for poi in pois:
            name = poi.get("name", f"poi_{idx+1}")
            x = float(poi["x"])
            y = float(poi["y"])

            cross = self.make_cross_marker(idx, x, y)
            markers.markers.append(cross)
            idx += 1

            text = self.make_text_marker(idx, x, y, name)
            markers.markers.append(text)
            idx += 1

        self.pub.publish(markers)

    def publish_delete_all(self):
        markers = MarkerArray()
        marker = Marker()
        marker.action = Marker.DELETEALL
        markers.markers.append(marker)
        self.pub.publish(markers)


if __name__ == "__main__":
    rospy.init_node("poi_markers")
    try:
        PoiMarkersTool()
    except rospy.ROSInterruptException:
        pass
