#!/usr/bin/env python3

import os
import math
import yaml
import rospy
import actionlib

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus


class PoiManager:
    def __init__(self):
        # ------------------------------------------------------------
        # Initialisation du noeud
        # ------------------------------------------------------------
        rospy.init_node("poi_manager_yaml")

        # ------------------------------------------------------------
        # Fichier YAML contenant les POI
        # ------------------------------------------------------------
        self.pois_file = rospy.get_param(
            "~pois_file",
            os.path.expanduser("~/catkin_ws/src/tb3_projet_ma/config/pois_demo.yaml")
        )

        # ------------------------------------------------------------
        # Pose estimée du robot via AMCL
        # Utilisée pour trier les POI avant le départ
        # ------------------------------------------------------------
        self.current_robot_pose = None

        # ------------------------------------------------------------
        # Variables pour les indicateurs
        # ------------------------------------------------------------
        self.mission_started = False
        self.start_time = None
        self.end_time = None

        self.total_path_length = 0.0
        self.last_odom_position = None

        self.reached_pois = 0
        self.total_pois = 0

        # ------------------------------------------------------------
        # Abonnements ROS
        # ------------------------------------------------------------
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

        # ------------------------------------------------------------
        # Client move_base
        # ------------------------------------------------------------
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    def amcl_callback(self, msg):
        """
        Récupère la pose estimée du robot sur la carte.
        """
        self.current_robot_pose = {
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y
        }

    def odom_callback(self, msg):
        """
        Calcule la longueur de trajectoire pendant la mission.

        Principe :
        - on lit la position courante depuis /odom
        - on calcule la distance avec la position précédente
        - on ajoute cette distance au total
        """
        if not self.mission_started:
            return

        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y

        if self.last_odom_position is None:
            self.last_odom_position = (current_x, current_y)
            return

        last_x, last_y = self.last_odom_position
        step_distance = math.hypot(current_x - last_x, current_y - last_y)

        self.total_path_length += step_distance
        self.last_odom_position = (current_x, current_y)

    def load_pois(self, path):
        """
        Charge les POI depuis le fichier YAML.
        """
        with open(path, "r") as f:
            data = yaml.safe_load(f) or {}
        return data.get("pois", [])

    def euclidean_distance(self, x1, y1, x2, y2):
        """
        Distance euclidienne entre deux points.
        """
        return math.hypot(x2 - x1, y2 - y1)

    def sort_pois_nearest_neighbor(self, pois, start_x, start_y):
        """
        Trie les POI par méthode du plus proche voisin.

        Etapes :
        1. partir de la position initiale du robot
        2. choisir le POI le plus proche
        3. se déplacer virtuellement à ce POI
        4. recommencer avec les POI restants
        """
        remaining = pois.copy()
        ordered = []

        current_x = start_x
        current_y = start_y

        while remaining:
            nearest_poi = min(
                remaining,
                key=lambda poi: self.euclidean_distance(
                    current_x, current_y,
                    poi["x"], poi["y"]
                )
            )

            ordered.append(nearest_poi)
            current_x = nearest_poi["x"]
            current_y = nearest_poi["y"]
            remaining.remove(nearest_poi)

        return ordered

    def build_goal(self, poi):
        """
        Construit un objectif move_base à partir d'un POI.
        """
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = poi["x"]
        goal.target_pose.pose.position.y = poi["y"]
        goal.target_pose.pose.position.z = 0.0

        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = poi["z"]
        goal.target_pose.pose.orientation.w = poi["w"]

        return goal

    def wait_for_amcl_pose(self, timeout=10.0):
        """
        Attend qu'une pose AMCL soit disponible.
        """
        start_time = rospy.Time.now().to_sec()
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if self.current_robot_pose is not None:
                return True

            if rospy.Time.now().to_sec() - start_time > timeout:
                return False

            rate.sleep()

    def print_order(self, pois):
        """
        Affiche l'ordre choisi pour la mission.
        """
        rospy.loginfo("Ordre de visite choisi :")
        for i, poi in enumerate(pois, start=1):
            rospy.loginfo(
                f"  {i}. {poi['name']} (x={poi['x']:.2f}, y={poi['y']:.2f})"
            )

    def print_mission_summary(self):
        """
        Affiche le bilan final de la mission dans le terminal.
        """
        if self.start_time is None or self.end_time is None:
            rospy.logwarn("Impossible d'afficher le bilan : temps incomplets.")
            return

        total_time = self.end_time - self.start_time

        rospy.loginfo("====================================")
        rospy.loginfo("===== BILAN DE LA MISSION ==========")
        rospy.loginfo(f"POI atteints : {self.reached_pois} / {self.total_pois}")
        rospy.loginfo(f"Temps de parcours : {total_time:.2f} s")
        rospy.loginfo(f"Longueur de trajectoire : {self.total_path_length:.2f} m")

        if self.reached_pois == self.total_pois:
            rospy.loginfo("Mission terminee avec succes.")
        else:
            rospy.loginfo("Mission interrompue avant la fin.")

        rospy.loginfo("====================================")

    def run(self):
        # ------------------------------------------------------------
        # 1) Chargement des POI
        # ------------------------------------------------------------
        pois = self.load_pois(self.pois_file)
        if not pois:
            rospy.logerr("Aucun POI trouve dans le fichier YAML.")
            return

        self.total_pois = len(pois)
        rospy.loginfo(f"{self.total_pois} POI charges depuis {self.pois_file}")

        # ------------------------------------------------------------
        # 2) Connexion à move_base
        # ------------------------------------------------------------
        rospy.loginfo("Attente du serveur move_base...")
        self.client.wait_for_server()
        rospy.loginfo("Connecte a move_base.")

        # ------------------------------------------------------------
        # 3) Localisation manuelle
        # ------------------------------------------------------------
        input("Fais d'abord 2D Pose Estimate dans RViz, puis appuie sur ENTREE ici pour lancer la mission... ")

        # ------------------------------------------------------------
        # 4) Attente d'une pose AMCL valide
        # ------------------------------------------------------------
        rospy.loginfo("Attente de la pose AMCL du robot...")
        if not self.wait_for_amcl_pose(timeout=10.0):
            rospy.logerr("Impossible de recuperer la pose AMCL du robot.")
            rospy.logerr("Verifie la localisation dans RViz avant de relancer.")
            return

        start_x = self.current_robot_pose["x"]
        start_y = self.current_robot_pose["y"]

        rospy.loginfo(
            f"Pose initiale utilisee pour le tri : x={start_x:.2f}, y={start_y:.2f}"
        )

        # ------------------------------------------------------------
        # 5) Tri initial des POI
        # ------------------------------------------------------------
        ordered_pois = self.sort_pois_nearest_neighbor(pois, start_x, start_y)
        self.print_order(ordered_pois)

        # ------------------------------------------------------------
        # 6) Début de la mission
        # ------------------------------------------------------------
        self.start_time = rospy.Time.now().to_sec()
        self.mission_started = True
        self.last_odom_position = None

        # ------------------------------------------------------------
        # 7) Envoi des objectifs un par un
        # ------------------------------------------------------------
        for poi in ordered_pois:
            rospy.loginfo(f"Envoi du point : {poi['name']}")
            goal = self.build_goal(poi)
            self.client.send_goal(goal)

            finished = self.client.wait_for_result()
            if not finished:
                rospy.logwarn("move_base ne repond plus.")
                self.end_time = rospy.Time.now().to_sec()
                self.mission_started = False
                self.print_mission_summary()
                return

            state = self.client.get_state()
            if state == GoalStatus.SUCCEEDED:
                self.reached_pois += 1
                rospy.loginfo(f"POI atteint : {poi['name']}")
            else:
                rospy.logwarn(f"Echec sur {poi['name']} (etat={state}).")
                self.end_time = rospy.Time.now().to_sec()
                self.mission_started = False
                self.print_mission_summary()
                return

        # ------------------------------------------------------------
        # 8) Fin de la mission
        # ------------------------------------------------------------
        self.end_time = rospy.Time.now().to_sec()
        self.mission_started = False

        rospy.loginfo("Mission terminee : tous les POI ont ete atteints.")
        self.print_mission_summary()


if __name__ == "__main__":
    try:
        manager = PoiManager()
        manager.run()
    except rospy.ROSInterruptException:
        pass