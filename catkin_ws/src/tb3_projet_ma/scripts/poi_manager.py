#!/usr/bin/env python3

# ==========================================================
# POI Manager très simple pour TurtleBot3
# Version volontairement minimale :
# - 2 POI codés directement dans le script
# - envoi séquentiel à move_base
# - arrêt si un point échoue
# ==========================================================

import rospy
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus


# ----------------------------------------------------------
# Liste des POI
# Chaque POI contient :
# - un nom
# - une position (x, y)
# - une orientation finale (z, w) du quaternion
# Ces valeurs viennent de /move_base_simple/goal
# ----------------------------------------------------------
POIS = [
    {
        "name": "poi_1",
        "x": -1.7186739444732666,
        "y": -1.1476316452026367,
        "z": 0.7434909285689083,
        "w": 0.6687460199027299
    },
    {
        "name": "poi_2",
        "x": -2.1488308906555176,
        "y": -0.26023969054222107,
        "z": 0.5769111113824241,
        "w": 0.8168069353057038
    }
]


def creer_goal(poi):
    """
    Crée un objectif MoveBaseGoal à partir d'un dictionnaire POI.
    """
    goal = MoveBaseGoal()

    # On travaille dans le repère de la carte
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    # Position du point à atteindre
    goal.target_pose.pose.position.x = poi["x"]
    goal.target_pose.pose.position.y = poi["y"]
    goal.target_pose.pose.position.z = 0.0

    # Orientation finale souhaitée
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = poi["z"]
    goal.target_pose.pose.orientation.w = poi["w"]

    return goal


def main():
    rospy.init_node("poi_manager")

    rospy.loginfo("Demarrage du POI Manager...")

    # Client action vers move_base
    # C'est move_base qui calcule le chemin et déplace réellement le robot
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    rospy.loginfo("Attente du serveur move_base...")
    client.wait_for_server()
    rospy.loginfo("Connexion a move_base reussie.")

    # On envoie les POI un par un
    for poi in POIS:
        rospy.loginfo("Envoi du point : %s", poi["name"])

        goal = creer_goal(poi)
        client.send_goal(goal)

        # On attend la fin de l'objectif courant
        finished = client.wait_for_result()

        if not finished:
            rospy.logwarn("Le serveur move_base ne repond plus.")
            return

        state = client.get_state()

        # GoalStatus.SUCCEEDED = objectif atteint
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("POI atteint : %s", poi["name"])
        else:
            rospy.logwarn("Echec sur %s, code retour = %d", poi["name"], state)
            rospy.logwarn("Arret de la mission.")
            return

    rospy.loginfo("Mission terminee : tous les POI ont ete atteints.")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass