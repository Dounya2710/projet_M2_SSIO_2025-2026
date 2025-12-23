# projet_M2_SSIO_2025-2026
Ce projet vise à concevoir et tester des algorithmes de planification de trajectoire (Dijkstra, A*, Q-learning) dans un environnement simulé sous **Gazebo + ROS 2**, avec un **TurtleBot4** (robot mobile) et, à terme, un **drone Crazyflie**.

La planification repose sur une **représentation matricielle de l’environnement**
extraite du monde Gazebo.

--

## Accès aux fichiers

Google Drive: https://drive.google.com/drive/folders/1a9tsptmZjvx-htNuJZf33DHj_fXNanC6?usp=sharing

--

## Pré-requis

- Ubuntu 22.04
- ROS 2 **Humble**
- Gazebo
- TurtleBot4 packages installés
- Python ≥ 3.10
- Colcon

--

## 1️) Initialisation de l’environnement ROS 2

Attention, à faire avant chaque utilisation de ROS 2 (nouveau terminal) :

```bash
source /opt/ros/humble/setup.bash
```

## 2) Vérification des topics ROS

Avant toute intégration, vérifier que les topics nécessaires à la navigation sont disponibles :

```bash
ros2 topic list
```

Il faut notamment s’assurer de la présence de :

- `/cmd_vel` (commande de vitesse)
- `/odom` (odométrie)
- `/tf` et `/tf_static`

## 3) Lancement du robot TurtleBot4

### 3.1 Apparition du robot dans Gazebo

```bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_spawn.launch.py turtlebot4_model:=standard
````

### 3.2 Lancement de Gazebo avec le monde personnalisé

Le monde utilisé correspond au fichier env.sdf présent dans le dépôt GitHub.

```bash
ros2 launch turtlebot4_ignition_bringup ignition.launch.py nav2:=True world:=/home/USER/chemin/vers/le/projet/env/env.sdf
```
Adapter le chemin vers env.sdf selon votre arborescence locale.

## 4) Création et compilation du package de planification

### 4.1 Création du package Python planner
```bash
ros2 pkg create planner --build-type ament_python --dependencies rclpy nav_msgs geometry_msgs
````

### 4.2 Compilation du package

Depuis la racine du workspace ROS 2 :

```bash
colcon build --packages-select planner
```

### 4.3 Sourcing du workspace

Obligatoire après chaque compilation :

```bash
source install/setup.bash
```

## 5) Exécution du nœud de planification

Le nœud planner_node.py :

- charge la matrice de l’environnement,
- calcule un chemin avec Dijkstra, A* ou Q-learning,
- publie le résultat sous forme de nav_msgs/Path.

Lancer le planner :

```bash
ros2 run planner planner_node
````

Ou en précisant l’algorithme :

```bash
ros2 run planner planner_node --ros-args -p algo:=dijkstra
ros2 run planner planner_node --ros-args -p algo:=astar
ros2 run planner planner_node --ros-args -p algo:=qlearning
````

## 6) Visualisation du chemin dans RViz

Le chemin calculé est publié sur le topic :

```bash
/planned_path
```

Pour le visualiser :

```bash
rviz2
```

Dans RViz :

- Fixed Frame : `odom` (ou `map` selon la configuration TF)

- Ajouter → Path

- Topic : `/planned_path`

Le chemin est affiché par-dessus la simulation Gazebo.