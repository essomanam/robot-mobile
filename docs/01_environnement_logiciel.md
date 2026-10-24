# 01 — Environnement logiciel

> **Résumé en 3 lignes**
> 1. Le développement se fait sous **Ubuntu 26.04 + ROS 2 Lyrical + Gazebo Jetty** ; ce document fixe les versions exactes pour que les résultats soient reproductibles.
> 2. Les paquets nécessaires aux étapes suivantes (RTAB-Map, Nav2, slam_toolbox, ArUco…) existent pour Lyrical mais ne sont **pas encore installés** : ils le seront à l'étape qui en a besoin.
> 3. Point de vigilance : ne jamais mélanger deux versions de ROS dans un même terminal, et vérifier tôt la version de ROS du robot réel.

Versions relevées le **2026-10-24**. À mettre à jour à chaque installation (commande en §8).

---

## 1. Pourquoi documenter l'environnement ?

Un résultat scientifique doit pouvoir être reproduit. En robotique logicielle, le même code peut
se comporter différemment selon la version de ROS, du simulateur ou des bibliothèques
(changement de type de message, de paramètre par défaut, de comportement physique…). Noter
les versions exactes permet d'expliquer un écart et de reconstruire l'environnement plus tard.

## 2. Machine de développement

| Élément | Valeur |
|---|---|
| Système | Ubuntu 26.04.1 LTS (*Resolute Raccoon*), noyau Linux 7.0.0-38 |
| Processeur | 12 cœurs logiques |
| Carte graphique | Hybride : Intel Iris Xe (intégrée) + NVIDIA T550 Laptop GPU (pilote 580.178.04) |

## 3. Pile logicielle

| Composant | Version | Rôle |
|---|---|---|
| ROS 2 | **Lyrical Luth** (`ros-lyrical-desktop` 0.13.0) | Intergiciel robotique : communication entre programmes, outils |
| Middleware DDS | `rmw_fastrtps_cpp` (Fast DDS, par défaut) | Couche réseau qui transporte les messages ROS 2 |
| Gazebo | **Jetty** (gz-sim 10.5.0, via `gz_sim_vendor` 0.4.6) | Simulateur physique et capteurs |
| ros_gz | 3.0.10 | Pont entre Gazebo et ROS 2 |
| turtlebot3_gazebo / turtlebot3_simulations | 2.3.7 | Modèles et mondes TurtleBot3 pour Gazebo |
| cv_bridge | 4.1.0 | Conversion images ROS ↔ OpenCV |
| RViz 2 | 15.2.6 | Visualisation 3D des données ROS |
| rclpy | 10.0.11 | Bibliothèque ROS 2 pour Python |
| Python | 3.14.4 | Langage des nœuds du projet |
| OpenCV | 4.10.0 | Traitement d'image |
| NumPy | 2.3.5 | Calcul numérique |
| colcon | colcon-common-extensions 0.3.0 | Outil de compilation des paquets ROS 2 |

## 4. Paquets ROS : installés et à installer

| Paquet apt | État | Utilisé à l'étape |
|---|---|---|
| `ros-lyrical-desktop` | ✅ installé | toutes |
| `ros-lyrical-ros-gz` | ✅ installé | E1 → E4 |
| `ros-lyrical-turtlebot3-simulations` (`turtlebot3_gazebo`, `turtlebot3_fake_node`) | ✅ installé | E1 → E4 |
| `ros-lyrical-depthimage-to-laserscan` | ✅ installé | E4 |
| `ros-lyrical-teleop-twist-keyboard` | ✅ installé | E1 (pilotage manuel) |
| `ros-lyrical-rqt-image-view` | à installer | E1, E3 (visualiser les images) |
| `ros-lyrical-rtabmap-ros` | à installer | E2 (V-SLAM) |
| `ros-lyrical-slam-toolbox` | à installer | E2 (référence LiDAR pour comparaison) |
| `ros-lyrical-navigation2`, `ros-lyrical-nav2-bringup` | à installer | E2 → E4 (navigation) |
| `ros-lyrical-image-proc`, `ros-lyrical-depth-image-proc` | à installer | E1 → E4 (traitement images / profondeur) |
| `ros-lyrical-aruco-opencv`, `ros-lyrical-vision-msgs` | à installer | E3 (marqueurs, messages de détection) |
| `evo` (outil Python, via `pip`/`pipx`) | à installer | E2 (calcul des erreurs de trajectoire) |

Notes :

- Le paquet `turtlebot3_gazebo` de Lyrical commande le robot sur `/cmd_vel` avec le type
  **`geometry_msgs/msg/TwistStamped`** (vitesse + horodatage), et non plus `Twist`.
- **ORB-SLAM3** n'existe pas en paquet ROS 2 : il faudrait le compiler depuis les sources.
  C'est l'une des raisons du choix de RTAB-Map (voir [D-005](decisions.md#d-005--rtab-map-comme-outil-de-v-slam-principal)).

## 5. Configuration du terminal

Chaque nouveau terminal doit charger ROS 2, puis le workspace du projet une fois compilé :

```bash
source /opt/ros/lyrical/setup.bash
source ~/Etudes/A2026/AS52/robot-mobile/install/setup.bash   # après colcon build
export TURTLEBOT3_MODEL=waffle          # modèle utilisé par les launch de turtlebot3_gazebo
export ROS_DOMAIN_ID=<numéro>           # à fixer au labo pour ne pas recevoir les robots des autres
```

Ces lignes peuvent être ajoutées à `~/.bashrc`. `ROS_DOMAIN_ID` isole les groupes sur un même
réseau : deux machines ne se « voient » que si elles ont le même numéro.

`rosdep` (installation automatique des dépendances déclarées dans les `package.xml`) est
présent mais **pas encore initialisé** sur la machine. À faire une seule fois :

```bash
sudo rosdep init
rosdep update
```

## 6. Points de vigilance

### 6.1 Deux versions de ROS sur la machine

Le dossier `/opt/ros/jazzy` existe aussi (paquets ROS 2 Jazzy, prévus pour Ubuntu 24.04).
La ligne `source /opt/ros/jazzy/setup.bash` est commentée dans `~/.bashrc` et doit le rester :
**ne jamais charger Jazzy et Lyrical dans le même terminal**, les bibliothèques se mélangeraient.

### 6.2 Version de ROS du robot réel

Les TurtleBot3 sont en général livrés avec ROS 2 Humble ou Jazzy. Deux versions différentes de
ROS 2 ne sont pas garanties compatibles sur le réseau (définitions de messages et métadonnées
DDS qui évoluent). À vérifier dès que possible (question Q2 de
l'[analyse du sujet](00_analyse_du_sujet.md#5-questions-ouvertes-pour-le-superviseur)).
Solution de repli : exécuter la même version que le robot dans un conteneur Docker.

### 6.3 Carte graphique hybride

Par défaut, Gazebo s'affiche sur la carte Intel. Si la simulation est lente (le LiDAR simulé
`gpu_lidar` et la caméra utilisent le rendu 3D), on peut forcer la carte NVIDIA :

```bash
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia ros2 launch ...
```

*À vérifier à l'étape E1* (gain réel à mesurer avec le facteur temps réel affiché par Gazebo).

## 7. Commandes de vérification

```bash
printenv ROS_DISTRO                       # doit afficher : lyrical
ros2 doctor --report | grep -A2 MIDDLEWARE
gz sim --versions                         # doit afficher : 10.x
ros2 pkg list | grep turtlebot3
```

## 8. Mettre à jour ce document

Après toute installation, relever les nouvelles versions :

```bash
dpkg-query -W -f='${Package} ${Version}\n' 'ros-lyrical-*' | grep -E 'desktop|ros-gz|turtlebot3|rtabmap|nav2-bringup|slam-toolbox|aruco'
```

et noter l'installation dans le [journal](journal/).
