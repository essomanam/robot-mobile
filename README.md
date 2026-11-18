# robot-mobile — Perception visuelle, V-SLAM et navigation autonome sur TurtleBot3

Projet du module **AS52 – Robotique mobile**, Master 2 RSA (Robotique et Systèmes Autonomes),
UTBM, année 2026-2027.

L'objectif est de rendre un robot **TurtleBot3** autonome en s'appuyant principalement sur la
**perception visuelle** (caméra RGB-D), complétée par le LiDAR : construire une carte et s'y
localiser (V-SLAM), se diriger vers des cibles repérées à la caméra, éviter les obstacles imprévus
grâce à la profondeur. Le travail est développé en simulation (Gazebo) puis validé sur le robot réel.

## Avancement

| Étape | Contenu | Statut |
|---|---|---|
| E0 | Fondations : dépôt, documentation, analyse, feuille de route | 🟢 terminée |
| E1 | Simulation Gazebo et acquisition des données capteurs | 🟢 terminée |
| E2 | Cartographie et localisation visuelle (V-SLAM, RTAB-Map) | 🟢 terminée |
| E3 | Navigation guidée par la vision | 🟢 terminée |
| E4 | Perception de la profondeur et évitement d'obstacles | 🟢 terminée |
| E5 | Validation sur le robot réel | ⚪ à faire |
| E6 | Rapport technique et démonstration | ⚪ à faire |

## Structure du dépôt

Ce dépôt regroupe plusieurs paquets ROS 2. Comme tout dépôt ROS, il se clone dans le dossier
`src/` d'un workspace colcon ; la compilation produit `build/`, `install/` et `log/` à la racine
du workspace, en dehors du dépôt :

```
as52_ws/                          workspace colcon (non versionné)
├── src/
│   └── robot-mobile/             ce dépôt
├── build/                        fichiers intermédiaires de compilation
├── install/                      paquets compilés, chargés par `source install/setup.bash`
└── log/                          journaux de colcon
```

Contenu du dépôt :

```
robot-mobile/
├── README.md                     ce fichier : présentation et manuel d'exécution
├── LICENSE.md                    licence Apache-2.0
├── robot_mobile_description/     le robot : TurtleBot3 Waffle + caméra RGB-D
│   ├── urdf/                     description pour ROS (TF, RViz)
│   └── models/robot_mobile/      modèle pour Gazebo (physique, capteurs)
├── robot_mobile_bringup/         lancement et configuration
│   ├── launch/                   simulation, jeu de données, SLAM, approche de la cible, navigation, essais
│   ├── config/                   pont Gazebo ↔ ROS 2, trajectoires, slam/, perception/, control/, navigation/
│   ├── worlds/, models/          salle as52_room (et variante avec obstacles), cible rouge
│   ├── rviz/                     configuration d'affichage
│   └── scripts/                  textures, séries d'essais mesurés (SLAM, cible, navigation)
├── robot_mobile_perception/      détection de la cible (couleur + profondeur) et tests
├── robot_mobile_control/         comportement d'approche (machine à états) et tests
└── robot_mobile_evaluation/      outils d'expérience (Python) et tests
    ├── robot_mobile_evaluation/  contrôle des capteurs, trajectoires, métriques, suivi des essais
    └── test/                     tests des métriques (ATE, RPE, marges) et du style
```

## Environnement requis

| Composant | Version |
|---|---|
| Ubuntu | 26.04 LTS |
| ROS 2 | Lyrical Luth |
| Gazebo | Jetty (gz-sim 10) |
| Python | 3.14 |

## Installation et compilation

Le workspace peut être placé n'importe où ; les commandes ci-dessous utilisent `~/as52_ws`.

```bash
# 1. Charger ROS 2
source /opt/ros/lyrical/setup.bash

# 2. Créer le workspace et cloner le dépôt dans src/
mkdir -p ~/as52_ws/src
git clone https://github.com/essomanam/robot-mobile.git ~/as52_ws/src/robot-mobile

# 3. Installer les dépendances déclarées par les paquets
#    (première fois seulement : sudo rosdep init && rosdep update)
cd ~/as52_ws
rosdep install --from-paths src --ignore-src -y

# 4. Compiler depuis la racine du workspace (jamais depuis src/), puis charger le résultat
colcon build --symlink-install
source install/setup.bash
```

Vérification : `ros2 pkg prefix robot_mobile_bringup` doit afficher
`~/as52_ws/install/robot_mobile_bringup`.

Recompiler (`colcon build --symlink-install`, depuis `~/as52_ws`) après l'ajout d'un fichier ou
la modification d'un `package.xml`, `setup.py` ou `CMakeLists.txt`. Grâce à `--symlink-install`,
la modification d'un fichier existant (Python, YAML, launch) est prise en compte sans recompiler.

## Utilisation

Dans chaque terminal : `source /opt/ros/lyrical/setup.bash && source ~/as52_ws/install/setup.bash`.

Les commandes `ros2` fonctionnent ensuite depuis n'importe quel dossier. Les chemins de scripts
(`robot_mobile_bringup/scripts/...`) sont relatifs à la racine du dépôt (`~/as52_ws/src/robot-mobile`).

### Simulation (étape E1)

```bash
ros2 launch robot_mobile_bringup simulation.launch.py
```

Ouvre Gazebo (salle `as52_room`, robot au point (-2,5 ; -1,5)) et RViz (robot, scan LiDAR,
nuage de profondeur, images couleur et profondeur). Sans fenêtre : ajouter `gui:=false rviz:=false`.

Piloter le robot au clavier (autre terminal) :

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
    -p stamped:=true -p frame_id:=base_footprint -p speed:=0.2 -p turn:=0.8
```

Les touches (`i` avancer, `,` reculer, `j` / `l` tourner, `k` arrêter) doivent être tapées
**dans ce terminal** : il doit être la fenêtre active, pas Gazebo ni RViz. Les vitesses
0,2 m/s et 0,8 rad/s restent dans les limites d'un Waffle réel (0,26 m/s, 1,82 rad/s).

### Contrôle des capteurs (E1)

Simulation lancée, robot immobile à sa position de départ :

```bash
ros2 run robot_mobile_evaluation sensor_check --ros-args -p use_sim_time:=true
```

Compare les distances mesurées par le LiDAR et la caméra de profondeur aux distances calculées
à partir de la géométrie de la salle. Résultat attendu : biais de l'ordre du millimètre.

### Jeu de données de référence (E1)

```bash
ros2 launch robot_mobile_bringup reference_dataset.launch.py
ros2 run robot_mobile_evaluation analyze_dataset ~/as52_data/rosbags/<enregistrement> ~/as52_data/figures
```

La première commande lance la simulation sans fenêtre, fait faire au robot un tour de salle
scripté et l'enregistre (rosbag MCAP dans `~/as52_data/rosbags/`) ; elle s'arrête seule.
La seconde calcule les fréquences des capteurs et compare l'odométrie à la vérité terrain.

### Cartographie et localisation (SLAM, étape E2)

Pendant la simulation (autre terminal), construire une carte avec RTAB-Map :

```bash
ros2 launch robot_mobile_bringup slam.launch.py                       # RTAB-Map RGB-D + LiDAR + roues
ros2 launch robot_mobile_bringup slam.launch.py method:=slam_toolbox  # autre méthode
```

Méthodes : `rtabmap_rgbd_scan` (caméra RGB-D + LiDAR + roues, **par défaut**, la plus précise),
`rtabmap_rgbd` (caméra + roues), `rtabmap_rgbd_vo` (caméra seule), `slam_toolbox` (LiDAR + roues). La carte
RTAB-Map est enregistrée dans `~/.ros/rtabmap.db` (`database_path:=` pour la changer).

Comparer les méthodes sur le jeu de données de référence (≈ 15 min, sans simulation) :

```bash
robot_mobile_bringup/scripts/run_slam_comparison.sh ~/as52_data/slam_runs/<essai>
```

Chaque méthode rejoue le même rosbag ; le script produit un tableau d'erreurs (ATE, RPE) et des
figures (trajectoires, erreurs, cartes) dans `~/as52_data/slam_runs/<essai>/`.

Se localiser dans une carte enregistrée, sur un autre parcours (sens horaire, départ au coin
nord-est) :

```bash
# 1. enregistrer le second parcours (≈ 3 min, sans fenêtre)
ros2 launch robot_mobile_bringup reference_dataset.launch.py \
    trajectory:=$(ros2 pkg prefix robot_mobile_bringup)/share/robot_mobile_bringup/config/localization_trajectory.yaml \
    x:=2.8 y:=1.8 yaw:=-1.5708 output:=~/as52_data/rosbags/E2_localisation

# 2. localisation dans la carte d'un essai de cartographie, puis évaluation
ROS_DOMAIN_ID=42 ros2 launch robot_mobile_bringup slam_evaluation.launch.py localization:=true \
    map_database:=<essai de cartographie>/rtabmap.db bag:=~/as52_data/rosbags/E2_localisation \
    output:=~/as52_data/slam_runs/localisation/rtabmap_rgbd_scan_1
ros2 run robot_mobile_evaluation evaluate_localization ~/as52_data/slam_runs/localisation \
    <essai de cartographie>/ground_truth.tum ~/as52_data/slam_runs/localisation/figures
```

Si une simulation est déjà ouverte, isoler l'enregistrement : `ROS_DOMAIN_ID=43 GZ_PARTITION=eval ros2 launch ...`.

### Navigation guidée par la vision (étape E3)

Simulation lancée, le robot cherche la cible rouge, s'aligne, s'en approche et s'arrête à 0,6 m :

```bash
ros2 launch robot_mobile_bringup target_approach.launch.py
ros2 run rqt_image_view rqt_image_view /target/debug_image     # ce que voit le détecteur
robot_mobile_bringup/scripts/run_target_trials.sh ~/as52_data/target_trials/E3   # 6 scénarios mesurés
robot_mobile_bringup/scripts/run_perception_check.sh ~/as52_data/perception/E3   # détection seule
```

### Navigation avec évitement d'obstacles (étape E4)

Monde avec des obstacles absents de la carte, localisation RTAB-Map (carte faite dans les deux
sens), Nav2 avec le LiDAR et la caméra de profondeur :

```bash
ros2 launch robot_mobile_bringup simulation.launch.py \
    world:=$(ros2 pkg prefix robot_mobile_bringup)/share/robot_mobile_bringup/worlds/as52_room_obstacles.sdf
ros2 launch robot_mobile_bringup navigation.launch.py map_database:=<carte>.db rviz:=true
# puis « 2D Goal Pose » dans RViz ; use_depth:=false pour le LiDAR seul
robot_mobile_bringup/scripts/run_navigation_trials.sh ~/as52_data/nav_trials/E4 <carte>.db   # essais mesurés
robot_mobile_bringup/scripts/run_costmap_snapshots.sh ~/as52_data/nav_trials/E4_costmap <carte>.db
```

Construire la carte (deux sens, ≈ 4 min, sans fenêtre) :
`ros2 launch robot_mobile_bringup mapping_session.launch.py output:=~/as52_data/maps/carte`.

### Exploration autonome d'un lieu inconnu (en cours, D-018)

Le robot démarre sans carte, cartographie en naviguant (RTAB-Map + Nav2) et va de frontière en
frontière (limites entre le connu et l'inconnu) jusqu'à ce qu'il n'en reste plus :

```bash
ros2 launch robot_mobile_bringup exploration.launch.py world:=arena    # arène TurtleBot3
ros2 launch robot_mobile_bringup exploration.launch.py world:=house    # maison, plusieurs pièces
ros2 launch robot_mobile_bringup exploration.launch.py world:=as52_room
```

La carte est écrite dans `~/as52_data/exploration/<monde>/rtabmap.db` (écrasée à chaque lancement).
RViz affiche les frontières (orange) et le but en cours (vert). La détection des frontières
(`robot_mobile_control/frontiers.py`) est à écrire : `python3 -m pytest robot_mobile_control/test/test_frontiers.py`.

## Auteur

essomanam — Master 2 RSA, UTBM. Encadrant : Mohamed KAS.

## Licence

Apache-2.0 — voir [LICENSE.md](LICENSE.md).
