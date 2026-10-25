# robot-mobile — Perception visuelle, V-SLAM et navigation autonome sur TurtleBot3

Projet du module **AS52 – Robotique mobile**, Master 2 RSA (Robotique et Systèmes Autonomes),
UTBM, année 2026-2027.

L'objectif est de rendre un robot **TurtleBot3** autonome en s'appuyant principalement sur la
**perception visuelle** (caméra RGB-D), complétée par le LiDAR : construire une carte et s'y
localiser (V-SLAM), se diriger vers des cibles repérées à la caméra, éviter les obstacles imprévus
grâce à la profondeur. Le travail est développé en simulation (Gazebo) puis validé sur le robot réel.

> 📚 **Documentation complète : [`docs/`](docs/README.md)** — analyse du sujet, méthode, choix
> justifiés, résultats, étape par étape.

## Avancement

| Étape | Contenu | Statut |
|---|---|---|
| E0 | Fondations : dépôt, documentation, analyse, feuille de route | 🟢 terminée |
| E1 | Simulation Gazebo et acquisition des données capteurs | 🟡 en cours |
| E2 | Cartographie et localisation visuelle (V-SLAM, RTAB-Map) | 🟡 en cours |
| E3 | Navigation guidée par la vision | ⚪ à faire |
| E4 | Perception de la profondeur et évitement d'obstacles | ⚪ à faire |
| E5 | Validation sur le robot réel | ⚪ à faire |
| E6 | Rapport technique et démonstration | ⚪ à faire |

Détail et critères : [feuille de route](docs/04_feuille_de_route.md).

## Structure du dépôt

```
robot-mobile/
├── README.md                     ce fichier : présentation et manuel d'exécution
├── LICENSE.md                    licence Apache-2.0
├── docs/                         documentation scientifique du projet
├── robot_mobile_description/     le robot : TurtleBot3 Waffle + caméra RGB-D
│   ├── urdf/                     description pour ROS (TF, RViz)
│   └── models/robot_mobile/      modèle pour Gazebo (physique, capteurs)
├── robot_mobile_bringup/         lancement et configuration
│   ├── launch/                   simulation, jeu de données, SLAM, évaluation des SLAM
│   ├── config/                   pont Gazebo ↔ ROS 2, trajectoire de référence, slam/
│   ├── worlds/, models/          salle as52_room, cible rouge
│   ├── rviz/                     configuration d'affichage
│   └── scripts/                  génération des textures, comparaison des SLAM
└── robot_mobile_evaluation/      outils d'expérience (Python) et tests
    ├── robot_mobile_evaluation/  contrôle des capteurs, trajectoire, enregistrement, métriques
    └── test/                     tests des métriques (ATE, RPE) et du style
```

Les paquets `robot_mobile_perception` et `robot_mobile_control` seront créés à l'étape E3
(voir l'[architecture cible](docs/03_architecture_cible.md#5-organisation-proposée-du-dépôt)).

## Environnement requis

| Composant | Version |
|---|---|
| Ubuntu | 26.04 LTS |
| ROS 2 | Lyrical Luth |
| Gazebo | Jetty (gz-sim 10) |
| Python | 3.14 |

Liste complète, paquets à installer et points de vigilance :
[docs/01_environnement_logiciel.md](docs/01_environnement_logiciel.md).

## Installation et compilation

```bash
# 1. Charger ROS 2
source /opt/ros/lyrical/setup.bash

# 2. Récupérer le dépôt
git clone https://github.com/essomanam/robot-mobile.git
cd robot-mobile

# 3. Installer les dépendances déclarées par les paquets
#    (première fois seulement : sudo rosdep init && rosdep update)
rosdep install --from-paths . --ignore-src -y

# 4. Compiler, puis charger le résultat
colcon build --symlink-install
source install/setup.bash
```

Vérification : `ros2 pkg prefix robot_mobile_bringup` doit afficher le chemin du paquet installé.

## Utilisation

Dans chaque terminal : `source /opt/ros/lyrical/setup.bash && source install/setup.bash`.

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
ros2 run robot_mobile_evaluation analyze_dataset ~/as52_data/rosbags/<enregistrement> docs/figures
```

La première commande lance la simulation sans fenêtre, fait faire au robot un tour de salle
scripté et l'enregistre (rosbag MCAP dans `~/as52_data/rosbags/`) ; elle s'arrête seule.
La seconde calcule les fréquences des capteurs et compare l'odométrie à la vérité terrain.

### Cartographie et localisation (SLAM, étape E2)

Pendant la simulation (autre terminal), construire une carte avec RTAB-Map :

```bash
ros2 launch robot_mobile_bringup slam.launch.py                       # RTAB-Map RGB-D + roues
ros2 launch robot_mobile_bringup slam.launch.py method:=slam_toolbox  # autre méthode
```

Méthodes : `rtabmap_rgbd` (caméra RGB-D + roues), `rtabmap_rgbd_vo` (caméra seule),
`rtabmap_rgbd_scan` (caméra + LiDAR + roues), `slam_toolbox` (LiDAR + roues). La carte
RTAB-Map est enregistrée dans `~/.ros/rtabmap.db` (`database_path:=` pour la changer).

Comparer les méthodes sur le jeu de données de référence (≈ 15 min, sans simulation) :

```bash
robot_mobile_bringup/scripts/run_slam_comparison.sh ~/as52_data/slam_runs/<essai>
```

Chaque méthode rejoue le même rosbag ; le script produit un tableau d'erreurs (ATE, RPE) et des
figures (trajectoires, erreurs, cartes) dans `~/as52_data/slam_runs/<essai>/`.

*Les scénarios des étapes suivantes (navigation visuelle, évitement) seront ajoutés ici.*

## Auteur

essomanam — Master 2 RSA, UTBM. Encadrant : Mohamed KAS.

## Licence

Apache-2.0 — voir [LICENSE.md](LICENSE.md).
