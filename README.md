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
| E1 | Simulation Gazebo et acquisition des données capteurs | ⚪ à faire |
| E2 | Cartographie et localisation visuelle (V-SLAM, RTAB-Map) | ⚪ à faire |
| E3 | Navigation guidée par la vision | ⚪ à faire |
| E4 | Perception de la profondeur et évitement d'obstacles | ⚪ à faire |
| E5 | Validation sur le robot réel | ⚪ à faire |
| E6 | Rapport technique et démonstration | ⚪ à faire |

Détail et critères : [feuille de route](docs/04_feuille_de_route.md).

## Structure du dépôt

```
robot-mobile/
├── README.md            ce fichier : présentation et manuel d'exécution
├── LICENSE.md           licence Apache-2.0
├── docs/                documentation scientifique du projet
├── package.xml          paquet ROS 2 robot_mobile (ament_cmake)
├── CMakeLists.txt
├── config/              fichiers de paramètres
├── description/         description du robot (URDF / xacro)
├── launch/              fichiers de lancement
└── worlds/              mondes Gazebo
```

Cette structure évoluera à l'étape E1 vers plusieurs paquets (description, bringup, perception,
control) : voir l'[architecture cible](docs/03_architecture_cible.md#5-organisation-proposée-du-dépôt).

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

Vérification : `ros2 pkg prefix robot_mobile` doit afficher le chemin du paquet installé.

## Utilisation

*À venir : les scénarios (simulation, cartographie, navigation visuelle, évitement) seront
ajoutés ici au fil des étapes, chacun avec sa commande de lancement unique et le résultat attendu.*

## Auteur

essomanam — Master 2 RSA, UTBM. Encadrant : Mohamed KAS.

## Licence

Apache-2.0 — voir [LICENSE.md](LICENSE.md).
