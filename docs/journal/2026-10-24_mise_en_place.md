# Journal — 2026-10-24 — Mise en place des fondations

| Champ | Valeur |
|---|---|
| Durée | une séance |
| Étape(s) concernée(s) | E0 — [fiche](../etapes/E0_fondations.md) |

## Objectif de la séance

Repartir sur une base solide : relire le sujet en entier, faire l'inventaire de ce qui existe,
organiser le dépôt et mettre en place une documentation scientifique qui suivra tout le projet.

## Ce qui a été fait

- Lecture du sujet, de l'article [THALE2020] et du cours [ROB201] ; reformulation en exigences
  numérotées → [00_analyse_du_sujet.md](../00_analyse_du_sujet.md).
- Inventaire de l'environnement : ROS 2 Lyrical, Gazebo Jetty 10.5.0, Fast DDS ; paquets installés
  et disponibles → [01_environnement_logiciel.md](../01_environnement_logiciel.md).
- Lecture des modèles TurtleBot3 de `turtlebot3_gazebo` : capteurs, fréquences, repères.
- Analyse des prototypes de `dev_ws` → [02_etat_des_lieux.md](../02_etat_des_lieux.md).
- Architecture cible et feuille de route proposées, puis **validées** → [03](../03_architecture_cible.md), [04](../04_feuille_de_route.md).
- Renommage du dépôt `robo-mobile` → `robot-mobile` (paquet `robot_mobile`) → [D-007](../decisions.md#d-007--nommage-robot-mobile--robot_mobile).
- Création de `.gitignore`, réécriture du `README.md`, création de `docs/`.

## Vérifications effectuées

| Vérification | Résultat |
|---|---|
| `colcon build --symlink-install` | ✅ paquet `robot_mobile` compilé (1 avertissement CMake, voir ci-dessous) |
| `ros2 pkg prefix robot_mobile` | ✅ `…/robot-mobile/install/robot_mobile` |
| `ros2 launch robot_mobile rsp.launch.py` | ✅ `robot_state_publisher` démarre (« Robot initialized ») |
| Liens internes de la documentation (fichiers et ancres) | ✅ tous valides |
| Ancien nom dans les fichiers du dépôt | ✅ uniquement dans les mentions historiques volontaires |
| `git status` | ✅ `build/ install/ log/` ignorés |

## Problèmes rencontrés et solutions

| Problème | Cause identifiée | Solution / contournement |
|---|---|---|
| Aucun modèle TurtleBot3 simulé avec caméra de profondeur | Le Waffle simulé a une caméra de type `camera` (couleur seule) malgré son nom « realsense_r200 » | Créer un modèle avec un capteur `rgbd_camera` à l'étape E1 |
| Images de la caméra simulée horodatées dans `camera_rgb_frame` | Réglage `gz_frame_id` du modèle fourni | Utiliser le repère optique dans notre modèle (E1) |
| Avertissement CMake à la compilation | Le modèle de paquet déclare `cmake_minimum_required(VERSION 3.5)`, jugé trop ancien | Sans effet aujourd'hui ; à relever à `3.20` lors de la réorganisation en paquets (E1) |
| `rosdep` non initialisé | Jamais fait sur cette machine | `sudo rosdep init && rosdep update` (à faire une fois) |

## Décisions prises

- D-001 plateforme Lyrical, D-002 dépôt et bac à sable, D-003 documentation, D-007 nommage : **acceptées**.
- Feuille de route E0 → E6 **validée** ; D-005 (RTAB-Map) et D-006 (multi-paquets), qui en font partie, **acceptées** avec elle.
- D-004 hypothèse robot : reste **proposée**, en attente de la réponse du superviseur (Q1).

Détail dans le [registre des décisions](../decisions.md).

## Questions ouvertes

- Pour moi : choisir la scène simulée de E1 (`turtlebot3_house` enrichi de textures, ou monde dédié au projet).
- Pour le superviseur : questions Q1 à Q7 de l'[analyse du sujet](../00_analyse_du_sujet.md#5-questions-ouvertes-pour-le-superviseur),
  en priorité Q1 (robot et caméra) et Q2 (version de ROS 2 du robot).

## Prochaine séance

- Renommer le dépôt sur GitHub (Settings → Repository name → `robot-mobile`), puis pousser.
- Démarrer E1 : réorganisation en paquets ([D-006](../decisions.md#d-006--dépôt-organisé-en-plusieurs-paquets)), modèle RGB-D, monde texturé.
