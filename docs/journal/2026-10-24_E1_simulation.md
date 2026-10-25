# Journal — 2026-10-24 — E1 : simulation RGB-D et jeu de données de référence

| Champ | Valeur |
|---|---|
| Durée | une séance (suite de la séance E0) |
| Étape(s) concernée(s) | E1 — [fiche](../etapes/E1_simulation.md) |

## Objectif de la séance

Démarrer E1 : réorganiser le dépôt en paquets, créer le robot simulé avec caméra RGB-D et la
salle du projet, vérifier les capteurs par la mesure, enregistrer un jeu de données de référence.

## Ce qui a été fait

- Choix de la scène : **monde dédié** (choix de l'étudiant) → [D-010](../decisions.md#d-010--scène-simulée-dédiée-as52_room).
- Réorganisation du dépôt ([D-006](../decisions.md#d-006--dépôt-organisé-en-plusieurs-paquets)) : paquet modèle `robot_mobile` retiré (conservé dans l'historique Git), création de `robot_mobile_description` et `robot_mobile_bringup`.
- Modèle du robot : URDF + SDF dérivés du Waffle, caméra RGB-D → [D-008](../decisions.md#d-008--description-du-robot--sdf-pour-gazebo-urdf-pour-ros), [D-009](../decisions.md#d-009--caméra-rgb-d-simulée-et-noms-des-topics).
- Salle `as52_room` avec textures générées par script, mobilier, cible rouge.
- Pont Gazebo ↔ ROS 2, launch de simulation, configuration RViz.
- Outils : `sensor_check.py` (contrôle des capteurs), `reference_trajectory.py` (trajectoire scriptée),
  `reference_dataset.launch.py` (enregistrement en une commande), `analyze_dataset.py` (analyse).
- Mesures : fréquences, facteur temps réel, biais et bruit du LiDAR et de la profondeur, arbre TF,
  enregistrement de référence et dérive de l'odométrie.
- Correction de la doc d'environnement : `rqt_image_view`, `vision_msgs`, `tf2_tools`, `xacro` et le stockage MCAP
  étaient déjà installés (la première vérification lisait mal la sortie française d'`apt`).

## Problèmes rencontrés et solutions

| Problème | Cause identifiée | Solution / contournement |
|---|---|---|
| `gz sdf -k` ne trouve pas `model://red_target` | L'outil de validation cherche les modèles dans `SDF_PATH`, Gazebo dans `GZ_SIM_RESOURCE_PATH` | Définir `SDF_PATH` pour valider ; rien à changer pour Gazebo |
| Avertissement `gz_frame_id is deprecated` | Élément renommé `frame_id` dans SDF 1.12 (Gazebo Jetty) ; le modèle TurtleBot3 officiel utilise encore l'ancien nom | Modèle du robot passé en SDF 1.12 avec `<frame_id>` |
| Textures `Wood_Floor_Dark.jpg`, `Maple.jpg` introuvables | Absentes du paquet `turtlebot3_gazebo` (table reprise de `turtlebot3_house`) | Aucune : seule l'apparence de la table est touchée |
| Premier enregistrement vide | Sous Lyrical, `ros2 bag record` n'accepte plus les topics en arguments positionnels | Option `--topics` ajoutée dans `reference_dataset.launch.py` |
| `parameter_bridge` plante à l'arrêt (signal 11) | Défaut de `ros_gz_bridge` à la fermeture, reproductible | Sans effet sur les données (enregistrement déjà finalisé) ; à signaler si gênant |
| Serveur `gz-sim-main` orphelin après l'arrêt | Le lanceur tue le script `gz` mais pas le serveur qu'il a démarré | Après usage : `pkill -x gz-sim-main` (noté dans la fiche E1) |
| Un `pkill -f` a interrompu la commande qui l'appelait | Le motif apparaissait dans sa propre ligne de commande | Arrêter les processus par leur PID ou par nom exact (`pkill -x`) |

## Décisions prises

- D-008 à D-011 : **acceptées** (détail dans le [registre](../decisions.md)).

## Questions ouvertes

- Pour moi : faire le contrôle visuel dans RViz (nuage de profondeur superposé au scan) avec la simulation affichée.
- Pour le superviseur : toujours Q1 (caméra du robot réel) et Q2 (version ROS 2 du robot).

## Prochaine séance

- Valider la fiche E1, commiter.
- Démarrer E2 : installer RTAB-Map (`sudo apt install ros-lyrical-rtabmap-ros`) et slam_toolbox, rejouer le jeu de données de référence.
