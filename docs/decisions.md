# Registre des décisions

> **Résumé en 3 lignes**
> 1. Chaque choix important du projet est noté ici, avec le contexte, les options envisagées et la raison du choix.
> 2. Ce registre sert à justifier les choix dans le rapport (livrable LIV-2) et à ne pas rediscuter sans fin ce qui a déjà été tranché.
> 3. Une décision n'est jamais effacée : si elle change, on en écrit une nouvelle qui la *remplace*.

## Mode d'emploi

**Statuts :** `proposée` (à valider) · `acceptée` (en vigueur) · `remplacée par D-xxx` · `abandonnée`.

**Format d'une entrée :**

```
### D-xxx — Titre court
| Date | Statut | Étape |
- Contexte : quel problème, quelles contraintes
- Options : ce qui était possible
- Décision : ce qui est retenu
- Justification : pourquoi
- Conséquences : ce que ça implique, ce qu'il faudra surveiller
```

## Index

| ID | Titre | Statut | Date |
|---|---|---|---|
| [D-001](#d-001--plateforme-ubuntu-2604--ros-2-lyrical--gazebo-jetty) | Plateforme Ubuntu 26.04 + ROS 2 Lyrical + Gazebo Jetty | acceptée | 2026-10-24 |
| [D-002](#d-002--dépôt-du-projet-et-bac-à-sable) | Dépôt du projet et bac à sable | acceptée | 2026-10-24 |
| [D-003](#d-003--documentation-en-français-en-markdown-dans-docs) | Documentation en français, en Markdown, dans `docs/` | acceptée | 2026-10-24 |
| [D-004](#d-004--hypothèse-sur-le-robot-du-laboratoire) | Hypothèse sur le robot du laboratoire | proposée | 2026-10-24 |
| [D-005](#d-005--rtab-map-comme-outil-de-v-slam-principal) | RTAB-Map comme outil de V-SLAM principal | acceptée | 2026-10-24 |
| [D-006](#d-006--dépôt-organisé-en-plusieurs-paquets) | Dépôt organisé en plusieurs paquets | acceptée | 2026-10-24 |
| [D-007](#d-007--nommage-robot-mobile--robot_mobile) | Nommage `robot-mobile` / `robot_mobile` | acceptée | 2026-10-24 |
| [D-008](#d-008--description-du-robot--sdf-pour-gazebo-urdf-pour-ros) | Description du robot : SDF pour Gazebo, URDF pour ROS | acceptée | 2026-10-24 |
| [D-009](#d-009--caméra-rgb-d-simulée-et-noms-des-topics) | Caméra RGB-D simulée et noms des topics | acceptée | 2026-10-24 |
| [D-010](#d-010--scène-simulée-dédiée-as52_room) | Scène simulée dédiée `as52_room` | acceptée | 2026-10-24 |
| [D-011](#d-011--jeu-de-données-de-référence-scripté-et-enregistré) | Jeu de données de référence scripté et enregistré | acceptée | 2026-10-24 |
| [D-012](#d-012--paquet-robot_mobile_evaluation-pour-les-outils-dexpérience) | Paquet `robot_mobile_evaluation` pour les outils d'expérience | acceptée | 2026-10-25 |
| [D-013](#d-013--protocole-de-comparaison-des-slam) | Protocole de comparaison des SLAM | acceptée | 2026-10-25 |
| [D-014](#d-014--configuration-slam-retenue-pour-la-suite) | Configuration SLAM retenue pour la suite | proposée | 2026-10-25 |

---

### D-001 — Plateforme Ubuntu 26.04 + ROS 2 Lyrical + Gazebo Jetty

| Date | Statut | Étape |
|---|---|---|
| 2026-10-24 | acceptée | E0 |

- **Contexte** : la machine de développement est sous Ubuntu 26.04, avec ROS 2 Lyrical et Gazebo Jetty déjà installés.
- **Options** : (a) garder Lyrical ; (b) installer une version plus ancienne (Jazzy, Humble), prévue pour Ubuntu 24.04 / 22.04, via un conteneur Docker ; (c) réinstaller un autre système.
- **Décision** : (a) Lyrical pour le développement en simulation.
- **Justification** : déjà installé et fonctionnel ; tous les paquets nécessaires existent pour Lyrical (RTAB-Map, Nav2, slam_toolbox, ArUco, TurtleBot3 simulation) ; pas de temps perdu en réinstallation.
- **Conséquences** : risque d'incompatibilité avec la version de ROS du robot réel (risque R1 de la [feuille de route](04_feuille_de_route.md#4-risques)) ; à vérifier tôt, avec Docker en solution de repli. Versions exactes dans [01_environnement_logiciel.md](01_environnement_logiciel.md).

### D-002 — Dépôt du projet et bac à sable

| Date | Statut | Étape |
|---|---|---|
| 2026-10-24 | acceptée | E0 |

- **Contexte** : les prototypes sont dans `AS52/dev_ws`, non versionné ; un dépôt GitHub `robo-mobile` (modèle de paquet presque vide) existe déjà.
- **Options** : (a) faire de `dev_ws` le dépôt ; (b) partir du dépôt existant ; (c) créer un nouveau dépôt.
- **Décision** : (b) le dépôt existant, renommé `robot-mobile` ([D-007](#d-007--nommage-robot-mobile--robot_mobile)), est **le** dépôt du projet. `dev_ws` reste un **bac à sable** d'essais, non modifié.
- **Justification** : dépôt déjà relié à GitHub ; dépôt propre dès le départ, sans les exercices de prise en main.
- **Conséquences** : les idées des prototypes sont reprises *proprement* dans le dépôt à l'étape E3, pas copiées telles quelles. Leur analyse est dans l'[état des lieux](02_etat_des_lieux.md).

### D-003 — Documentation en français, en Markdown, dans `docs/`

| Date | Statut | Étape |
|---|---|---|
| 2026-10-24 | acceptée | E0 |

- **Contexte** : le projet doit être documenté de façon scientifique, au fil de l'eau, pour alimenter le rapport et les présentations, et être lisible par un non-spécialiste.
- **Options** : (a) Markdown dans le dépôt ; (b) document Word / LaTeX séparé ; (c) wiki GitHub.
- **Décision** : (a) fichiers Markdown en français dans `docs/`, diagrammes en Mermaid, une fiche par étape selon un [modèle commun](modeles/modele_fiche_etape.md), un journal daté, ce registre.
- **Justification** : versionné avec le code (la doc et le code évoluent ensemble) ; lisible directement sur GitHub et dans VS Code ; Mermaid est rendu par GitHub sans outil supplémentaire ; facile à convertir en chapitres de rapport.
- **Conséquences** : noms de variables et de fichiers de code en anglais (convention ROS), commentaires et documentation en français.

### D-004 — Hypothèse sur le robot du laboratoire

| Date | Statut | Étape |
|---|---|---|
| 2026-10-24 | proposée — à confirmer par le superviseur (Q1) | E1 |

- **Contexte** : le sujet cite une caméra RGB-D et un LiDAR, mais le modèle exact du robot du labo n'est pas connu.
- **Options** : (a) TurtleBot3 Waffle / Waffle Pi + caméra RGB-D + LiDAR ; (b) Waffle Pi avec sa caméra Raspberry Pi (couleur seule) ; (c) Burger + caméra.
- **Décision** : (a) en hypothèse de travail pour la simulation.
- **Justification** : c'est la configuration décrite par le sujet ; le Waffle a la place et la charge utile pour une caméra RGB-D.
- **Conséquences** : aucun modèle TurtleBot3 fourni avec Gazebo n'a de caméra de profondeur ; il faudra en créer un (E1). Si l'hypothèse est fausse : voir le risque R2 de la [feuille de route](04_feuille_de_route.md#4-risques).

### D-005 — RTAB-Map comme outil de V-SLAM principal

| Date | Statut | Étape |
|---|---|---|
| 2026-10-24 | acceptée avec la feuille de route | E2 |

- **Contexte** : le sujet cite RTAB-Map et ORB-SLAM comme exemples d'outils de V-SLAM.
- **Options** :

  | Critère | RTAB-Map | ORB-SLAM3 |
  |---|---|---|
  | Disponible en paquet ROS 2 Lyrical | oui (`ros-lyrical-rtabmap-ros`) | non (compilation depuis les sources, adaptateur ROS 2 non officiel) |
  | Caméra RGB-D | oui | oui |
  | Fusion odométrie roues + LiDAR | oui | non |
  | Produit une carte 2D utilisable par Nav2 | oui | non (carte de points 3D épars) |
  | Mode localisation sur carte sauvegardée | oui | oui |

- **Décision** : RTAB-Map en outil principal ; **slam_toolbox** (LiDAR seul) comme référence de comparaison.
- **Justification** : intégration directe avec ROS 2 et Nav2, fusion des capteurs du TurtleBot3. La comparaison avec un SLAM LiDAR permet de mesurer ce qu'apporte la vision.
- **Conséquences** : la qualité dépendra de la texture de la scène (risque R3). Explications dans [notions/slam_vslam.md](notions/slam_vslam.md).

### D-006 — Dépôt organisé en plusieurs paquets

| Date | Statut | Étape |
|---|---|---|
| 2026-10-24 | acceptée avec la feuille de route — à appliquer au début de E1 | E1 |

- **Contexte** : le dépôt contient aujourd'hui un seul paquet (`robot_mobile`) à la racine ; le projet comprendra de la configuration, un modèle de robot, des nœuds de perception et des nœuds de commande.
- **Options** : (a) un seul paquet qui contient tout ; (b) plusieurs paquets spécialisés dans des sous-dossiers.
- **Décision** : (b) `robot_mobile_description`, `robot_mobile_bringup`, `robot_mobile_perception`, `robot_mobile_control` (détail dans l'[architecture cible](03_architecture_cible.md#5-organisation-proposée-du-dépôt)).
- **Justification** : chaque paquet a une responsabilité claire ; la perception se teste sans la commande ; les paquets Python (`ament_python`) et de configuration (`ament_cmake`) ne se mélangent pas.
- **Conséquences** : le paquet `robot_mobile` actuel sera déplacé ou fusionné dans `robot_mobile_bringup` / `robot_mobile_description`.

### D-007 — Nommage `robot-mobile` / `robot_mobile`

| Date | Statut | Étape |
|---|---|---|
| 2026-10-24 | acceptée | E0 |

- **Contexte** : le dépôt avait été créé sous le nom `robo-mobile` (paquet `robo_mobile`), avec un « t » oublié.
- **Décision** : renommer en **`robot-mobile`** pour le dépôt et le dossier, **`robot_mobile`** pour le paquet ROS.
- **Justification** : un nom de paquet ROS n'accepte que des minuscules, des chiffres et le tiret bas `_` (REP 144) ; le tiret `-` est d'usage pour les noms de dépôts.
- **Conséquences** : dossier local renommé, `package.xml`, `CMakeLists.txt` et `launch/rsp.launch.py` modifiés, adresse du dépôt distant mise à jour vers `github.com/essomanam/robot-mobile`. Le dépôt doit aussi être renommé sur GitHub (Settings → Repository name).

### D-008 — Description du robot : SDF pour Gazebo, URDF pour ROS

| Date | Statut | Étape |
|---|---|---|
| 2026-10-24 | acceptée | E1 |

- **Contexte** : il faut un TurtleBot3 Waffle équipé d'une caméra RGB-D, à la fois pour Gazebo (physique, capteurs) et pour ROS (arbre TF, affichage dans RViz).
- **Options** : (a) un seul fichier URDF/xacro avec des balises `<gazebo>`, converti par Gazebo ; (b) un modèle SDF pour Gazebo et un URDF pour ROS, comme le fait `turtlebot3_gazebo` ; (c) le modèle Waffle d'origine, sans profondeur.
- **Décision** : (b) `robot_mobile_description/models/robot_mobile/model.sdf` + `robot_mobile_description/urdf/robot_mobile.urdf.xacro`.
- **Justification** : schéma éprouvé sur cette version de Gazebo ; contrôle direct de la physique (roulettes en liaison rotule, frottements) et des capteurs, sans dépendre de la conversion URDF → SDF (qui fusionne les liaisons fixes et traduit approximativement les frottements). (c) est exclu car il n'a pas de profondeur.
- **Conséquences** : deux fichiers à garder cohérents (rappel en tête de chacun) ; la cohérence est **vérifiée par mesure** avec `sensor_check.py`, qui compare les distances mesurées aux distances calculées à partir des positions TF. En passant, deux défauts du modèle d'origine ont été corrigés : `base_footprint` placé 1 cm au-dessus du sol, et images horodatées hors du repère optique.

### D-009 — Caméra RGB-D simulée et noms des topics

| Date | Statut | Étape |
|---|---|---|
| 2026-10-24 | acceptée | E1 |

- **Contexte** : aucun modèle TurtleBot3 fourni n'a de caméra de profondeur ; le matériel réel n'est pas encore connu ([D-004](#d-004--hypothèse-sur-le-robot-du-laboratoire)).
- **Options** : résolution 1920 × 1080 (caméra du Waffle d'origine) ou 640 × 480 ; 30 ou 15 images/s ; noms de topics de Gazebo ou des pilotes réels.
- **Décision** : capteur Gazebo `rgbd_camera`, **640 × 480, champ horizontal 69,4°, 15 images/s**, profondeur 0,2 à 8 m recalée sur la couleur, publiée dans `camera_rgb_optical_frame` ; topics `/camera/color/{image_raw,camera_info}` et `/camera/depth/{image_raw,camera_info}`.
- **Justification** : 640 × 480 est la résolution de travail usuelle de RTAB-Map et d'une RealSense D435 (dont on reprend le champ de vision couleur) ; 15 images/s suffisent à 0,2 m/s (1,3 cm parcouru entre deux images) et divisent par deux le calcul et le volume de données ; en RGB-D brut, 30 images/s représenteraient 64 Mo/s, impossibles à transmettre en Wi-Fi depuis le robot réel. Les noms de topics des pilotes réels évitent de modifier le code en E5.
- **Conséquences** : le pilote de la caméra réelle devra être réglé de la même façon. La profondeur simulée a un bruit constant (≈ 6 mm), plus faible et plus régulier qu'une vraie caméra dont l'erreur croît avec la distance : écart sim / réel à analyser en E5.

### D-010 — Scène simulée dédiée `as52_room`

| Date | Statut | Étape |
|---|---|---|
| 2026-10-24 | acceptée | E1 |

- **Contexte** : le SLAM visuel a besoin de détails visuels ; les étapes E3 et E4 ont besoin de scénarios maîtrisés.
- **Options** : (a) monde `turtlebot3_house` fourni (15 × 10,5 m, briques répétitives, dépend de modèles en ligne) ; (b) salle construite pour le projet ; (c) les deux.
- **Décision** : (b), choisie par l'étudiant : salle de 7 × 5 m, murs de 1,5 m, textures générées par script (`generate_textures.py`, graine fixe), deux piliers, table, poubelle, cible rouge.
- **Justification** : taille adaptée à la portée du LiDAR (3,5 m) ; textures **toutes différentes** (pas de lieux qui se ressemblent) et **sans rouge saturé** (pas de fausse détection de la cible) ; aucune dépendance Internet ; positions des objets connues exactement, donc distances attendues calculables.
- **Conséquences** : les noms de modèles et de fichiers suivent la convention anglaise ([D-003](#d-003--documentation-en-français-en-markdown-dans-docs)) : `as52_room`, `red_target`. Deux textures de la table reprise de `turtlebot3_house` sont absentes du paquet TurtleBot3 : la table s'affiche sans texture (aucun effet sur la géométrie).

### D-011 — Jeu de données de référence scripté et enregistré

| Date | Statut | Étape |
|---|---|---|
| 2026-10-24 | acceptée | E1 |

- **Contexte** : les étapes E2 (SLAM) et E3 (perception) doivent pouvoir être développées et **comparées sur les mêmes données**.
- **Options** : téléopération à la main ; trajectoire scriptée ; enregistrement ou non.
- **Décision** : trajectoire scriptée (tour de la salle, retour au départ) définie dans `config/reference_trajectory.yaml`, enregistrée en rosbag MCAP compressé zstd, avec la vérité terrain ; lancement en une commande (`reference_dataset.launch.py`). Données stockées **hors Git** dans `~/as52_data/rosbags/`.
- **Justification** : une trajectoire scriptée est identique d'un essai à l'autre, contrairement à une téléopération ; l'enregistrement permet de rejouer exactement les mêmes capteurs pour comparer deux algorithmes ; le retour au départ offre une fermeture de boucle au SLAM ; la taille (2,2 Go pour un tour de 103 s) exclut Git.
- **Conséquences** : la trajectoire est en boucle ouverte, donc pas exactement le rectangle prévu, mais la vérité terrain enregistrée donne la trajectoire exacte.

### D-012 — Paquet `robot_mobile_evaluation` pour les outils d'expérience

| Date | Statut | Étape |
|---|---|---|
| 2026-10-25 | acceptée | E2 |

- **Contexte** : en E1, les scripts (contrôle des capteurs, trajectoire de référence, analyse) ont été placés dans `robot_mobile_bringup`, alors que l'[architecture](03_architecture_cible.md#5-organisation-proposée-du-dépôt) réserve ce paquet à la configuration. E2 ajoute de nouveaux outils (enregistrement des trajectoires, métriques) qui partagent du code.
- **Options** : (a) continuer dans `bringup` ; (b) un paquet Python dédié.
- **Décision** : (b) paquet `ament_python` **`robot_mobile_evaluation`** : modules partagés (`room.py` : plan de la salle ; `trajectory.py` : format TUM, alignement, ATE, RPE) et exécutables (`sensor_check`, `reference_trajectory`, `analyze_dataset`, `trajectory_logger`, `evaluate_slam`).
- **Justification** : respect de la règle « un paquet, une responsabilité » ; plan de la salle et calculs d'erreur écrits **une seule fois** ; **tests unitaires** sur les métriques (une métrique fausse fausserait toutes les comparaisons).
- **Conséquences** : les commandes changent (`ros2 run robot_mobile_evaluation <outil>`, sans `.py`) ; README et fiche E1 mis à jour.

### D-013 — Protocole de comparaison des SLAM

| Date | Statut | Étape |
|---|---|---|
| 2026-10-25 | acceptée | E2 |

- **Contexte** : il faut montrer, chiffres à l'appui, ce qu'apporte le SLAM visuel, et justifier le choix de configuration ([D-005](#d-005--rtab-map-comme-outil-de-v-slam-principal)).
- **Décision** : rejouer **le même rosbag de référence** (E1) avec chaque méthode : A odométrie des roues seule, B RTAB-Map RGB-D + roues, C RTAB-Map RGB-D seule (odométrie visuelle), D RTAB-Map RGB-D + LiDAR + roues, E slam_toolbox (LiDAR + roues). Mesures : ATE (RMSE après alignement rigide), RPE sur 1 m, erreur finale (dérive accumulée), ATE du graphe optimisé, fermetures de boucle, temps de calcul, carte superposée au plan réel. La configuration principale est répétée pour mesurer la variabilité d'un essai à l'autre.
- **Justification** : mêmes données pour tous = comparaison loyale ; ATE / RPE sont les métriques de référence de la littérature (Sturm et al., 2012) ; métriques implémentées dans le projet (transparentes, testées) et trajectoires exportées au format TUM pour un contrôle croisé possible avec `evo`.
- **Conséquences** : chaque essai s'isole d'une éventuelle simulation en cours par un `ROS_DOMAIN_ID` dédié ; résultats bruts stockés hors Git dans `~/as52_data/slam_runs/`.

### D-014 — Configuration SLAM retenue pour la suite

| Date | Statut | Étape |
|---|---|---|
| 2026-10-25 | proposée — à valider par l'étudiant | E2 → E3, E4 |

- **Contexte** : comparaison E2 de cinq méthodes sur le même jeu de données ([fiche E2](etapes/E2_vslam.md#81-comparaison-des-méthodes-configuration-finale)).
- **Options** : B RTAB-Map RGB-D + roues (ATE 3,2 cm) ; C caméra seule (fragile, perte de suivi) ; D RTAB-Map RGB-D + LiDAR + roues (ATE 1,0 cm, erreur finale 0,4 cm) ; E slam_toolbox (ATE 1,3 cm, erreur finale 3 cm).
- **Décision proposée** : **D** (`method:=rtabmap_rgbd_scan`) pour la navigation des étapes E3 et E4, avec les réglages finaux (incertitude d'odométrie explicite, `Vis/MaxDepth` = 4 m) ; E (slam_toolbox) comme solution de repli si le calcul de RTAB-Map posait problème sur le matériel réel.
- **Justification** : meilleure précision et meilleure carte, faible variabilité (3 essais) ; utilise la caméra RGB-D **et** le LiDAR comme le demande le sujet ; la vision apporte la reconnaissance de lieux, le LiDAR la précision locale.
- **Conséquences** : à confirmer en mode localisation (E2, partie 2) ; `slam.launch.py` garde `rtabmap_rgbd` par défaut tant que cette décision n'est pas validée.
