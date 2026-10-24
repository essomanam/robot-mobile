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
