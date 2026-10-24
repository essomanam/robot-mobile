# Étape E0 — Fondations du projet

> **Résumé en 3 lignes**
> 1. Avant de développer, poser une base solide : un dépôt propre, une analyse précise du sujet, une méthode de documentation et une feuille de route.
> 2. Méthode : inventaire complet de l'existant (sujet, environnement, prototypes), puis structuration du dépôt et de la documentation autour d'exigences numérotées.
> 3. Résultat : dépôt `robot-mobile` renommé et organisé, documentation scientifique en place, feuille de route validée le 2026-10-24.

| Champ | Valeur |
|---|---|
| Statut | 🟢 terminée |
| Dates | début 2026-10-24 — fin 2026-10-24 |
| Exigences couvertes | LIV-1 (début) |
| Décisions liées | [D-001](../decisions.md#d-001--plateforme-ubuntu-2604--ros-2-lyrical--gazebo-jetty), [D-002](../decisions.md#d-002--dépôt-du-projet-et-bac-à-sable), [D-003](../decisions.md#d-003--documentation-en-français-en-markdown-dans-docs), [D-005](../decisions.md#d-005--rtab-map-comme-outil-de-v-slam-principal), [D-006](../decisions.md#d-006--dépôt-organisé-en-plusieurs-paquets), [D-007](../decisions.md#d-007--nommage-robot-mobile--robot_mobile) |
| Entrées de journal | [2026-10-24](../journal/2026-10-24_mise_en_place.md) |

---

## 1. Objectif

Disposer d'une base de travail qui permette d'avancer vite **et** de justifier chaque choix dans le rapport.

- [x] Le sujet est découpé en exigences vérifiables, chacune reliée à une étape.
- [x] L'environnement logiciel est décrit avec ses versions exactes.
- [x] Les prototypes existants sont analysés (forces, limites, réutilisation).
- [x] Le dépôt a un nom correct, un `.gitignore`, un `README.md` et un dossier `docs/` structuré.
- [x] Un modèle de fiche d'étape et de journal existe.
- [x] La feuille de route est discutée et validée (2026-10-24).

## 2. Pourquoi cette étape ?

Le projet a commencé par une phase d'apprentissage : des essais utiles, mais dispersés et non
versionnés. Le sujet exige un **dépôt documenté** et un **rapport qui justifie les choix**. Sans
méthode dès maintenant, il faudrait reconstituer après coup ce qui a été fait et pourquoi, ce
qui est long et peu fiable.

## 3. Notions nécessaires

Aucune notion technique particulière. Pour lire la suite du projet : [ROS 2](../notions/ros2.md),
[capteurs](../notions/capteurs.md), [SLAM](../notions/slam_vslam.md),
[navigation](../notions/navigation_controle.md), [glossaire](../glossaire.md).

## 4. Choix techniques et justification

| Choix | Retenu | Raison principale | Décision |
|---|---|---|---|
| Plateforme | Ubuntu 26.04 + ROS 2 Lyrical + Gazebo Jetty | déjà installée, tous les paquets nécessaires existent | D-001 |
| Dépôt | `robot-mobile` (GitHub), `dev_ws` reste un bac à sable | dépôt propre dès le départ | D-002 |
| Documentation | Markdown en français dans `docs/`, Mermaid, fiches d'étape | versionnée avec le code, lisible sur GitHub | D-003 |
| Nom | `robot-mobile` / paquet `robot_mobile` | correction du nom, règle de nommage ROS | D-007 |

## 5. Méthodologie

1. **Lecture du sujet** et reformulation en exigences numérotées avec critère de vérification
   → [00_analyse_du_sujet.md](../00_analyse_du_sujet.md).
2. **Inventaire de l'environnement** par commandes en lecture seule (`dpkg-query`, `ros2 doctor`,
   `gz sim --versions`, recherche des paquets disponibles avec `apt-cache`)
   → [01_environnement_logiciel.md](../01_environnement_logiciel.md).
3. **Inventaire des modèles simulés** TurtleBot3 (lecture des fichiers SDF/URDF de
   `turtlebot3_gazebo`) : capteurs, fréquences, repères. Constat clé : aucun modèle RGB-D.
4. **Analyse critique des prototypes** de `dev_ws` : chaîne de traitement, loi de commande, forces,
   limites → [02_etat_des_lieux.md](../02_etat_des_lieux.md).
5. **Conception** de l'architecture cible et de la feuille de route à partir des exigences et des
   constats → [03_architecture_cible.md](../03_architecture_cible.md),
   [04_feuille_de_route.md](../04_feuille_de_route.md).
6. **Mise en place du dépôt** : renommage, `.gitignore`, `README.md`, documentation.

## 6. Implémentation

| Fichier | Modification |
|---|---|
| dossier `AS52/robo-mobile` | renommé `AS52/robot-mobile` ; dossiers `build/ install/ log/` supprimés (régénérés à la compilation) |
| `package.xml` | nom `robot_mobile`, description, mainteneur, licence Apache-2.0 |
| `CMakeLists.txt` | `project(robot_mobile)` |
| `launch/rsp.launch.py` | recherche le paquet `robot_mobile` |
| `.gitignore` | nouveau : artefacts de compilation, caches, rosbags, bases RTAB-Map |
| `README.md` | réécrit : présentation, avancement, structure, compilation |
| `docs/` | nouveau : toute la documentation |
| remote Git `origin` | `https://github.com/essomanam/robot-mobile.git` |

## 7. Protocole de vérification

- `git status` : seuls les fichiers prévus apparaissent ; `build/ install/ log/` sont ignorés.
- Recherche de l'ancien nom (`robo_mobile`, `robo-mobile`) dans les fichiers suivis : aucune occurrence hors mentions historiques.
- Vérification automatique des liens internes de la documentation.
- Compilation : `colcon build --symlink-install` puis `ros2 pkg prefix robot_mobile`.

## 8. Résultats

Voir l'[entrée de journal du 2026-10-24](../journal/2026-10-24_mise_en_place.md) pour le détail des vérifications.

## 9. Analyse et discussion

- La découverte qu'**aucun modèle TurtleBot3 simulé n'a de caméra de profondeur** change le
  contenu de l'étape E1 : il faudra créer un modèle, et non simplement en choisir un.
- La **compatibilité ROS 2 entre le PC (Lyrical) et le robot** est le principal risque technique
  du projet ; il ne coûte rien de le lever tôt en posant la question au superviseur.
- Les prototypes valident les notions de base (HSV, régulateur P, surveillance des capteurs) :
  l'étape E3 partira de là au lieu de repartir de zéro.

## 10. Reproduire

```bash
cd ~/Etudes/A2026/AS52/robot-mobile
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 pkg prefix robot_mobile
```

## 11. Bilan et suites

- Fait : dépôt, documentation, analyse, feuille de route validée.
- Reste (hors E0) : renommer le dépôt sur GitHub et pousser ; envoyer les questions Q1 à Q7 au superviseur.
- Transmis à E1 : réorganisation en paquets (D-006), création du modèle RGB-D, monde texturé.

## 12. Messages clés pour la présentation

1. Le sujet a été découpé en **13 exigences et 3 livrables**, chacun relié à une étape et à une preuve attendue.
2. L'existant a été **analysé de façon critique** : trois prototypes fonctionnels dont on garde les idées, pas le code tel quel.
3. Deux constats orientent la suite : **pas de modèle simulé RGB-D** à disposition, et un **risque de compatibilité ROS 2** avec le robot réel.
4. Chaque étape suivra le même protocole documenté : objectif mesurable → choix justifiés → méthode → résultats chiffrés.
