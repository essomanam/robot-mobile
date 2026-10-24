# 02 — État des lieux : prise en main et prototypes

> **Résumé en 3 lignes**
> 1. De septembre à octobre 2026, la prise en main de ROS 2 et Gazebo a abouti à trois prototypes : détection d'une cible rouge, suivi de cette cible, évitement d'obstacles au LiDAR.
> 2. Ces prototypes **fonctionnent en simulation** et contiennent de bonnes idées (surveillance des capteurs, arrêt sûr), mais ils ont des limites de conception : code dupliqué, paramètres écrits en dur, pas de fichier de lancement, profondeur non utilisée.
> 3. Ils restent dans le bac à sable `dev_ws` (non versionné) ; leurs idées seront reprises proprement dans ce dépôt à l'étape E3.

État constaté le **2026-10-24**. Le dossier `AS52/dev_ws` n'est pas modifié par ce dépôt : il sert
de terrain d'essai.

---

## 1. Chronologie de la prise en main

Dates tirées des dates de fichiers et des journaux de compilation `colcon`.

| Période | Activité | Ce qui a été appris |
|---|---|---|
| 21 sept. 2026 | Installation de ROS 2, premier workspace `~/ros2_projects/mini_robot_ws` | Installation, `colcon`, structure d'un workspace |
| sept.–oct. | Nœuds de démonstration *talker / listener*, commandes `ros2 topic pub/echo/hz/list` | Nœuds, topics, messages, outils en ligne de commande |
| sept.–oct. | Démo Gazebo `ros_gz_sim_demos diff_drive` (véhicule `vehicle_blue`), téléopération clavier | Pont Gazebo ↔ ROS 2, commande en vitesse d'un robot différentiel |
| sept.–oct. | Démo Nav2 avec SLAM sur TurtleBot4 simulé (`nav2_bringup tb4_simulation_launch.py slam:=True`) | Vue d'ensemble d'une pile de navigation complète |
| 13 oct. | Création du dépôt `robo-mobile` à partir d'un modèle de paquet (renommé `robot-mobile` le 24 oct., [D-007](decisions.md#d-007--nommage-robot-mobile--robot_mobile)) | Dépôt Git, structure d'un paquet `ament_cmake` |
| 17–18 oct. | `dev_ws` : paquet `my_package` (launch talker/listener), paquet `robot_controller`, nœud `move_robot` | Fichiers launch, paquet Python `ament_python`, timer ROS |
| 23 oct. | TurtleBot3 dans Gazebo, pont caméra, nœuds `avoid_obstacles`, `camera_processor`, `target_follower`, modèle `cible_rouge.sdf` | Lecture LiDAR et caméra, OpenCV, boucle perception → commande |

## 2. Inventaire du bac à sable `dev_ws`

| Fichier | Rôle | Robot visé |
|---|---|---|
| `src/my_package/launch/{talker,listner}.launch.py` | Lancer les nœuds de démonstration | — |
| `src/robot_controller/robot_controller/move_robot.py` | Parcourir un carré en boucle ouverte | `vehicle_blue` (démo ros_gz) |
| `src/robot_controller/robot_controller/avoid_obstacles.py` | Avancer et tourner devant un obstacle (LiDAR) | TurtleBot3 |
| `src/robot_controller/robot_controller/camera_processor.py` | Détecter des zones rouges dans l'image et les afficher | TurtleBot3 |
| `src/robot_controller/robot_controller/target_follower.py` | Se tourner vers la cible rouge et s'en approcher | TurtleBot3 |
| `models/cible_rouge.sdf` | Pavé rouge statique 0,4 × 0,4 × 0,6 m servant de cible | — |

D'après le code (largeur d'image par défaut de 320 pixels), la caméra utilisée était celle du
modèle simulé **`turtlebot3_burger_cam`** : caméra couleur **grand-angle** 320 × 240 pixels avec un
champ de vision horizontal d'environ 182°, sans profondeur.

## 3. Analyse de chaque prototype

### 3.1 `move_robot.py` — carré en boucle ouverte

**Principe.** Une machine à états (`AVANCER` → `TOURNER`, quatre fois) envoie des vitesses
pendant un nombre fixé de pas de 0,1 s :

- côté : 20 pas × 0,1 s × 0,5 m/s = **1,0 m** ;
- virage : 16 pas × 0,1 s × 1,0 rad/s = **1,6 rad ≈ 91,7°**, au lieu de 90° (π/2 ≈ 1,571 rad).

**Enseignement.** C'est une commande en **boucle ouverte** : le robot ne mesure pas ce qu'il fait
réellement. L'erreur de 1,7° par virage s'ajoute aux glissements et aux retards du minuteur, si
bien que le carré ne se referme pas. Cet exercice montre pourquoi il faut une **boucle fermée**
(mesurer, comparer à la consigne, corriger), qui est le principe de tous les nœuds suivants.

**Limites.** Vise le véhicule de démonstration (`/model/vehicle_blue/cmd_vel`, type `Twist`), pas le
TurtleBot3 (`/cmd_vel`, type `TwistStamped`).

### 3.2 `avoid_obstacles.py` — évitement réactif au LiDAR

**Chaîne de traitement.**

```
/scan (LaserScan, ~10 Hz)
  → angle de chaque mesure ramené dans [-π, π]
  → garder les mesures valides dans le secteur avant ±20°
  → d_avant = minimum de ces distances
  → si d_avant < 0,5 m : tourner sur place à gauche (ω = 0,5 rad/s)
    sinon               : avancer (v = 0,08 m/s)
  → /cmd_vel (TwistStamped, 10 Hz)
```

**Points forts.**
- Mesures invalides écartées (infinies ou hors de `[range_min, range_max]`).
- **Surveillance des capteurs** : si aucun scan n'est arrivé depuis plus d'1 s, le robot reste
  immobile (comportement sûr par défaut).
- Commande d'arrêt envoyée à la fermeture du nœud.

**Limites.**
- **Tourne toujours à gauche**, même si la gauche est plus encombrée : risque de blocage dans
  un coin ou d'aller-retour sans fin.
- **Pas d'hystérésis** : autour de 0,5 m, le robot peut alterner avancer / tourner à chaque cycle.
- **Secteur fixe de ±20°** : à une distance *d*, il couvre une demi-largeur de *d* · tan(20°).
  À 0,5 m cela fait 0,18 m, assez pour un Waffle (demi-largeur ≈ 0,15 m), mais à 0,3 m seulement
  0,11 m : les coins du robot peuvent accrocher un obstacle sur le côté.
- Comportement purement réactif : aucun but, aucune mémoire de l'environnement.

### 3.3 `camera_processor.py` — détection d'une cible rouge

**Chaîne de traitement.**

```
/camera/image_raw (Image)
  → conversion ROS → OpenCV (cv_bridge, BGR 8 bits)
  → conversion BGR → HSV
  → deux seuillages : H ∈ [0, 10] ou H ∈ [170, 179], avec S ≥ 100 et V ≥ 100
  → union des deux masques
  → contours extérieurs ; garder ceux d'aire > 200 pixels
  → rectangle englobant et centre (cx, cy) de chaque contour
  → affichage dans une fenêtre OpenCV + message dans le journal
```

**Pourquoi HSV ?** En BGR, la couleur et la luminosité sont mélangées dans les trois canaux.
L'espace HSV sépare la **teinte** (H), la **saturation** (S, pureté) et la **valeur** (V, luminosité) :
un seuil sur la teinte reste valable quand l'éclairage change un peu. Dans OpenCV, H va de 0 à 179
et le rouge se trouve **aux deux extrémités**, d'où les deux plages. Les seuils S ≥ 100 et V ≥ 100
éliminent les pixels pâles ou sombres.

**Limites.**
- `cv2.imshow` dans le traitement : nécessite un écran (inutilisable sur le robot sans écran) et
  ralentit le traitement de chaque image.
- Le résultat n'est **publié sur aucun topic** : aucun autre nœud ne peut l'utiliser.
- Un message de journal **par contour et par image** (jusqu'à 30 par seconde).
- Pas de filtrage morphologique (ouverture / fermeture) pour éliminer les petits bruits.
- Seuils écrits en dur ; ils devront être réglés à nouveau sous l'éclairage réel.

### 3.4 `target_follower.py` — suivi de la cible

**Chaîne de traitement.** Il **refait la même détection** que `camera_processor.py` (en ne gardant
que le plus grand contour), lit aussi le LiDAR, puis applique toutes les 0,1 s la loi de commande :

$$
e = \frac{c_x - W/2}{W/2} \in [-1, 1]
$$

où $c_x$ est l'abscisse (en pixels) du centre de la cible et $W$ la largeur de l'image.

| Condition | Commande | État affiché |
|---|---|---|
| Capteurs absents ou datant de plus d'1 s | arrêt | `CAPTEURS ABSENTS OU ANCIENS` |
| Cible non détectée | arrêt | `CIBLE ROUGE NON DETECTEE` |
| \|e\| > 0,12 | rotation seule : ω = sat(−0,6 · e, ±0,4 rad/s) | `ROTATION VERS CIBLE` |
| obstacle devant (±20°) à ≤ 0,6 m | arrêt | `OBSTACLE TROP PROCHE` |
| sinon | avance : v = 0,05 m/s | `AVANCEMENT` |

C'est un **régulateur proportionnel** sur l'erreur horizontale, avec une **zone morte** de ±12 %
de la demi-largeur d'image pour ne pas osciller autour du centre.

**Points forts.**
- Surveillance de la fraîcheur des deux capteurs, arrêt sûr, arrêt à la fermeture.
- Journal limité à un message par seconde, avec l'état courant : facile à suivre.

**Limites.**
- **Code de détection dupliqué** avec `camera_processor.py` : toute correction doit être faite deux fois.
- Le robot **tourne puis avance**, jamais les deux à la fois : trajectoire saccadée et lente.
- **Cible perdue = arrêt** : pas de comportement de recherche.
- L'arrêt à 0,6 m utilise le LiDAR, qui ne distingue pas la cible d'un autre obstacle ; la
  **profondeur de la caméra n'est pas utilisée**, alors qu'elle donnerait la distance de la cible elle-même.
- Avec l'objectif grand-angle du `burger_cam`, la relation entre position dans l'image et angle
  réel n'est **pas linéaire** : l'erreur *e* n'est pas proportionnelle à l'écart angulaire.
- Paramètres (seuils HSV, gains, distances, vitesses) écrits en dur.

### 3.5 Éléments annexes

- `models/cible_rouge.sdf` : une **ligne vide précède la déclaration `<?xml …?>`**. La norme XML
  impose que cette déclaration soit tout au début du fichier ; certains lecteurs refusent le fichier.
- `robot_controller/package.xml` : dépendances `python3-opencv` et `python3-numpy` non déclarées ;
  description et licence laissées à `TODO`.
- `my_package/package.xml` : dépendances `demo_node_cpp` / `demo_node_py` mal orthographiées
  (les paquets s'appellent `demo_nodes_cpp` / `demo_nodes_py`).
- Aucun fichier launch ni fichier de paramètres pour les nœuds TurtleBot3 : chaque essai demande
  plusieurs terminaux et commandes manuelles.

## 4. Synthèse : ce qu'on garde, ce qu'on change

| Élément | Décision pour la suite | Étape |
|---|---|---|
| Détection HSV de la cible rouge | **Garder** l'idée ; en faire un nœud de perception unique qui *publie* ses détections et une image de contrôle | E3 |
| Surveillance de la fraîcheur des capteurs + arrêt sûr | **Garder** dans tous les nœuds de commande | E3, E4 |
| Régulateur proportionnel sur l'erreur image | **Garder** comme base ; ajouter rotation et avance simultanées, recherche de cible, distance par la profondeur | E3 |
| Évitement « toujours à gauche » | **Remplacer** par l'évitement de Nav2 (carte de coûts) + une couche de sécurité réactive | E4 |
| Paramètres en dur | **Remplacer** par des paramètres ROS dans des fichiers YAML | E1 → E4 |
| Lancement à la main | **Remplacer** par des fichiers launch, un par scénario | E1 |
| Caméra grand-angle sans profondeur | **Remplacer** par un modèle simulé avec caméra RGB-D | E1 |
| Carré en boucle ouverte | **Abandonner** (exercice terminé) ; à citer dans le rapport pour illustrer la boucle fermée | — |

## 5. Enseignements

1. Une architecture **une fonction = un nœud** évite la duplication et permet de tester la
   perception sans faire bouger le robot.
2. Rendre les nœuds **paramétrables et lançables en une commande** est indispensable pour des
   essais répétables, donc pour des résultats mesurables.
3. Le passage au robot réel (pas d'écran, autre éclairage, autre caméra) doit être anticipé dès
   la simulation.
