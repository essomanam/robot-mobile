# 00 — Analyse du sujet

> **Résumé en 3 lignes**
> 1. Le projet demande de rendre un TurtleBot3 autonome en s'appuyant d'abord sur la **vision** (caméra RGB-D) complétée par le LiDAR, en simulation puis sur le robot réel.
> 2. Le sujet est découpé ici en **exigences numérotées** (EX-…, LIV-…), chacune associée à une étape de la feuille de route et à une preuve vérifiable.
> 3. Plusieurs points ne sont pas précisés par le sujet (matériel exact, version de ROS du robot, dates) : ils sont listés à la fin comme questions pour le superviseur.

Source : *Projet AS52 – Robotique Mobile. Perception Visuelle, V-SLAM et Navigation Autonome (TurtleBot 3)*,
Master 2 RSA, UTBM, 2026-2027. Contact : Mohamed KAS.

---

## 1. Le sujet en clair

Un **robot mobile** est une machine qui se déplace seule. Pour être **autonome**, il doit répondre
en permanence à trois questions : *où suis-je ?*, *qu'y a-t-il autour de moi ?* et *comment aller
où je veux sans rien heurter ?*

Le robot du projet est un **TurtleBot3**, petite plateforme pédagogique à deux roues motrices,
équipée d'un **LiDAR** (télémètre laser à 360°) et d'une **caméra**. On le programme avec
**ROS 2**, l'ensemble d'outils logiciels standard de la robotique (voir [notions/ros2.md](notions/ros2.md)).
Avant de toucher au vrai robot, on travaille dans **Gazebo**, un simulateur physique qui reproduit
le robot, ses capteurs et son environnement.

Le sujet insiste sur la **perception visuelle** : c'est l'analyse des images (couleur et profondeur)
qui doit guider le robot, *en temps réel*. Le LiDAR est un capteur complémentaire.

## 2. Ce que demande le sujet

### 2.1 Axes de travail (texte du sujet, résumé)

1. **Prise en main de la simulation** — déployer le robot dans Gazebo dans une scène statique et acquérir les données des capteurs.
2. **Cartographie visuelle (V-SLAM)** — construire une carte de l'environnement et y localiser le robot (ex. RTAB-Map, ORB-SLAM).
3. **Navigation guidée par la vision** — coupler un traitement d'image (détection de cibles, suivi de lignes, reconnaissance de marqueurs…) aux commandes de déplacement.
4. **Perception et évitement** — utiliser la profondeur pour détecter les obstacles *non cartographiés* et produire un évitement dynamique.
5. **Validation matérielle** — porter les développements sur le robot physique et analyser les enjeux du monde réel.

### 2.2 Livrables

1. **Code source** : dépôt Git documenté (scripts de traitement, fichiers de configuration et de lancement) avec un manuel d'exécution détaillé (`README.md`).
2. **Rapport technique** : document synthétique qui *justifie les choix* (traitement d'image, architecture logicielle, configuration des contrôleurs) et *analyse les résultats*.
3. **Démonstration** : validation pratique en laboratoire de la navigation autonome basée sur la vision.

## 3. Exigences numérotées et traçabilité

Chaque exigence reçoit un identifiant stable, réutilisé dans les fiches d'étape et le rapport.
La colonne *Preuve* sera remplie au fil du projet : c'est elle qui montre, à la fin, que tout
le sujet a été traité.

### 3.1 Exigences fonctionnelles

| ID | Exigence (reformulée) | Source | Critère de vérification proposé | Étape | Preuve |
|---|---|---|---|---|---|
| EX-SIM-1 | Le TurtleBot3 est simulé dans Gazebo, dans une scène statique | Axe 1 | Une seule commande `ros2 launch` démarre monde + robot ; le robot répond aux commandes de vitesse | E1 | [E1 § 6 et 10](etapes/E1_simulation.md#6-implémentation) (contrôle visuel : § 8.5) |
| EX-SIM-2 | Les données des capteurs (caméra RGB-D, LiDAR, odométrie, IMU) sont acquises dans ROS 2 | Axe 1 | Topics publiés, fréquences et résolutions mesurées, enregistrement rosbag rejouable | E1 | [E1 § 8](etapes/E1_simulation.md#8-résultats) |
| EX-VSLAM-1 | Une carte de l'environnement est construite par SLAM visuel | Axe 2 | Carte sauvegardée et rechargeable, cohérente avec le monde simulé | E2 | [E2 § 8.1, 8.4](etapes/E2_vslam.md#84-figures-de-la-cartographie), rechargée en § 8.5 |
| EX-VSLAM-2 | Le robot est localisé dans cette carte | Axe 2 | Erreur de trajectoire (ATE/RPE) mesurée par rapport à la vérité terrain du simulateur | E2 | [E2 § 8.1](etapes/E2_vslam.md#81-comparaison-des-méthodes-configuration-finale) et [§ 8.5](etapes/E2_vslam.md#85-localisation-dans-la-carte-partie-2) |
| EX-VIS-1 | Un traitement d'image extrait une information utile à la navigation (cible, ligne ou marqueur) | Axe 3 | Taux de détection et fréquence de traitement mesurés | E3 | — |
| EX-VIS-2 | Cette information pilote les déplacements du robot | Axe 3 | Taux de succès d'atteinte de la cible, temps, erreur de position finale sur N essais | E3 | — |
| EX-EVIT-1 | Les obstacles absents de la carte sont détectés à partir de la profondeur | Axe 4 | Obstacle ajouté après la cartographie détecté dans la carte de coûts / la perception | E4 | — |
| EX-EVIT-2 | Le robot les évite dynamiquement | Axe 4 | Aucune collision sur N essais ; distance minimale aux obstacles mesurée | E4 | — |
| EX-HW-1 | Les développements tournent sur le robot physique | Axe 5 | Mêmes scénarios rejoués sur le robot réel | E5 | — |
| EX-HW-2 | Les écarts simulation / réalité sont analysés | Axe 5 | Comparaison chiffrée sim vs réel et explication des écarts | E5 | — |

### 3.2 Exigences transverses

| ID | Exigence | Critère de vérification proposé | Preuve |
|---|---|---|---|
| EX-TR | Les flux capteurs sont traités **en temps réel** | La boucle perception → commande tient sa fréquence cible (ordre de grandeur : ≥ 10 Hz) ; latence mesurée | — |
| EX-PERC | La navigation repose **principalement sur la perception visuelle** (RGB-D), le LiDAR étant complémentaire | Pour chaque fonction, le capteur utilisé est justifié dans la fiche d'étape | — |
| EX-ROS | La solution est construite sur la pile standard ROS 2 | Paquets ROS 2, fichiers launch, paramètres YAML | — |

### 3.3 Livrables

| ID | Livrable | Critère de vérification proposé | Étape | Preuve |
|---|---|---|---|---|
| LIV-1 | Dépôt Git documenté + `README.md` manuel d'exécution | Un lecteur extérieur relance chaque scénario depuis un clone propre en suivant le README | E0 → E6 | — |
| LIV-2 | Rapport technique justifiant les choix et analysant les résultats | Chaque choix renvoie à une décision du [registre](decisions.md) ; chaque résultat à une fiche d'étape | E6 | — |
| LIV-3 | Démonstration en laboratoire | Scénario de démonstration défini, répété et chronométré avant le jour J | E5, E6 | — |

## 4. Interprétation et cadrage

Certains termes du sujet laissent de la marge. Voici comment ils sont interprétés ici ; ces
interprétations seront confirmées avec le superviseur.

- **« V-SLAM »** : on vise un SLAM *RGB-D* (images couleur + profondeur), qui peut intégrer
  l'odométrie des roues et le LiDAR. RTAB-Map le permet nativement ; ORB-SLAM est cité comme
  alternative (comparaison dans [notions/slam_vslam.md](notions/slam_vslam.md)).
- **« Temps réel »** : temps réel *souple* (pas de garantie stricte d'échéance) : chaque image
  est traitée avant l'arrivée de la suivante et la boucle de commande garde une fréquence stable.
- **« Obstacles non cartographiés »** : objets absents lors de la construction de la carte et
  ajoutés ensuite (en simulation : objets insérés dans Gazebo après la cartographie).
- **« Navigation guidée par la vision »** : on commencera par une cible colorée (déjà prototypée,
  voir [état des lieux](02_etat_des_lieux.md)), puis des marqueurs ArUco, plus robustes et qui
  donnent aussi une distance et une orientation.

## 5. Questions ouvertes pour le superviseur

| # | Question | Pourquoi c'est important |
|---|---|---|
| Q1 | Quel modèle exact de TurtleBot3 et quelle caméra (RGB-D ? laquelle ?) sont disponibles au labo ? | Détermine le modèle simulé (hypothèse actuelle : Waffle + RGB-D, [D-004](decisions.md#d-004--hypothèse-sur-le-robot-du-laboratoire)) |
| Q2 | Quelle version de ROS 2 tourne sur le robot et sur les PC du labo ? | Le PC de développement est en ROS 2 Lyrical ; des versions différentes peuvent empêcher la communication |
| Q3 | Quelles sont les dates de rendu du rapport, de la démonstration et des points d'étape ? | Calibrer la feuille de route |
| Q4 | Format attendu du rapport (longueur, langue, modèle) ? | Préparer la rédaction au fil de l'eau |
| Q5 | Quel scénario de démonstration est attendu (cible, marqueurs, parcours, obstacles) ? | Orienter les étapes E3 et E4 |
| Q6 | L'usage de Nav2 est-il attendu, ou faut-il écrire nos propres contrôleurs ? | Choix d'architecture pour E3 et E4 |
| Q7 | Quand et comment accéder au robot et à la salle de démonstration ? | Planifier E5 assez tôt |
