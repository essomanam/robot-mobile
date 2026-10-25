# Journal — 2026-10-25 — E1 : contrôle visuel et installation des outils SLAM

| Champ | Valeur |
|---|---|
| Durée | courte séance |
| Étape(s) concernée(s) | E1 — [fiche](../etapes/E1_simulation.md) ; préparation de E2 |

## Objectif de la séance

Vérifier à l'écran la simulation (Gazebo, RViz), piloter le robot au clavier, installer les
paquets nécessaires à E2.

## Ce qui a été fait

- Simulation lancée avec interfaces : salle et objets conformes au plan ; nuage de profondeur et
  scan LiDAR superposés dans RViz → critère de contrôle visuel de E1 validé ([§ 8.5](../etapes/E1_simulation.md#85-contrôle-visuel-2026-10-25)).
- Installation de `ros-lyrical-rtabmap-ros` (0.23.13) et `ros-lyrical-slam-toolbox` (2.10.0) →
  [environnement](../01_environnement_logiciel.md#4-paquets-ros--installés-et-à-installer).

## Problèmes rencontrés et solutions

| Problème | Cause identifiée | Solution / contournement |
|---|---|---|
| Le robot ne bouge pas avec `teleop_twist_keyboard` | Le robot n'a reçu aucune commande (odométrie à 0) alors que la chaîne fonctionne : une commande envoyée avec `ros2 topic pub` le fait avancer de 0,22 m. Le nœud lit les touches dans son terminal, qui doit être la fenêtre active | Taper les touches dans le terminal de téléopération ; vitesses limitées à 0,2 m/s et 0,8 rad/s (commande mise à jour dans le README) |
| Panneau « Image profondeur » noir dans RViz | Pixels à +∞ (au-delà de 8 m) + normalisation automatique de RViz : l'échelle devient infinie et tout est noir (vérifié dans le code de `rviz_default_plugins`) | Désactiver « Normalize Range » et fixer « Max Value » à 8 dans le panneau Displays ; vérifier pourquoi la configuration ne l'applique pas |
| Facteur temps réel 59 % avec les interfaces | Rendu de l'interface Gazebo et de RViz en plus des capteurs | Sans effet sur les résultats (temps simulé) ; préférer `gui:=false` pour les enregistrements |

## Décisions prises

- Aucune nouvelle décision.

## Questions ouvertes

- Pour moi : confirmer le pilotage au clavier ; état du réglage « Normalize Range » dans RViz.
- Pour le superviseur : Q1 et Q2 toujours en attente.

## Prochaine séance

- Commiter E1.
- Démarrer E2 : RTAB-Map sur le rosbag de référence, comparaison avec l'odométrie seule et slam_toolbox.
