# Documentation du projet

Cette documentation décrit **ce qui est fait, comment et pourquoi**, étape par étape. Elle est écrite
pour être lisible par quelqu'un qui ne connaît pas la robotique, et pour servir de matière première
au rapport technique et aux présentations.

## Par où commencer ?

| Vous voulez… | Lisez |
|---|---|
| Comprendre le projet en 5 minutes | [00 — Analyse du sujet](00_analyse_du_sujet.md), sections 1 et 2 |
| Savoir où en est le projet | [04 — Feuille de route](04_feuille_de_route.md), tableau de la section 1 |
| Comprendre le vocabulaire | [Glossaire](glossaire.md) et les [notions](#notions-de-base) |
| Comprendre comment le logiciel est organisé | [03 — Architecture cible](03_architecture_cible.md) |
| Savoir pourquoi tel choix a été fait | [Registre des décisions](decisions.md) |
| Refaire une expérience | La section *Reproduire* de la fiche d'étape concernée, dans [`etapes/`](etapes/) |

## Contenu

### Documents de cadrage

| Document | Contenu |
|---|---|
| [00_analyse_du_sujet.md](00_analyse_du_sujet.md) | Sujet reformulé, exigences numérotées, traçabilité, questions pour le superviseur |
| [01_environnement_logiciel.md](01_environnement_logiciel.md) | Système, versions exactes, paquets, configuration, points de vigilance |
| [02_etat_des_lieux.md](02_etat_des_lieux.md) | Prise en main et analyse critique des prototypes |
| [03_architecture_cible.md](03_architecture_cible.md) | Nœuds, topics, repères TF, organisation des paquets |
| [04_feuille_de_route.md](04_feuille_de_route.md) | Étapes, livrables, critères, jalons, risques |

### Suivi du travail

| Document | Contenu |
|---|---|
| [etapes/](etapes/) | Une fiche par étape : objectif, méthode, résultats, analyse |
| [journal/](journal/) | Journal de bord daté, une entrée par séance de travail |
| [decisions.md](decisions.md) | Chaque choix important, ses options et sa justification |

### Notions de base

| Document | Contenu |
|---|---|
| [notions/ros2.md](notions/ros2.md) | Nœuds, topics, messages, paramètres, launch, TF, QoS |
| [notions/capteurs.md](notions/capteurs.md) | LiDAR, caméra, RGB-D, IMU, odométrie : principe et limites |
| [notions/slam_vslam.md](notions/slam_vslam.md) | SLAM, V-SLAM, RTAB-Map, ORB-SLAM, évaluation |
| [notions/navigation_controle.md](notions/navigation_controle.md) | Boucle de commande, asservissement visuel, Nav2 |

### Références et outils

| Document | Contenu |
|---|---|
| [glossaire.md](glossaire.md) | Définitions simples de tous les termes techniques |
| [references.md](references.md) | Bibliographie |
| [modeles/](modeles/) | Modèles de fiche d'étape et d'entrée de journal |
| [figures/](figures/) | Images et schémas utilisés dans la documentation |

## Conventions

- **Langue** : documentation et commentaires en français ; noms de fichiers, de variables, de
  nœuds et de topics en anglais (usage ROS).
- **Chaque document commence par un résumé en 3 lignes.**
- **Statuts** : ⚪ à faire · 🟡 en cours · 🟢 terminé · 🔴 bloqué.
- **Identifiants** : exigences `EX-…` / `LIV-…`, étapes `E0` à `E6`, décisions `D-xxx`,
  risques `R…`, questions au superviseur `Q…`, références `[NOM_ANNÉE]`.
- **Journal** : un fichier par séance, nommé `AAAA-MM-JJ_sujet.md`, à partir du [modèle](modeles/modele_journal.md).
- **Figures** : dans `figures/`, préfixées par l'étape (`E2_trajectoire_ate.png`), toujours
  citées avec une légende qui dit ce qu'on doit y voir.
- **Schémas** : en Mermaid quand c'est possible (texte versionné, affiché par GitHub et VS Code).
- **Données volumineuses** (rosbags, bases RTAB-Map) : hors Git ; leur emplacement et leur
  contenu sont indiqués dans la fiche d'étape.
- **Rien n'est effacé** : une décision qui change est *remplacée* par une nouvelle ; un résultat
  négatif est noté et expliqué.
