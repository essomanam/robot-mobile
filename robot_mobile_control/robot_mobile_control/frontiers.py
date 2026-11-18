"""Détection des frontières d'exploration dans une grille d'occupation (sans ROS, testable seul).

Une FRONTIÈRE est la limite entre ce que le robot connaît déjà et ce qu'il n'a jamais vu :
une case LIBRE qui touche une case INCONNUE. Aller à une frontière, c'est aller voir du nouveau.
L'exploration répète : trouver les frontières → en choisir une → y aller → recommencer, jusqu'à
ce qu'il n'en reste plus (explorer.py).

La grille est celle du topic /map (nav_msgs/OccupancyGrid), mise sous forme d'un tableau NumPy
2D d'entiers, une case par élément :
    grid[ligne, colonne]   ligne = axe y de la carte, colonne = axe x
    -1 = inconnue      0 = libre      100 = occupée   (RTAB-Map n'utilise que ces trois valeurs)

À ÉCRIRE PAR L'ÉTUDIANT : les trois fonctions marquées « À écrire ». Les tests sont dans
test/test_frontiers.py ; lancer :
    colcon build --symlink-install --packages-select robot_mobile_control
    colcon test --packages-select robot_mobile_control && colcon test-result --verbose
ou, plus rapide, depuis le dossier robot_mobile_control :
    python3 -m pytest test/test_frontiers.py -v
"""

from dataclasses import dataclass

import cv2  # noqa: F401  (utile pour find_frontiers : cv2.connectedComponentsWithStats)
import numpy as np

UNKNOWN = -1
FREE = 0


@dataclass
class Frontier:
    """Une zone de frontière : un groupe de cases frontières qui se touchent."""

    cells: np.ndarray   # tableau (N, 2) d'entiers : une ligne par case, (ligne, colonne)
    goal: tuple         # (ligne, colonne) : la case de la zone la plus proche de son centre

    @property
    def size(self):
        """Nombre de cases de la zone."""
        return len(self.cells)


def frontier_mask(grid):
    """À écrire. Renvoie un tableau booléen de la forme de `grid` : True sur les cases frontières.

    Une case est une frontière si :
      - elle est LIBRE (valeur FREE) ;
      - et au moins une de ses 4 voisines (haut, bas, gauche, droite) est INCONNUE (UNKNOWN).
    Les voisines en diagonale ne comptent pas. Le bord de la grille n'est PAS de l'inconnu :
    une case libre au bord, sans voisine inconnue dans la grille, n'est pas une frontière.

    Indice : construire un tableau booléen `unknown = grid == UNKNOWN`, puis, pour chacune des
    4 directions, savoir si la voisine est inconnue en décalant ce tableau d'une case (tranches
    NumPy, par exemple unknown[1:, :] pour « la case du dessous »), en faisant attention aux bords.
    Une double boucle for sur les cases fonctionne aussi, mais elle est lente sur une grande carte.
    """
    raise NotImplementedError('frontier_mask : à écrire')


def find_frontiers(grid, min_size):
    """À écrire. Renvoie la liste des zones de frontière d'au moins `min_size` cases.

    Étapes :
      1. mask = frontier_mask(grid) ;
      2. regrouper les cases frontières qui se touchent, diagonales COMPRISES (8-connexité) :
         ce sont les « composantes connexes » du masque, exactement comme pour la cible rouge
         (voir largest_blob dans robot_mobile_perception/color_detection.py) ;
      3. ignorer les zones de moins de `min_size` cases (bruit, petits recoins) ;
      4. pour chaque zone gardée, construire un Frontier :
           cells = les (ligne, colonne) de ses cases (np.argwhere peut aider) ;
           goal  = la case de la zone la plus proche de son centre (moyenne des cases).
                   Pourquoi pas le centre lui-même ? Une frontière en forme de L ou d'arc a un
                   centre qui n'est pas sur la frontière, parfois dans un mur ou dans l'inconnu.
    L'ordre des zones dans la liste n'a pas d'importance.
    """
    raise NotImplementedError('find_frontiers : à écrire')


def choose_frontier(frontiers, robot_cell, resolution, min_distance=0.5):
    """À écrire. Choisit la zone à explorer en premier, ou renvoie None si la liste est vide.

    Score d'une zone (décision de l'étudiant : compromis taille / distance) :
        score = taille / max(distance, min_distance)
      - taille   : nombre de cases de la zone (frontier.size) ;
      - distance : distance en MÈTRES entre le robot et le but de la zone (frontier.goal) ;
                   robot_cell et goal sont en (ligne, colonne), une case mesure `resolution` m ;
      - min_distance (m) : évite de diviser par zéro, et ne favorise pas sans fin une zone
        collée au robot.
    On renvoie la zone de plus grand score.
    """
    raise NotImplementedError('choose_frontier : à écrire')
