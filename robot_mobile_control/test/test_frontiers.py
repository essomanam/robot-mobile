"""Tests de la détection des frontières (frontiers.py).

Chaque test construit une petite grille dont on connaît la bonne réponse, dessinée dans le code :
    U = inconnue (-1)     F = libre (0)     X = occupée (100)
ligne 0 en haut, colonne 0 à gauche ; une case repérée par (ligne, colonne).

    python3 -m pytest test/test_frontiers.py -v      (depuis le dossier robot_mobile_control)
"""

import numpy as np
import pytest
from robot_mobile_control.frontiers import (choose_frontier, find_frontiers, FREE, Frontier,
                                            frontier_mask, UNKNOWN)

U, F, X = UNKNOWN, FREE, 100


def cells(mask):
    """Ensemble des (ligne, colonne) où le masque est vrai."""
    return {(int(r), int(c)) for r, c in np.argwhere(mask)}


def cells_of(frontier):
    """Ensemble des (ligne, colonne) d'une zone de frontière."""
    return {(int(r), int(c)) for r, c in frontier.cells}


def zone(size, goal):
    """Zone de frontière fabriquée à la main (seules la taille et le but comptent ici)."""
    return Frontier(cells=np.zeros((size, 2), dtype=int), goal=goal)


# --------------------------------------------------------------- frontier_mask

def test_mask_free_cells_next_to_unknown():
    grid = np.array([[F, F, U, U],
                     [F, F, U, U],
                     [F, F, U, U]])
    mask = frontier_mask(grid)
    assert mask.shape == grid.shape and mask.dtype == bool
    assert cells(mask) == {(0, 1), (1, 1), (2, 1)}   # la dernière colonne libre


def test_mask_four_directions_but_not_diagonals():
    grid = np.array([[F, F, F],
                     [F, U, F],
                     [F, F, F]])
    # les 4 voisines directes de l'inconnue, pas les coins
    assert cells(frontier_mask(grid)) == {(0, 1), (1, 0), (1, 2), (2, 1)}


def test_mask_occupied_and_unknown_cells_are_never_frontiers():
    grid = np.array([[F, X, U],
                     [F, X, U],
                     [F, F, F]])
    # seule (2, 2) est libre ET touche une inconnue (celle du dessus) ;
    # le mur (colonne 1) sépare la zone libre de l'inconnu
    assert cells(frontier_mask(grid)) == {(2, 2)}


def test_mask_grid_border_is_not_unknown():
    grid = np.array([[F, F, F, F],
                     [F, X, X, F],
                     [F, F, F, F]])
    assert not frontier_mask(grid).any()   # carte entièrement connue : aucune frontière


# --------------------------------------------------------------- find_frontiers

def test_find_two_separate_zones_and_filter_small_ones():
    grid = np.full((8, 6), F)
    grid[:, 5] = U          # colonne de droite inconnue
    grid[3, 4:] = X         # un mur coupe la frontière (colonne 4) en deux
    zones = find_frontiers(grid, min_size=1)
    assert sorted(z.size for z in zones) == [3, 4]   # lignes 0-2 et lignes 4-7
    big = find_frontiers(grid, min_size=4)
    assert [z.size for z in big] == [4]


def test_find_cells_touching_diagonally_form_one_zone():
    grid = np.array([[F, F, U, U],
                     [F, F, F, U],
                     [F, F, F, F],
                     [F, F, F, F]])
    # frontières : (0, 1), (1, 2), (2, 3), un escalier qui ne se touche qu'en diagonale
    assert cells(frontier_mask(grid)) == {(0, 1), (1, 2), (2, 3)}
    zones = find_frontiers(grid, min_size=1)
    assert len(zones) == 1 and zones[0].size == 3     # 8-connexité : une seule zone


def test_find_goal_is_the_zone_cell_closest_to_its_centre():
    grid = np.full((6, 5), F)
    grid[0, :] = U          # ligne du haut inconnue
    grid[:, 4] = U          # colonne de droite inconnue
    # frontière en L : ligne 1 (colonnes 0 à 3) et colonne 3 (lignes 2 à 5), 8 cases ;
    # centre (2,25 ; 2,25), qui tombe hors de la frontière ; case la plus proche : (2, 3)
    zones = find_frontiers(grid, min_size=1)
    assert len(zones) == 1
    assert zones[0].size == 8
    assert cells_of(zones[0]) == {(1, 0), (1, 1), (1, 2), (1, 3), (2, 3), (3, 3), (4, 3), (5, 3)}
    assert tuple(int(v) for v in zones[0].goal) == (2, 3)


def test_find_nothing_in_a_fully_known_map():
    grid = np.array([[X, X, X],
                     [X, F, X],
                     [X, X, X]])
    assert find_frontiers(grid, min_size=1) == []


# --------------------------------------------------------------- choose_frontier

def test_choose_balances_size_and_distance():
    robot, res = (0, 0), 0.05
    near_small = zone(10, (0, 20))    # 1 m  -> score 10
    far_big = zone(30, (0, 100))      # 5 m  -> score 6
    assert choose_frontier([far_big, near_small], robot, res) is near_small
    far_huge = zone(60, (0, 100))     # 5 m  -> score 12
    assert choose_frontier([near_small, far_huge], robot, res) is far_huge


def test_choose_uses_min_distance_for_very_close_zones():
    robot, res = (0, 0), 0.05
    on_robot = zone(5, (0, 0))        # 0 m   -> ramené à 0,5 m -> score 10
    close = zone(8, (0, 8))           # 0,4 m -> ramené à 0,5 m -> score 16
    assert choose_frontier([on_robot, close], robot, res) is close


def test_choose_returns_none_without_frontier():
    assert choose_frontier([], (0, 0), 0.05) is None


@pytest.mark.parametrize('rows, cols', [(1, 1), (1, 5), (5, 1)])
def test_mask_tiny_grids_do_not_crash(rows, cols):
    grid = np.full((rows, cols), F)
    assert not frontier_mask(grid).any()
