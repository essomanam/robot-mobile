"""Tests des métriques : chaque cas a une réponse connue à l'avance.

Si ces tests échouent, toutes les comparaisons de SLAM sont fausses : ils valident
l'outil de mesure avant de s'en servir.
"""

import math

import numpy as np
import pytest

from robot_mobile_evaluation.room import (clearance, distance_to_box, ray_to_wall,
                                          sight_distance_to_box)
from robot_mobile_evaluation.trajectory import (align_first_pose, align_umeyama, ate_rmse,
                                                read_tum, rpe, transform, write_tum)


def rectangle_trajectory(n_side=50):
    """Tour de 5 x 3 m, comme la trajectoire de référence (t, x, y, cap)."""
    corners = [(0, 0), (5, 0), (5, 3), (0, 3), (0, 0)]
    rows, t = [], 0.0
    for (x0, y0), (x1, y1) in zip(corners[:-1], corners[1:]):
        yaw = math.atan2(y1 - y0, x1 - x0)
        for k in range(n_side):
            s = k / n_side
            rows.append((t, x0 + s * (x1 - x0), y0 + s * (y1 - y0), yaw))
            t += 0.1
    return np.array(rows)


def test_ray_to_wall():
    assert ray_to_wall(0.0, 0.0, 0.0) == pytest.approx(3.5)
    assert ray_to_wall(0.0, 0.0, math.pi / 2) == pytest.approx(2.5)
    assert ray_to_wall(-2.5, -1.5, math.pi) == pytest.approx(1.0)
    # diagonale : le mur y = 2,5 est atteint avant le mur x = 3,5
    assert ray_to_wall(0.0, 0.0, math.pi / 4) == pytest.approx(2.5 * math.sqrt(2))


def test_umeyama_recovers_rigid_transform():
    truth = rectangle_trajectory()
    # estimation = vérité exprimée dans un autre repère (rotation 30°, translation)
    est = transform(truth, -0.5236, 1.0, -2.0)
    theta, tx, ty = align_umeyama(est, truth)
    aligned = transform(est, theta, tx, ty)
    assert np.allclose(aligned[:, 1:3], truth[:, 1:3], atol=1e-9)
    ate, _ = ate_rmse(est, truth)
    assert ate == pytest.approx(0.0, abs=1e-9)


def test_ate_of_known_offset():
    truth = rectangle_trajectory()
    est = truth.copy()
    # moitié des points décalés de +0,1 m, l'autre moitié de -0,1 m en y : l'alignement
    # ne peut pas compenser, l'erreur RMS vaut exactement 0,1 m
    est[::2, 2] += 0.1
    est[1::2, 2] -= 0.1
    ate, _ = ate_rmse(est, truth)
    assert ate == pytest.approx(0.1, rel=0.02)


def test_first_pose_alignment():
    truth = rectangle_trajectory()
    est = transform(truth, 1.0, 3.0, 4.0)
    aligned = transform(est, *align_first_pose(est, truth))
    assert np.allclose(aligned[:, 1:3], truth[:, 1:3], atol=1e-9)


def test_rpe_zero_and_heading_drift():
    truth = rectangle_trajectory()
    assert rpe(truth, truth) == pytest.approx((0.0, 0.0), abs=1e-12)
    # cap qui dérive de 1° par mètre parcouru : RPE en rotation ≈ 1° sur 1 m
    dist = np.concatenate([[0], np.cumsum(np.hypot(np.diff(truth[:, 1]), np.diff(truth[:, 2])))])
    est = truth.copy()
    est[:, 3] += np.radians(dist)
    _, rot = rpe(est, truth, delta=1.0)
    assert rot == pytest.approx(1.0, rel=0.1)


def test_tum_roundtrip(tmp_path):
    truth = rectangle_trajectory()
    write_tum(tmp_path / 'traj.tum', truth)
    back = read_tum(tmp_path / 'traj.tum')
    assert np.allclose(back[:, :3], truth[:, :3], atol=1e-6)
    assert np.allclose(np.cos(back[:, 3] - truth[:, 3]), 1.0, atol=1e-9)


def test_distance_to_box_and_clearance():
    assert distance_to_box(0.0, 0.0, 2.0, 0.0, 0.5, 0.5) == pytest.approx(1.5)
    assert distance_to_box(2.1, 0.1, 2.0, 0.0, 0.5, 0.5) == 0.0           # à l'intérieur
    assert distance_to_box(3.0, 1.0, 2.0, 0.0, 0.5, 0.5) == pytest.approx(math.hypot(0.5, 0.5))
    # au départ par défaut, l'obstacle le plus proche est le mur sud (1,0 m) ou ouest (1,0 m)
    assert clearance(-2.5, -1.5) == pytest.approx(1.0)
    # devant la cible rouge (exclue par défaut), l'obstacle le plus proche est le pilier B,
    # centré en (0,5 ; -0,6), demi-côté 0,15 : écarts 0,35 m en x et 0,15 m en y
    assert clearance(1.0, -0.3) == pytest.approx(math.hypot(0.35, 0.15))


def test_sight_distance_to_box():
    # en face : identique à la distance au point le plus proche
    assert sight_distance_to_box(0.0, 0.0, 2.0, 0.0, 0.2, 0.2) == pytest.approx(1.8)
    # en biais à 45° vers le centre d'un carré : on entre par le coin, à d * (1 - h / dx)
    d = math.hypot(2.0, 2.0)
    expected = d * (1 - 0.2 / 2.0)
    assert sight_distance_to_box(0.0, 0.0, 2.0, 2.0, 0.2, 0.2) == pytest.approx(expected)
    assert sight_distance_to_box(2.0, 0.1, 2.0, 0.0, 0.2, 0.2) == 0.0      # à l'intérieur
