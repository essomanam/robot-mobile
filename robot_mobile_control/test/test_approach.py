"""Tests de la machine à états et des lois de commande de l'approche de cible."""

import math

import pytest
from robot_mobile_control.approach import (ALIGN, APPROACH, ApproachController,
                                           ApproachParams, ARRIVED, SEARCH)


def test_search_rotates_when_target_never_seen():
    c = ApproachController()
    v, w = c.update(0.0, None)
    assert c.state == SEARCH and v == 0.0 and w > 0


def test_align_then_approach():
    c = ApproachController()
    v, w = c.update(0.0, (math.radians(40), 3.0))      # cible à 40° à gauche
    assert c.state == ALIGN and v == 0.0 and w > 0
    v, w = c.update(0.1, (math.radians(5), 3.0))       # presque centrée
    assert c.state == APPROACH and v == pytest.approx(0.2, rel=0.01) and w > 0


def test_speed_saturation_and_slowdown_near_target():
    p = ApproachParams()
    c = ApproachController(p)
    c.update(0.0, (0.0, 5.0))
    v_far, _ = c.update(0.1, (0.0, 5.0))
    v_near, _ = c.update(0.2, (0.0, p.desired_distance + 0.2))
    assert v_far == pytest.approx(p.max_v) and v_near == pytest.approx(p.k_v * 0.2)


def test_arrival_and_stop():
    p = ApproachParams()
    c = ApproachController(p)
    c.update(0.0, (0.0, 2.0))
    v, w = c.update(0.1, (0.01, p.desired_distance + 0.01))
    assert c.state == ARRIVED and v == 0.0 and w == 0.0


def test_obstacle_ahead_blocks_forward_motion_only():
    c = ApproachController()
    c.update(0.0, (0.0, 3.0))
    v, w = c.update(0.1, (math.radians(10), 3.0), obstacle_ahead=True)
    assert v == 0.0 and w != 0.0


def test_lost_target_returns_to_search_on_last_seen_side():
    p = ApproachParams()
    c = ApproachController(p)
    c.update(0.0, (math.radians(-10), 3.0))            # vue à droite
    v, w = c.update(0.0 + p.lost_timeout + 0.1, None)
    assert c.state == SEARCH and w < 0                 # on la cherche à droite
