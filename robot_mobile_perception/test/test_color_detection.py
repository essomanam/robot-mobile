"""Tests de la détection sur des images synthétiques dont on connaît la réponse."""

import numpy as np
import pytest
from robot_mobile_perception.color_detection import (blob_position, color_mask,
                                                     depth_in_meters, largest_blob)

RED_HUES = [0, 10, 170, 179]


def scene(red_box=(200, 150, 80, 60), background=(200, 200, 200)):
    """Image BGR 640 x 480 avec un rectangle rouge (x, y, largeur, hauteur)."""
    img = np.full((480, 640, 3), background, np.uint8)
    x, y, w, h = red_box
    img[y:y + h, x:x + w] = (0, 0, 200)  # rouge en BGR
    return img


def test_finds_red_rectangle_center():
    blob = largest_blob(color_mask(scene(), RED_HUES, 100, 60), min_area=100)
    assert blob is not None
    assert blob.u == pytest.approx(200 + 79 / 2, abs=1.0)
    assert blob.v == pytest.approx(150 + 59 / 2, abs=1.0)
    assert blob.area == pytest.approx(80 * 60, rel=0.05)


def test_ignores_non_red_and_small_blobs():
    img = scene(red_box=(10, 10, 5, 5))           # trop petit
    img[300:360, 300:380] = (0, 200, 0)           # vert : pas une cible
    assert largest_blob(color_mask(img, RED_HUES, 100, 60), min_area=100) is None


def test_pale_or_dark_red_rejected():
    pale = scene()
    pale[150:210, 200:280] = (170, 170, 230)      # rose pâle : saturation trop faible
    assert largest_blob(color_mask(pale, RED_HUES, 100, 60), min_area=100) is None


def test_back_projection_with_known_depth():
    img = scene(red_box=(400, 220, 40, 40))       # centre (419.5, 239.5)
    blob = largest_blob(color_mask(img, RED_HUES, 100, 60), min_area=100)
    depth = np.full((480, 640), 2.0, np.float32)  # tout à 2 m
    x, y, z = blob_position(blob, depth, fx=462.2, fy=462.2, cx=320.0, cy=240.0, max_depth=8.0)
    assert z == pytest.approx(2.0)
    assert x == pytest.approx((419.5 - 320.0) * 2.0 / 462.2, abs=0.01)
    assert y == pytest.approx(0.0, abs=0.01)


def test_depth_median_ignores_invalid_pixels():
    img = scene(red_box=(300, 200, 40, 40))
    blob = largest_blob(color_mask(img, RED_HUES, 100, 60), min_area=100)
    depth = np.full((480, 640), 1.5, np.float32)
    depth[200:210, 300:340] = np.inf              # un quart de la zone sans mesure
    assert blob_position(blob, depth, 462.2, 462.2, 320.0, 240.0, 8.0)[2] == pytest.approx(1.5)


def test_depth_16_bits_in_millimeters():
    raw = np.array([[0, 1500]], np.uint16)
    out = depth_in_meters(raw)
    assert np.isnan(out[0, 0]) and out[0, 1] == pytest.approx(1.5)
