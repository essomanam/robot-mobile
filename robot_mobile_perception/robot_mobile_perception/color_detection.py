"""Détection d'une cible colorée et mesure de sa position 3D (sans ROS, testable seul).

Chaîne de traitement d'une image couleur + profondeur :
  1. conversion BGR -> HSV et seuillage sur la teinte (le rouge occupe les deux extrémités
     de l'échelle de teinte d'OpenCV, 0-179), la saturation et la valeur ;
  2. nettoyage morphologique du masque (ouverture : retire les petits points isolés ;
     fermeture : bouche les petits trous) ;
  3. plus grande composante connexe, si son aire dépasse un seuil ;
  4. distance : médiane de la profondeur sur les pixels de la composante (robuste aux trous
     et aux bords), puis rétroprojection du centre par le modèle sténopé :
         X = (u - cx) Z / fx    Y = (v - cy) Z / fy    (repère optique de la caméra).
"""

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class Blob:
    """Zone détectée dans l'image (pixels)."""

    area: int
    u: float          # centre, colonne
    v: float          # centre, ligne
    x: int            # rectangle englobant
    y: int
    width: int
    height: int
    mask: np.ndarray  # pixels de la zone (booléens)


def color_mask(bgr, hue_ranges, s_min, v_min, kernel_size=5):
    """Masque binaire (0/255) des pixels dont la teinte est dans l'une des plages données.

    hue_ranges : liste à plat [h_min1, h_max1, h_min2, h_max2, ...] (échelle OpenCV 0-179).
    """
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], np.uint8)
    for h_min, h_max in zip(hue_ranges[::2], hue_ranges[1::2]):
        mask |= cv2.inRange(hsv, (int(h_min), int(s_min), int(v_min)), (int(h_max), 255, 255))
    if kernel_size > 1:
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def largest_blob(mask, min_area):
    """Plus grande composante connexe du masque, ou None si aucune n'atteint min_area pixels."""
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if count <= 1:
        return None
    i = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    area = int(stats[i, cv2.CC_STAT_AREA])
    if area < min_area:
        return None
    return Blob(area=area, u=float(centroids[i][0]), v=float(centroids[i][1]),
                x=int(stats[i, cv2.CC_STAT_LEFT]), y=int(stats[i, cv2.CC_STAT_TOP]),
                width=int(stats[i, cv2.CC_STAT_WIDTH]), height=int(stats[i, cv2.CC_STAT_HEIGHT]),
                mask=labels == i)


def depth_in_meters(depth):
    """Image de profondeur en mètres (float32) : 32FC1 (m) ou 16UC1 (mm, caméras réelles)."""
    if depth.dtype == np.uint16:
        out = depth.astype(np.float32) / 1000.0
        out[depth == 0] = np.nan  # 0 = pas de mesure
        return out
    return depth.astype(np.float32)


def blob_position(blob, depth_m, fx, fy, cx, cy, max_depth):
    """Position 3D (X, Y, Z) du centre de la zone dans le repère optique, ou None.

    Z est la médiane des profondeurs valides de la zone : c'est la distance à la face visible.
    """
    z = depth_m[blob.mask]
    z = z[np.isfinite(z) & (z > 0) & (z < max_depth)]
    if z.size == 0:
        return None
    z_med = float(np.median(z))
    return ((blob.u - cx) * z_med / fx, (blob.v - cy) * z_med / fy, z_med)
