#!/usr/bin/env python3
"""Génère les textures des murs, piliers et sol de la salle simulée as52_room.

Pourquoi : le SLAM visuel suit des points d'intérêt (coins, contrastes). Des murs
unis n'en offrent aucun. Chaque surface reçoit donc un motif riche, différent des
autres (pour éviter de confondre deux endroits) et sans rouge saturé (pour ne pas
perturber la détection de la cible rouge).

Reproductible : graine fixe par surface. À relancer seulement si l'on modifie le
décor ; les images produites sont versionnées.

Usage : python3 generate_textures.py <dossier_de_sortie>
"""

from pathlib import Path
import sys

import cv2
import numpy as np

PX_PER_M = 400

# (nom, largeur m, hauteur m, graine)
SURFACES = [
    ('wall_north', 7.2, 1.5, 11),
    ('wall_south', 7.2, 1.5, 22),
    ('wall_east', 5.0, 1.5, 33),
    ('wall_west', 5.0, 1.5, 44),
    ('pillar_a', 0.3, 1.5, 55),
    ('pillar_b', 0.3, 1.5, 66),
]
FLOOR = ('floor', 7.0, 5.0, 77, 200)  # sol : résolution plus faible (200 px/m)

# Teintes interdites pour les couleurs saturées (échelle OpenCV 0-179) :
# le détecteur de cible garde H <= 10 ou H >= 170 ; on prend une marge.
FORBIDDEN_HUE = (20, 160)


def random_color(rng, saturated=True):
    """Couleur BGR aléatoire ; les couleurs saturées évitent les teintes rouges."""
    if saturated and rng.random() < 0.75:
        hue = int(rng.integers(FORBIDDEN_HUE[0], FORBIDDEN_HUE[1]))
        sat = int(rng.integers(90, 256))
    else:
        hue = int(rng.integers(0, 180))
        sat = int(rng.integers(0, 60))
    val = int(rng.integers(40, 256))
    hsv = np.uint8([[[hue, sat, val]]])
    return tuple(int(c) for c in cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0, 0])


def draw_shapes(img, rng, count, max_size):
    h, w = img.shape[:2]
    for _ in range(count):
        color = random_color(rng)
        cx, cy = int(rng.integers(0, w)), int(rng.integers(0, h))
        size = int(rng.integers(max_size // 6, max_size))
        kind = rng.integers(0, 6)
        if kind == 0:
            cv2.circle(img, (cx, cy), size // 2, color, -1, cv2.LINE_AA)
        elif kind == 1:
            cv2.rectangle(img, (cx, cy), (cx + size, cy + size // 2), color, -1)
        elif kind == 2:
            pts = rng.integers(-size, size, (3, 2)) + [cx, cy]
            cv2.fillPoly(img, [pts.astype(np.int32)], color, cv2.LINE_AA)
        elif kind == 3:
            cv2.circle(img, (cx, cy), size // 2, color, max(3, size // 10), cv2.LINE_AA)
        elif kind == 4:
            end = (cx + int(rng.integers(-size, size)), cy + int(rng.integers(-size, size)))
            cv2.line(img, (cx, cy), end, color, max(3, size // 12), cv2.LINE_AA)
        else:
            # damier : coins très contrastés, faciles à suivre
            cell = max(8, size // 6)
            for i in range(4):
                for j in range(4):
                    c = (20, 20, 20) if (i + j) % 2 else (235, 235, 235)
                    cv2.rectangle(img, (cx + i * cell, cy + j * cell),
                                  (cx + (i + 1) * cell, cy + (j + 1) * cell), c, -1)


def make_texture(width_m, height_m, seed, px_per_m, density, max_size_m):
    rng = np.random.default_rng(seed)
    w, h = int(width_m * px_per_m), int(height_m * px_per_m)
    img = np.full((h, w, 3), random_color(rng, saturated=False), np.uint8)
    img[:] = np.clip(img.astype(int) + 120, 0, 255).astype(np.uint8)  # fond clair
    count = int(density * width_m * height_m)
    draw_shapes(img, rng, count, int(max_size_m * px_per_m))
    return img


def main():
    out = Path(sys.argv[1] if len(sys.argv) > 1 else '.')
    out.mkdir(parents=True, exist_ok=True)
    for name, w, h, seed in SURFACES:
        img = make_texture(w, h, seed, PX_PER_M, density=40, max_size_m=0.35)
        cv2.imwrite(str(out / f'{name}.png'), img)
        print(f'{name}.png  {img.shape[1]}x{img.shape[0]}')
    name, w, h, seed, px = FLOOR
    img = make_texture(w, h, seed, px, density=12, max_size_m=0.25)
    img = cv2.addWeighted(img, 0.5, np.full_like(img, 128), 0.5, 0)  # sol plus terne
    cv2.imwrite(str(out / f'{name}.png'), img)
    print(f'{name}.png  {img.shape[1]}x{img.shape[0]}')


if __name__ == '__main__':
    main()
