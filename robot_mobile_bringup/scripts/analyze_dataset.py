#!/usr/bin/env python3
"""Analyse un enregistrement (rosbag) du jeu de données de référence.

Produit :
  1. la fréquence réelle de chaque topic, en temps simulé ;
  2. la comparaison odométrie des roues / vérité terrain : trajectoires superposées
     sur le plan de la salle et erreur de position au cours du temps (la dérive).

Usage :
    ros2 run robot_mobile_bringup analyze_dataset.py <dossier_rosbag> <dossier_figures>
"""

import math
from pathlib import Path
import sys

import matplotlib
from matplotlib.patches import Circle, Rectangle
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
import numpy as np
from rclpy.serialization import deserialize_message
import rosbag2_py

matplotlib.use('Agg')  # figures écrites dans des fichiers, sans fenêtre

# Plan de la salle (doit rester cohérent avec worlds/as52_room.sdf et models/as52_room)
ROOM = (-3.5, 3.5, -2.5, 2.5)
OBSTACLES = [  # (nom, x, y, demi-largeur x, demi-largeur y)
    ('pilier A', -1.6, 0.3, 0.15, 0.15),
    ('pilier B', 0.5, -0.6, 0.15, 0.15),
    ('cible rouge', 1.7, -0.3, 0.2, 0.2),
    ('table', -0.4, -0.2, 0.4565, 0.4565),  # plateau carré de 0,913 m
]
DISCS = [('poubelle', 0.8, 0.5, 0.2)]  # (nom, x, y, rayon approximatif)


def yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def read_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    stamps, odom, truth = {}, [], []
    while reader.has_next():
        # t : instant de réception, en temps simulé (enregistrement en --use-sim-time)
        topic, data, t, _ = reader.read_next_ext()
        stamps.setdefault(topic, []).append(t * 1e-9)
        if topic in ('/odom', '/ground_truth/odom'):
            msg = deserialize_message(data, Odometry)
            p = msg.pose.pose
            row = (t * 1e-9, p.position.x, p.position.y, yaw(p.orientation))
            (odom if topic == '/odom' else truth).append(row)
    return stamps, np.array(odom), np.array(truth)


def rates_table(stamps):
    print(f'{"topic":32s} {"messages":>9s} {"fréquence (Hz, temps simulé)":>30s}')
    for topic in sorted(stamps):
        t = np.array(stamps[topic])
        rate = (len(t) - 1) / (t[-1] - t[0]) if len(t) > 1 and t[-1] > t[0] else float('nan')
        print(f'{topic:32s} {len(t):9d} {rate:30.2f}')


def odom_in_world(odom, truth):
    """Exprime l'odométrie (repère odom, part de 0) dans le repère du monde.

    Le repère odom coïncide avec la pose du robot au démarrage de la simulation :
    on lui applique la pose vraie à cet instant.
    """
    t0 = odom[0, 0]
    i = np.argmin(np.abs(truth[:, 0] - t0))
    x0, y0, th0 = truth[i, 1:]
    dx, dy, dth = odom[0, 1:]  # pose odométrique à t0 (≈ 0)
    th_off = th0 - dth
    c, s = math.cos(th_off), math.sin(th_off)
    x = x0 + c * (odom[:, 1] - dx) - s * (odom[:, 2] - dy)
    y = y0 + s * (odom[:, 1] - dx) + c * (odom[:, 2] - dy)
    th = odom[:, 3] + th_off
    return np.column_stack([odom[:, 0], x, y, th])


def draw_room(ax):
    x0, x1, y0, y1 = ROOM
    ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, lw=3, color='0.3'))
    for name, x, y, hx, hy in OBSTACLES:
        color = 'red' if 'cible' in name else '0.5'
        ax.add_patch(Rectangle((x - hx, y - hy), 2 * hx, 2 * hy, color=color, alpha=0.7))
        ax.annotate(name, (x, y + hy + 0.08), ha='center', fontsize=8)
    for name, x, y, r in DISCS:
        ax.add_patch(Circle((x, y), r, color='0.7', alpha=0.7))
        ax.annotate(name, (x, y), ha='center', va='center', fontsize=8)


def main():
    bag, out = Path(sys.argv[1]), Path(sys.argv[2])
    out.mkdir(parents=True, exist_ok=True)
    stamps, odom, truth = read_bag(bag)
    rates_table(stamps)

    odom_w = odom_in_world(odom, truth)
    # erreur de position de l'odométrie, aux instants de la vérité terrain
    ox = np.interp(truth[:, 0], odom_w[:, 0], odom_w[:, 1])
    oy = np.interp(truth[:, 0], odom_w[:, 0], odom_w[:, 2])
    err = np.hypot(ox - truth[:, 1], oy - truth[:, 2])
    oth = np.interp(truth[:, 0], odom_w[:, 0], np.unwrap(odom_w[:, 3]))
    heading_err = np.degrees(np.angle(np.exp(1j * (oth - truth[:, 3]))))
    dist = np.concatenate([[0], np.cumsum(np.hypot(np.diff(truth[:, 1]), np.diff(truth[:, 2])))])
    end_gap = math.hypot(truth[-1, 1] - truth[0, 1], truth[-1, 2] - truth[0, 2])
    print(f'\nDistance parcourue (vérité terrain) : {dist[-1]:.2f} m')
    print(f'Écart arrivée / départ (vérité terrain) : {end_gap:.3f} m')
    print(f"Erreur de position de l'odométrie : finale {err[-1]:.4f} m, "
          f'max {err.max():.4f} m, soit {100 * err[-1] / dist[-1]:.3f} % de la distance')
    print(f"Erreur de cap de l'odométrie : finale {heading_err[-1]:+.2f}°, "
          f'max {np.abs(heading_err).max():.2f}°')

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 4.8),
                                   gridspec_kw={'width_ratios': [1.4, 1]})
    draw_room(ax1)
    ax1.plot(truth[:, 1], truth[:, 2], color='tab:blue', lw=2.5, label='vérité terrain')
    ax1.plot(odom_w[:, 1], odom_w[:, 2], '--', color='tab:orange', lw=1.5,
             label='odométrie des roues')
    ax1.plot(*truth[0, 1:3], 'o', color='black', label='départ')
    ax1.set_aspect('equal')
    ax1.set_xlim(ROOM[0] - 0.3, ROOM[1] + 0.3)
    ax1.set_ylim(ROOM[2] - 0.3, ROOM[3] + 0.3)
    ax1.set_xlabel('x (m)')
    ax1.set_ylabel('y (m)')
    ax1.set_title('Trajectoire de référence dans la salle as52_room')
    ax1.legend(loc='upper right', fontsize=8)
    ax1.grid(alpha=0.3)

    ax2.plot(dist, 1000 * err, color='tab:orange')
    ax2.set_xlabel('distance parcourue (m)')
    ax2.set_ylabel('erreur de position (mm)')
    ax2.set_title("Dérive de l'odométrie des roues")
    ax2.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out / 'E1_trajectoire_reference.png', dpi=110)
    print(f'Figure : {out / "E1_trajectoire_reference.png"}')


if __name__ == '__main__':
    main()
