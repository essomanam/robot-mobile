"""Analyse un enregistrement (rosbag) du jeu de données de référence.

Produit :
  1. la fréquence réelle de chaque topic, en temps simulé ;
  2. la comparaison odométrie des roues / vérité terrain : trajectoires superposées
     sur le plan de la salle et erreur de position au cours du temps (la dérive).

Usage :
    ros2 run robot_mobile_evaluation analyze_dataset <dossier_rosbag> <dossier_figures>
"""

from pathlib import Path
import sys

import matplotlib
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
import numpy as np
from rclpy.serialization import deserialize_message
from robot_mobile_evaluation.room import draw_room
from robot_mobile_evaluation.trajectory import (align_first_pose, associate, position_errors,
                                                transform, travelled_distance, wrap,
                                                yaw_from_quaternion)
import rosbag2_py

matplotlib.use('Agg')  # figures écrites dans des fichiers, sans fenêtre


def read_bag(path):
    """Instants de réception de chaque topic, odométrie et vérité terrain (t, x, y, cap)."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    stamps, odom, truth = {}, [], []
    while reader.has_next():
        # t : instant de réception, en temps simulé (enregistrement en --use-sim-time)
        topic, data, t, _ = reader.read_next_ext()
        stamps.setdefault(topic, []).append(t * 1e-9)
        if topic in ('/odom', '/ground_truth/odom'):
            p = deserialize_message(data, Odometry).pose.pose
            q = p.orientation
            row = (t * 1e-9, p.position.x, p.position.y,
                   yaw_from_quaternion(q.x, q.y, q.z, q.w))
            (odom if topic == '/odom' else truth).append(row)
    return stamps, np.array(odom), np.array(truth)


def rates_table(stamps):
    print(f'{"topic":32s} {"messages":>9s} {"fréquence (Hz, temps simulé)":>30s}')
    for topic in sorted(stamps):
        t = np.array(stamps[topic])
        rate = (len(t) - 1) / (t[-1] - t[0]) if len(t) > 1 and t[-1] > t[0] else float('nan')
        print(f'{topic:32s} {len(t):9d} {rate:30.2f}')


def main():
    bag, out = Path(sys.argv[1]), Path(sys.argv[2])
    out.mkdir(parents=True, exist_ok=True)
    stamps, odom, truth = read_bag(bag)
    rates_table(stamps)

    # Le repère odom part de la pose du robot au démarrage : on l'exprime dans le monde
    # en faisant coïncider la première pose odométrique avec la pose vraie à cet instant.
    odom_t, truth_t = associate(odom, truth)
    odom_w = transform(odom_t, *align_first_pose(odom_t, truth_t))
    err = position_errors(odom_w, truth_t)
    heading_err = np.degrees(wrap(odom_w[:, 3] - truth_t[:, 3]))
    dist = travelled_distance(truth_t)
    end_gap = np.hypot(truth[-1, 1] - truth[0, 1], truth[-1, 2] - truth[0, 2])
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
    ax1.set_title('Trajectoire de référence dans la salle as52_room')
    ax1.legend(loc='upper right', fontsize=8)

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
