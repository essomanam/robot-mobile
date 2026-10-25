"""Compare les trajectoires et les cartes produites par plusieurs SLAM sur le même jeu de données.

Chaque sous-dossier de <dossier_essais> est un essai écrit par trajectory_logger ; son nom est
`<méthode>` ou `<méthode>_<n>` pour des répétitions de la même méthode.

Pour chaque méthode :
  - odométrie des roues seule (référence de base) et estimation en ligne du SLAM :
    ATE (RMSE après alignement rigide optimal), RPE sur 1 m, erreur finale de position
    (première pose alignée sur la vérité terrain : c'est la dérive accumulée) ;
  - graphe optimisé en fin d'essai (RTAB-Map, exporté de sa base avec rtabmap-export) : ATE ;
  - fermetures de boucle et temps de calcul ;
  - RTAB-Map : erreur de chaque contrainte de fermeture de boucle (déplacement mesuré entre
    deux nœuds comparé au déplacement vrai), pour repérer les contraintes fausses.

Usage :
    ros2 run robot_mobile_evaluation evaluate_slam <dossier_essais> <dossier_figures>
"""

from collections import defaultdict
import json
import math
from pathlib import Path
import re
import sqlite3
import subprocess
import sys

import cv2
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.transforms import Affine2D
import numpy as np
from robot_mobile_evaluation.room import draw_room
from robot_mobile_evaluation.trajectory import (align_first_pose, associate, ate_rmse,
                                                interpolate, position_errors, read_tum,
                                                relative_motion, rpe, transform,
                                                travelled_distance)

matplotlib.use('Agg')

LABELS = {
    'odometry': 'A. Odométrie des roues seule',
    'rtabmap_rgbd': 'B. RTAB-Map RGB-D + roues',
    'rtabmap_rgbd_vo': 'C. RTAB-Map RGB-D seule (odométrie visuelle)',
    'rtabmap_rgbd_scan': 'D. RTAB-Map RGB-D + LiDAR + roues',
    'slam_toolbox': 'E. slam_toolbox (LiDAR + roues)',
}
ORDER = list(LABELS)


def evaluate(estimate, truth):
    """Métriques d'une estimation exprimée dans son propre repère (map ou odom)."""
    est, gt = associate(estimate, truth)
    if len(est) < 10:
        return None
    ate, _ = ate_rmse(est, gt)
    world = transform(est, *align_first_pose(est, gt))  # repère du monde, sans optimisation
    err = position_errors(world, gt)
    rpe_t, rpe_r = rpe(est, gt, delta=1.0)
    return {'ate': ate, 'final': float(err[-1]), 'max': float(err.max()),
            'rpe_t': rpe_t, 'rpe_r': rpe_r, 'world': world, 'truth': gt, 'err': err}


def export_graph(run_dir):
    """Renvoie le fichier des poses du graphe optimisé (TUM), exporté de la base RTAB-Map."""
    graph = run_dir / 'slam_graph_poses.txt'
    if not graph.exists() and (run_dir / 'rtabmap.db').exists():
        subprocess.run(['rtabmap-export', '--poses', '--poses_format', '10',
                        '--output', 'slam_graph', '--output_dir', str(run_dir),
                        str(run_dir / 'rtabmap.db')],
                       check=True, capture_output=True)
    return graph if graph.exists() else None


def constraint_errors(run_dir, truth):
    """Renvoie l'erreur (m) de chaque contrainte de boucle RTAB-Map, d'après la vérité terrain.

    Liens de type 1 (fermeture globale) et 2 (proximité) de la base ; chaque contrainte est
    comptée une fois (du nœud le plus récent vers le plus ancien).
    """
    db = run_dir / 'rtabmap.db'
    if not db.exists():
        return None
    con = sqlite3.connect(db)
    stamps = dict(con.execute('SELECT id, stamp FROM Node'))
    errors = []
    for src, dst, blob in con.execute(
            'SELECT from_id, to_id, transform FROM Link WHERE type IN (1, 2) AND from_id > to_id'):
        m = np.frombuffer(blob, dtype=np.float32)
        if m.size != 12:  # format inattendu (transformation compressée)
            continue
        m = m.reshape(3, 4)
        a, b = interpolate(truth, np.array([stamps[src], stamps[dst]]))
        dx, dy, _ = relative_motion(a, b)
        errors.append(math.hypot(m[0, 3] - dx, m[1, 3] - dy))
    con.close()
    return np.array(errors)


def load_run(run_dir):
    truth = read_tum(run_dir / 'ground_truth.tum')
    stats = json.loads((run_dir / 'stats.json').read_text())
    result = {'stats': stats, 'truth': truth,
              'odometry': evaluate(read_tum(run_dir / 'odometry.tum'), truth),
              'slam': evaluate(read_tum(run_dir / 'slam.tum'), truth)}
    graph = export_graph(run_dir)
    result['final'] = evaluate(read_tum(graph), truth) if graph else None
    result['constraints'] = constraint_errors(run_dir, truth)
    if (run_dir / 'map.yaml').exists():
        result['map'] = run_dir
    return result


def mean_std(values):
    values = [v for v in values if v is not None and np.isfinite(v)]
    if not values:
        return '—'
    if len(values) == 1:
        return f'{values[0]:.3f}'
    return f'{np.mean(values):.3f} ± {np.std(values):.3f}'


def table(groups):
    """Tableau Markdown : une ligne par méthode (moyenne ± écart-type sur les répétitions)."""
    head = ('| Méthode | Essais | ATE en ligne (m) | Erreur finale (m) | RPE 1 m (m) | '
            'RPE 1 m (°) | ATE graphe final (m) | Fermetures de boucle | Temps RTAB-Map (ms) |')
    lines = [head, '|' + '---|' * 9]
    odom_runs = [r['odometry'] for runs in groups.values() for r in runs if r['odometry']]
    lines.append(
        f'| {LABELS["odometry"]} | {len(odom_runs)} | {mean_std([o["ate"] for o in odom_runs])} '
        f'| {mean_std([o["final"] for o in odom_runs])} '
        f'| {mean_std([o["rpe_t"] for o in odom_runs])} '
        f'| {mean_std([o["rpe_r"] for o in odom_runs])} | — | — | — |')
    for method in ORDER[1:]:
        runs = groups.get(method, [])
        if not runs:
            continue
        slam = [r['slam'] for r in runs if r['slam']]
        final = [r['final']['ate'] for r in runs if r['final']]
        loops = [r['stats']['loop_closures'] for r in runs]
        timing = [r['stats']['timing_ms']['mean'] for r in runs if r['stats'].get('timing_ms')]
        lines.append(
            f'| {LABELS[method]} | {len(runs)} | {mean_std([s["ate"] for s in slam])} '
            f'| {mean_std([s["final"] for s in slam])} | {mean_std([s["rpe_t"] for s in slam])} '
            f'| {mean_std([s["rpe_r"] for s in slam])} | {mean_std(final)} '
            f'| {mean_std(loops) if method != "slam_toolbox" else "—"} '
            f'| {mean_std(timing)} |')
    return '\n'.join(lines)


def constraints_table(groups):
    """Qualité des contraintes de fermeture de boucle (RTAB-Map), tous essais confondus."""
    lines = ['| Méthode | Contraintes | Erreur médiane (cm) | Erreur max (cm) | > 10 cm |',
             '|---|---|---|---|---|']
    for method in ORDER[1:]:
        errors = [r['constraints'] for r in groups.get(method, [])
                  if r.get('constraints') is not None and len(r['constraints'])]
        if not errors:
            continue
        e = np.concatenate(errors)
        lines.append(f'| {LABELS[method]} | {len(e)} | {100 * np.median(e):.1f} '
                     f'| {100 * e.max():.1f} | {int((e > 0.10).sum())} |')
    return '\n'.join(lines)


def plot_trajectories(groups, out):
    methods = [m for m in ORDER[1:] if m in groups]
    fig, axes = plt.subplots(1, len(methods), figsize=(5.2 * len(methods), 4.4), squeeze=False)
    for ax, method in zip(axes[0], methods):
        run = groups[method][0]
        draw_room(ax, labels=False)
        ax.plot(run['truth'][:, 1], run['truth'][:, 2], color='tab:blue', lw=2.5,
                label='vérité terrain')
        if run['odometry']:
            w = run['odometry']['world']
            ax.plot(w[:, 1], w[:, 2], ':', color='tab:orange', lw=1.5, label='odométrie')
        if run['slam']:
            w = run['slam']['world']
            ax.plot(w[:, 1], w[:, 2], '--', color='tab:green', lw=1.8, label='SLAM (en ligne)')
        ax.set_title(LABELS[method], fontsize=10)
        ax.legend(loc='upper right', fontsize=7)
    fig.tight_layout()
    fig.savefig(out / 'E2_trajectoires.png', dpi=100)


def plot_errors(groups, out):
    fig, ax = plt.subplots(figsize=(9, 4.2))
    first = next(iter(groups.values()))[0]
    if first['odometry']:
        o = first['odometry']
        ax.plot(travelled_distance(o['truth']), o['err'], ':', color='0.3',
                label=LABELS['odometry'])
    for method in ORDER[1:]:
        if method in groups and groups[method][0]['slam']:
            s = groups[method][0]['slam']
            ax.plot(travelled_distance(s['truth']), s['err'], label=LABELS[method])
    ax.set_xlabel('distance parcourue (m)')
    ax.set_ylabel('erreur de position (m)')
    ax.set_title('Erreur de position en ligne (première pose alignée sur la vérité terrain)')
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out / 'E2_erreurs.png', dpi=100)


def read_map(run_dir):
    meta = {}
    for line in (run_dir / 'map.yaml').read_text().splitlines():
        key, _, value = line.partition(':')
        meta[key.strip()] = value.strip()
    origin = [float(v) for v in meta['origin'].strip('[]').split(',')]
    return cv2.imread(str(run_dir / 'map.png'), cv2.IMREAD_GRAYSCALE), \
        float(meta['resolution']), origin


def plot_maps(groups, out):
    methods = [m for m in ORDER[1:] if m in groups and 'map' in groups[m][0]]
    if not methods:
        return
    fig, axes = plt.subplots(1, len(methods), figsize=(5.2 * len(methods), 4.4), squeeze=False)
    for ax, method in zip(axes[0], methods):
        run = groups[method][0]
        image, res, (ox, oy, oyaw) = read_map(run['map'])
        h, w = image.shape
        # repère map = pose du robot au départ : on place la carte dans le monde avec la pose
        # vraie de départ (aucune optimisation, pour juger la carte telle que le SLAM l'a faite)
        start = run['truth'][0]
        to_world = (Affine2D().rotate(oyaw).translate(ox, oy)
                    .rotate(start[3]).translate(start[1], start[2]))
        ax.imshow(image, cmap='gray', vmin=0, vmax=255, origin='upper',
                  extent=(0, w * res, 0, h * res), transform=to_world + ax.transData)
        draw_room(ax, labels=False)
        for patch in ax.patches:
            patch.set_fill(False)
            patch.set_edgecolor('tab:red')
            patch.set_alpha(0.8)
        ax.set_title(LABELS[method], fontsize=10)
    fig.suptitle("Cartes d'occupation (gris : inconnu, blanc : libre, noir : occupé) "
                 'et plan réel de la salle (rouge)', fontsize=10)
    fig.tight_layout()
    fig.savefig(out / 'E2_cartes.png', dpi=100)


def main():
    runs_dir, out = Path(sys.argv[1]), Path(sys.argv[2])
    out.mkdir(parents=True, exist_ok=True)
    groups = defaultdict(list)
    for run_dir in sorted(p for p in runs_dir.iterdir() if (p / 'stats.json').exists()):
        method = re.sub(r'_\d+$', '', run_dir.name)
        groups[method].append(load_run(run_dir))
    text = table(groups) + '\n\n' + constraints_table(groups)
    print(text)
    (runs_dir / 'results.md').write_text(text + '\n')
    plot_trajectories(groups, out)
    plot_errors(groups, out)
    plot_maps(groups, out)
    print(f'\nFigures : {out}/E2_trajectoires.png, E2_erreurs.png, E2_cartes.png')


if __name__ == '__main__':
    main()
