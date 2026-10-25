"""Évalue la localisation de RTAB-Map dans une carte construite lors d'un autre parcours.

Le repère `map` de la carte est la pose du robot au début du parcours de cartographie : il
est connu exactement grâce à la vérité terrain de ce parcours. L'estimation de localisation
est donc comparée à la vérité **sans aucun alignement** : le robot doit savoir où il est dans
la carte, pas seulement reproduire la forme de sa trajectoire.

Au démarrage, RTAB-Map ne sait pas où se trouve le robot (il le suppose à l'origine de la
carte) ; il se **relocalise** quand il reconnaît un lieu de la carte. Mesures :
  - temps et distance parcourue avant relocalisation (première fois que l'erreur passe sous
    le seuil, 10 cm par défaut) ;
  - après relocalisation : erreur de position RMS et maximale, erreur de cap RMS, part du
    temps passée sous le seuil.

Usage :
    ros2 run robot_mobile_evaluation evaluate_localization <dossier_essais> \
        <vérité terrain de la cartographie (.tum)> <dossier_figures>
"""

from collections import defaultdict
import json
from pathlib import Path
import re
import sys

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from robot_mobile_evaluation.evaluate_slam import LABELS
from robot_mobile_evaluation.room import draw_room
from robot_mobile_evaluation.trajectory import (associate, position_errors, read_tum,
                                                transform, travelled_distance, wrap)

matplotlib.use('Agg')

THRESHOLD = 0.10  # m : erreur en dessous de laquelle le robot est considéré comme localisé


def evaluate_run(run_dir, map_origin):
    truth = read_tum(run_dir / 'ground_truth.tum')
    stats = json.loads((run_dir / 'stats.json').read_text())
    est, gt = associate(read_tum(run_dir / 'slam.tum'), truth)
    world = transform(est, map_origin[3], map_origin[1], map_origin[2])  # map -> monde
    err = position_errors(world, gt)
    heading = np.degrees(np.abs(wrap(world[:, 3] - gt[:, 3])))
    t = gt[:, 0] - truth[0, 0]
    dist = travelled_distance(gt)
    below = np.nonzero(err < THRESHOLD)[0]
    result = {'label': stats['label'], 't': t, 'err': err, 'world': world, 'truth': gt,
              'relocalizations': stats['loop_closures'], 'k': None}
    if len(below) == 0:
        result.update(reloc_time=None, reloc_dist=None, rmse=None, max=None, heading=None,
                      ratio=0.0)
        return result
    k = below[0]
    after = err[k:]
    result.update(k=int(k), reloc_time=float(t[k]), reloc_dist=float(dist[k]),
                  rmse=float(np.sqrt(np.mean(after ** 2))), max=float(after.max()),
                  heading=float(np.sqrt(np.mean(heading[k:] ** 2))),
                  ratio=float(np.mean(after < THRESHOLD)))
    return result


def fmt(values, scale=1.0, digits=1):
    values = [v * scale for v in values if v is not None]
    if not values:
        return '—'
    if len(values) == 1:
        return f'{values[0]:.{digits}f}'
    return f'{np.mean(values):.{digits}f} ± {np.std(values):.{digits}f}'


def table(groups):
    lines = ['| Méthode | Essais | Relocalisation (s) | Distance avant (m) | '
             'Erreur RMS après (cm) | Erreur max après (cm) | Cap RMS après (°) | '
             f'Temps < {100 * THRESHOLD:.0f} cm après (%) | Relocalisations |',
             '|' + '---|' * 9]
    for method, runs in groups.items():
        def col(key, scale=1.0, digits=1):
            return fmt([r[key] for r in runs], scale, digits)
        lines.append(
            f'| {LABELS.get(method, method)} | {len(runs)} | {col("reloc_time")} '
            f'| {col("reloc_dist", 1, 2)} | {col("rmse", 100)} | {col("max", 100)} '
            f'| {col("heading", 1, 2)} | {col("ratio", 100, 0)} | {col("relocalizations")} |')
    return '\n'.join(lines)


def plot(groups, out):
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 4.8),
                                   gridspec_kw={'width_ratios': [1.3, 1]})
    for method, runs in groups.items():
        for i, r in enumerate(runs):
            suffix = f' (essai {i + 1})' if len(runs) > 1 else ''
            ax1.plot(r['t'], r['err'], lw=1.2, label=LABELS.get(method, method) + suffix)
    ax1.axhline(THRESHOLD, color='0.4', ls='--', lw=1, label=f'seuil {100 * THRESHOLD:.0f} cm')
    ax1.set_yscale('log')
    ax1.set_xlabel("temps depuis le début de l'enregistrement (s)")
    ax1.set_ylabel('erreur de position dans la carte (m, échelle log)')
    ax1.set_title('Localisation dans la carte de référence')
    ax1.grid(alpha=0.3, which='both')
    ax1.legend(fontsize=7)

    # configuration retenue (D-014) si elle est présente, sinon la première
    first = (groups.get('rtabmap_rgbd_scan') or next(iter(groups.values())))[0]
    draw_room(ax2, labels=False)
    ax2.plot(first['truth'][:, 1], first['truth'][:, 2], color='tab:blue', lw=2.5,
             label='vérité terrain')
    if first['k'] is not None:  # position estimée, à partir de la relocalisation
        w = first['world'][first['k']:]
        ax2.plot(w[:, 1], w[:, 2], '--', color='tab:green', lw=1.5,
                 label='position estimée (une fois localisé)')
    ax2.plot(*first['truth'][0, 1:3], 'o', color='black', label='départ')
    ax2.set_title(LABELS.get(first['label'], first['label']), fontsize=10)
    ax2.legend(fontsize=7, loc='upper right')
    fig.tight_layout()
    fig.savefig(out / 'E2_localisation.png', dpi=100)


def main():
    runs_dir, map_truth, out = Path(sys.argv[1]), Path(sys.argv[2]), Path(sys.argv[3])
    out.mkdir(parents=True, exist_ok=True)
    map_origin = read_tum(map_truth)[0]  # pose du robot au début de la cartographie
    groups = defaultdict(list)
    for run_dir in sorted(p for p in runs_dir.iterdir() if (p / 'stats.json').exists()):
        groups[re.sub(r'_\d+$', '', run_dir.name)].append(evaluate_run(run_dir, map_origin))
    text = table(groups)
    print(text)
    (runs_dir / 'results.md').write_text(text + '\n')
    plot(groups, out)
    print(f'\nFigure : {out / "E2_localisation.png"}')


if __name__ == '__main__':
    main()
