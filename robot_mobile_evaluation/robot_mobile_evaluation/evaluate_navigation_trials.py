"""Compare les essais de navigation avec obstacles non cartographiés (E4).

Usage :
    ros2 run robot_mobile_evaluation evaluate_navigation_trials <dossier_essais> <dossier_figures>

Chaque sous-dossier `<scénario>_<variante>_<n>` contient le result.json et le ground_truth.tum
écrits par navigation_trial. Les essais sont regroupés par scénario et variante.
"""

from collections import defaultdict
import json
from pathlib import Path
import re
import sys

import matplotlib
from matplotlib.patches import Rectangle
import matplotlib.pyplot as plt
from robot_mobile_evaluation.room import clearance, distance_to_box, draw_room
from robot_mobile_evaluation.trajectory import read_tum

ROBOT_RADIUS = 0.21  # m : robot assimilé à un disque

matplotlib.use('Agg')

# Obstacles ajoutés (worlds/as52_room_obstacles.sdf) : nom, x, y, demi-largeur x, demi-largeur y
EXTRA = [('carton (40 cm)', 0.0, -1.5, 0.2, 0.2), ('obstacle bas (8 cm)', 0.0, 1.5, 0.3, 0.3)]
VARIANTS = {'profondeur': 'LiDAR + profondeur', 'lidar_seul': 'LiDAR seul'}


def load(runs_dir):
    groups = defaultdict(list)
    for d in sorted(p for p in runs_dir.iterdir() if (p / 'result.json').exists()):
        r = json.loads((d / 'result.json').read_text())
        truth = d / 'ground_truth.tum'
        r['truth'] = read_tum(truth) if truth.exists() else None
        if r['truth'] is not None and len(r['truth']):
            # marges recalculées avec le plan actuel de la salle et les obstacles ajoutés
            pts = [(x, y) for _, x, y, _ in r['truth']]
            r['min_room_margin'] = min(clearance(x, y) for x, y in pts) - ROBOT_RADIUS
            r['min_extra_obstacle_margin'] = min(
                distance_to_box(x, y, *box[1:]) for x, y in pts for box in EXTRA) - ROBOT_RADIUS
            r['collision'] = min(r['min_room_margin'], r['min_extra_obstacle_margin']) < 0.0
        groups[re.sub(r'_\d+$', '', d.name)].append(r)
    return groups


def table(groups):
    lines = ["| Scénario | Capteurs d'obstacles | Essais | Réussis | Contacts | Durée (s) | "
             "Trajet (m) | Marge min. à l'obstacle ajouté (m) | Marge min. salle (m) | "
             'Statut Nav2 |',
             '|' + '---|' * 10]
    for key, runs in groups.items():
        scenario, _, variant = key.partition('_')
        ok = [r for r in runs if r['success']]
        contacts = sum(1 for r in runs if r['collision'])
        mean = lambda k: sum(r[k] for r in runs) / len(runs)  # noqa: E731
        statuses = ', '.join(sorted({r['nav2_status'] for r in runs}))
        lines.append(f'| {scenario} | {VARIANTS.get(variant, variant)} | {len(runs)} | {len(ok)} '
                     f'| {contacts} | {mean("duration"):.1f} | {mean("path_length"):.2f} '
                     f'| {min(r["min_extra_obstacle_margin"] for r in runs):+.2f} '
                     f'| {min(r["min_room_margin"] for r in runs):+.2f} | {statuses} |')
    return '\n'.join(lines)


def plot(groups, out):
    scenarios = sorted({k.partition('_')[0] for k in groups})
    fig, axes = plt.subplots(1, len(scenarios), figsize=(7.5 * len(scenarios), 5.5),
                             squeeze=False)
    colors = {'profondeur': 'tab:green', 'lidar_seul': 'tab:red'}
    for ax, scenario in zip(axes[0], scenarios):
        draw_room(ax, labels=False)
        for name, x, y, hx, hy in EXTRA:
            ax.add_patch(Rectangle((x - hx, y - hy), 2 * hx, 2 * hy, color='tab:purple',
                                   alpha=0.6))
            ax.annotate(name, (x, y + hy + 0.08), ha='center', fontsize=8, color='tab:purple')
        for key, runs in groups.items():
            if not key.startswith(scenario + '_'):
                continue
            variant = key.partition('_')[2]
            for i, r in enumerate(runs):
                if r['truth'] is None or len(r['truth']) == 0:
                    continue
                ax.plot(r['truth'][:, 1], r['truth'][:, 2], color=colors.get(variant, 'k'),
                        lw=1.6, label=VARIANTS.get(variant, variant) if i == 0 else None)
            gx, gy = runs[0]['goal'][:2]
            ax.plot(gx, gy, '*', color='gold', ms=14, mec='k')
            sx, sy = runs[0]['start'][:2]
            ax.plot(sx, sy, 'o', color='k', ms=6)
        ax.set_title(f'Scénario « {scenario} » (rond : départ, étoile : but)', fontsize=10)
        ax.legend(loc='upper right', fontsize=8)
    fig.tight_layout()
    fig.savefig(out / 'E4_navigation_obstacles.png', dpi=100, bbox_inches='tight')


def main():
    runs_dir, out = Path(sys.argv[1]), Path(sys.argv[2])
    out.mkdir(parents=True, exist_ok=True)
    groups = load(runs_dir)
    text = table(groups)
    print(text)
    (runs_dir / 'results.md').write_text(text + '\n')
    plot(groups, out)
    print(f'\nFigure : {out / "E4_navigation_obstacles.png"}')


if __name__ == '__main__':
    main()
