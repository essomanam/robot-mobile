"""Compare les essais d'approche de cible (E3) : tableau de résultats et trajectoires.

Usage :
    ros2 run robot_mobile_evaluation evaluate_target_trials <dossier_essais> <dossier_figures>

Chaque sous-dossier contient le result.json et le ground_truth.tum écrits par trial_monitor.
"""

import json
import math
from pathlib import Path
import sys

import matplotlib
import matplotlib.pyplot as plt
from robot_mobile_evaluation.room import clearance, draw_room
from robot_mobile_evaluation.trajectory import read_tum

ROBOT_RADIUS = 0.21  # m : robot assimilé à un disque

matplotlib.use('Agg')


def load(runs_dir):
    runs = []
    for d in sorted(p for p in runs_dir.iterdir() if (p / 'result.json').exists()):
        r = json.loads((d / 'result.json').read_text())
        r['truth'] = read_tum(d / 'ground_truth.tum')
        # marge recalculée avec le plan actuel de la salle (indépendante de la version de l'outil
        # qui a enregistré l'essai)
        r['min_obstacle_margin'] = min(clearance(x, y) for _, x, y, _ in r['truth']) - ROBOT_RADIUS
        runs.append(r)
    return runs


def table(runs):
    lines = ['| Scénario | Résultat | Durée (s) | dont recherche (s) | Distance finale (m) | '
             'Écart à 0,6 m (cm) | Cap final (°) | Trajet (m) | Marge min. obstacles (m) |',
             '|' + '---|' * 9]
    for r in runs:
        ok = r['success']
        lines.append(
            f'| {r["scenario"]} | {"réussi" if ok else "**échec**"} | {r["duration"]:.1f} '
            f'| {r["search_time"]:.1f} | {r["final_face_distance"]:.3f} '
            f'| {100 * r["distance_error"]:+.1f} | {r["final_bearing_error_deg"]:+.1f} '
            f'| {r["path_length"]:.2f} | {r["min_obstacle_margin"]:.2f} |')
    done = [r for r in runs if r['success']]
    lines.append(f'\nRéussite : {len(done)} / {len(runs)}')
    if done:
        mean = sum(abs(r['distance_error']) for r in done) / len(done)
        bear = sum(abs(r['final_bearing_error_deg']) for r in done) / len(done)
        lines.append(f'Sur les essais réussis : écart moyen de distance {100 * mean:.1f} cm, '
                     f'écart moyen de cap {bear:.1f}°')
    return '\n'.join(lines)


def plot(runs, out):
    fig, ax = plt.subplots(figsize=(9, 6.5))
    draw_room(ax)
    for r in runs:
        t = r['truth']
        color = 'tab:green' if r['success'] else 'tab:red'
        ax.plot(t[:, 1], t[:, 2], color=color, lw=1.8)
        x, y, yaw = r['start']
        ax.annotate('', xy=(x + 0.3 * math.cos(yaw), y + 0.3 * math.sin(yaw)), xytext=(x, y),
                    arrowprops={'arrowstyle': '->', 'color': 'black', 'lw': 1.5})
        ax.annotate(r['scenario'], (x, y), textcoords='offset points', xytext=(4, -12),
                    fontsize=7)
    ax.plot([], [], color='tab:green', label='essai réussi')
    ax.plot([], [], color='tab:red', label='échec')
    ax.set_title('Approche de la cible rouge : trajectoires vraies (flèche = départ)')
    ax.legend(loc='upper left', fontsize=8)
    fig.tight_layout()
    fig.savefig(out / 'E3_essais_cible.png', dpi=100)


def main():
    runs_dir, out = Path(sys.argv[1]), Path(sys.argv[2])
    out.mkdir(parents=True, exist_ok=True)
    runs = load(runs_dir)
    text = table(runs)
    print(text)
    (runs_dir / 'results.md').write_text(text + '\n')
    plot(runs, out)
    print(f'\nFigure : {out / "E3_essais_cible.png"}')


if __name__ == '__main__':
    main()
