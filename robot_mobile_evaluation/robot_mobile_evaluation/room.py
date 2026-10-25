"""Plan de la salle as52_room : source unique pour les calculs et les figures.

Doit rester cohérent avec robot_mobile_bringup/worlds/as52_room.sdf et
robot_mobile_bringup/models/as52_room/model.sdf.
"""

import math

# Faces intérieures des murs (m) : x_min, x_max, y_min, y_max
ROOM = (-3.5, 3.5, -2.5, 2.5)

# Obstacles rectangulaires : (nom, x, y, demi-largeur x, demi-largeur y)
BOXES = [
    ('pilier A', -1.6, 0.3, 0.15, 0.15),
    ('pilier B', 0.5, -0.6, 0.15, 0.15),
    ('cible rouge', 1.7, -0.3, 0.2, 0.2),
    ('table', -0.4, -0.2, 0.4565, 0.4565),  # plateau carré de 0,913 m
]
# Obstacles ronds : (nom, x, y, rayon approximatif)
DISCS = [('poubelle', 0.8, 0.5, 0.2)]


def ray_to_wall(px, py, angle, room=ROOM):
    """Distance du point (px, py), intérieur à la salle, au premier mur dans la direction angle."""
    x_min, x_max, y_min, y_max = room
    dx, dy = math.cos(angle), math.sin(angle)
    candidates = []
    if abs(dx) > 1e-9:
        candidates.append(((x_max if dx > 0 else x_min) - px) / dx)
    if abs(dy) > 1e-9:
        candidates.append(((y_max if dy > 0 else y_min) - py) / dy)
    return min(t for t in candidates if t > 0)


def draw_room(ax, labels=True):
    """Dessine les murs et les obstacles sur un axe matplotlib (repère du monde)."""
    from matplotlib.patches import Circle, Rectangle

    x_min, x_max, y_min, y_max = ROOM
    ax.add_patch(Rectangle((x_min, y_min), x_max - x_min, y_max - y_min,
                           fill=False, lw=3, color='0.3'))
    for name, x, y, hx, hy in BOXES:
        color = 'red' if 'cible' in name else '0.5'
        ax.add_patch(Rectangle((x - hx, y - hy), 2 * hx, 2 * hy, color=color, alpha=0.7))
        if labels:
            ax.annotate(name, (x, y + hy + 0.08), ha='center', fontsize=8)
    for name, x, y, r in DISCS:
        ax.add_patch(Circle((x, y), r, color='0.7', alpha=0.7))
        if labels:
            ax.annotate(name, (x, y), ha='center', va='center', fontsize=8)
    ax.set_aspect('equal')
    ax.set_xlim(x_min - 0.3, x_max + 0.3)
    ax.set_ylim(y_min - 0.3, y_max + 0.3)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.grid(alpha=0.3)
