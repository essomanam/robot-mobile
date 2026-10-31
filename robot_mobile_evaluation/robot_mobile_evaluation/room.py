"""Plan de la salle as52_room : source unique pour les calculs et les figures.

Doit rester cohérent avec robot_mobile_bringup/worlds/as52_room.sdf et
robot_mobile_bringup/models/as52_room/model.sdf.
"""

import math

# Faces intérieures des murs (m) : x_min, x_max, y_min, y_max
ROOM = (-3.5, 3.5, -2.5, 2.5)

# Obstacles à hauteur du robot (≈ 0,2 m), rectangulaires :
# (nom, x, y, demi-largeur x, demi-largeur y)
BOXES = [
    ('pilier A', -1.6, 0.3, 0.15, 0.15),
    ('pilier B', 0.5, -0.6, 0.15, 0.15),
    ('cible rouge', 1.7, -0.3, 0.2, 0.2),
    # table (modèle cafe_table) : seuls le socle (56 cm de côté, 4 cm de haut, plus haut que le
    # dessous du robot à 1 cm) et la colonne centrale (4,2 cm) sont à hauteur du robot
    ('socle de table', -0.4, -0.2, 0.28, 0.28),
]
# Obstacles ronds : (nom, x, y, rayon approximatif)
DISCS = [('poubelle', 0.8, 0.5, 0.2)]
# Plateau de la table (0,913 m de côté, à 0,755 m de haut) : le robot passe dessous ;
# il n'est que dessiné, pas compté comme obstacle.
TABLE_TOP = ('table', -0.4, -0.2, 0.4565, 0.4565)


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


def distance_to_box(px, py, cx, cy, hx, hy):
    """Distance du point (px, py) au rectangle centré en (cx, cy), demi-côtés hx, hy (0 dedans)."""
    dx = max(abs(px - cx) - hx, 0.0)
    dy = max(abs(py - cy) - hy, 0.0)
    return math.hypot(dx, dy)


def sight_distance_to_box(px, py, cx, cy, hx, hy):
    """Distance du point (px, py) à la face d'un rectangle, le long de la droite vers son centre.

    C'est ce que mesure un robot qui vise le centre de la cible : il n'est pas toujours égal à la
    distance au point le plus proche (approche en biais d'un coin).
    """
    dx, dy = cx - px, cy - py
    length = math.hypot(dx, dy)
    if length == 0.0:
        return 0.0
    ux, uy = dx / length, dy / length
    # entrée dans la bande |x - cx| <= hx et dans la bande |y - cy| <= hy (méthode des « slabs »)
    tx = (abs(dx) - hx) / abs(ux) if abs(ux) > 1e-12 else 0.0
    ty = (abs(dy) - hy) / abs(uy) if abs(uy) > 1e-12 else 0.0
    return max(tx, ty, 0.0)


def clearance(px, py, exclude=('cible rouge',), room=ROOM):
    """Plus petite distance du point (px, py) aux murs et aux obstacles (sauf ceux exclus)."""
    x_min, x_max, y_min, y_max = room
    distances = [px - x_min, x_max - px, py - y_min, y_max - py]
    distances += [distance_to_box(px, py, x, y, hx, hy)
                  for name, x, y, hx, hy in BOXES if name not in exclude]
    distances += [max(math.hypot(px - x, py - y) - r, 0.0)
                  for name, x, y, r in DISCS if name not in exclude]
    return min(distances)


def draw_room(ax, labels=True):
    """Dessine les murs et les obstacles sur un axe matplotlib (repère du monde)."""
    from matplotlib.patches import Circle, Rectangle

    x_min, x_max, y_min, y_max = ROOM
    ax.add_patch(Rectangle((x_min, y_min), x_max - x_min, y_max - y_min,
                           fill=False, lw=3, color='0.3'))
    for name, x, y, hx, hy in BOXES:
        color = 'red' if 'cible' in name else '0.5'
        ax.add_patch(Rectangle((x - hx, y - hy), 2 * hx, 2 * hy, color=color, alpha=0.7))
        if labels and 'table' not in name:
            ax.annotate(name, (x, y + hy + 0.08), ha='center', fontsize=8)
    name, x, y, hx, hy = TABLE_TOP
    ax.add_patch(Rectangle((x - hx, y - hy), 2 * hx, 2 * hy, fill=False, ls='--', color='0.5'))
    if labels:
        ax.annotate('table', (x, y + hy + 0.08), ha='center', fontsize=8)
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
