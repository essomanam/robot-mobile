"""Comportement d'approche d'une cible vue par la caméra (sans ROS, testable seul).

Machine à états :

    RECHERCHE --cible vue--> ALIGNEMENT --cible centrée--> APPROCHE --distance atteinte--> ARRIVÉE
        ^                        |   ^---cible décentrée----'  |
        '-----cible perdue-------'-----------------------------'

Lois de commande (asservissement visuel) :
    rotation : w = sat(k_w * angle)                 angle > 0 : cible à gauche
    avance   : v = sat(k_v * (distance - d*)) * max(0, cos(angle))     (APPROCHE seulement)
Rotation et avance sont simultanées en APPROCHE : trajectoire courbe, pas « tourner puis avancer ».
Un obstacle trop proche devant le robot (LiDAR) annule l'avance mais pas la rotation.
"""

from dataclasses import dataclass
import math

SEARCH, ALIGN, APPROACH, ARRIVED = 'RECHERCHE', 'ALIGNEMENT', 'APPROCHE', 'ARRIVEE'


@dataclass
class ApproachParams:
    desired_distance: float = 0.6    # m : arrêt, du centre du robot à la face de la cible
    distance_tolerance: float = 0.02  # m
    align_angle: float = math.radians(20)     # au-delà, on tourne sur place (ALIGNEMENT)
    arrive_angle: float = math.radians(5)     # cible considérée centrée à l'arrivée
    k_v: float = 0.5                 # (m/s) par m d'écart de distance
    k_w: float = 1.2                 # (rad/s) par rad d'écart angulaire
    max_v: float = 0.2               # m/s (Waffle réel : 0,26)
    max_w: float = 0.8               # rad/s (Waffle réel : 1,82)
    search_w: float = 0.5            # rad/s : rotation sur place pendant la recherche
    lost_timeout: float = 1.0        # s sans voir la cible avant de repasser en RECHERCHE


def clamp(value, limit):
    return max(-limit, min(limit, value))


class ApproachController:
    """Calcule (v, w) à partir de la cible observée ; une instance par mission."""

    def __init__(self, params=None):
        self.p = params or ApproachParams()
        self.state = SEARCH
        self.last_seen = None      # instant (s) de la dernière observation
        self.last_angle = 1.0      # signe : côté où chercher en premier (gauche par défaut)

    def update(self, now, target, obstacle_ahead=False):
        """Met à jour l'état et renvoie (v, w).

        now : instant (s) ; target : (angle en rad, distance en m) dans le repère du robot,
        ou None si la cible n'est pas vue ; obstacle_ahead : obstacle trop proche devant.
        """
        p = self.p
        if target is not None:
            self.last_seen = now
            angle, distance = target
            self.last_angle = angle
        lost = self.last_seen is None or now - self.last_seen > p.lost_timeout

        if lost:
            if self.state != ARRIVED:
                self.state = SEARCH
            if self.state == SEARCH:
                return 0.0, math.copysign(p.search_w, self.last_angle)
            return 0.0, 0.0
        if target is None:       # brièvement invisible : on garde la dernière commande sûre
            return 0.0, 0.0

        error = distance - p.desired_distance
        if self.state in (SEARCH, ALIGN):
            self.state = APPROACH if abs(angle) < p.align_angle else ALIGN
        elif self.state == APPROACH:
            if abs(angle) > 1.5 * p.align_angle:          # hystérésis
                self.state = ALIGN
            elif abs(error) < p.distance_tolerance and abs(angle) < p.arrive_angle:
                self.state = ARRIVED
        elif self.state == ARRIVED and (abs(error) > 3 * p.distance_tolerance
                                        or abs(angle) > p.align_angle):
            self.state = APPROACH                         # la cible s'est éloignée

        w = clamp(p.k_w * angle, p.max_w)
        if self.state == ALIGN:
            return 0.0, w
        if self.state == ARRIVED:
            return 0.0, 0.0
        v = clamp(p.k_v * error, p.max_v) * max(0.0, math.cos(angle))
        if obstacle_ahead and v > 0:
            v = 0.0
        return v, w
