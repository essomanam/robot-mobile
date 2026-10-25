"""Trajectoires planes : format TUM, alignement et métriques d'erreur (ATE, RPE).

Une trajectoire est un tableau numpy de forme (N, 4) : temps (s), x (m), y (m), cap (rad).
Le robot se déplace dans un plan, on travaille donc en 2D (x, y, cap), comme le SLAM
configuré en 3 degrés de liberté (Reg/Force3DoF).

Définitions (Sturm et al., 2012, benchmark TUM RGB-D) :
  - ATE (Absolute Trajectory Error) : RMSE des écarts de position après avoir superposé
    au mieux l'estimation sur la vérité terrain par une transformation rigide ;
  - RPE (Relative Pose Error) : erreur sur le déplacement entre deux instants, ici séparés
    d'une distance parcourue fixe ; elle mesure la dérive locale.
"""

import math

import numpy as np


def yaw_from_quaternion(qx, qy, qz, qw):
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def write_tum(path, rows):
    """Écrit (t, x, y, cap) au format TUM : t x y z qx qy qz qw."""
    with open(path, 'w') as f:
        for t, x, y, yaw in rows:
            f.write(f'{t:.6f} {x:.6f} {y:.6f} 0 0 0 '
                    f'{math.sin(yaw / 2):.9f} {math.cos(yaw / 2):.9f}\n')


def read_tum(path):
    """Lit un fichier TUM et renvoie un tableau (N, 4) : t, x, y, cap."""
    data = np.loadtxt(path, ndmin=2)
    if data.size == 0:
        return np.zeros((0, 4))
    yaw = [yaw_from_quaternion(*q) for q in data[:, 4:8]]
    return np.column_stack([data[:, 0], data[:, 1], data[:, 2], yaw])


def wrap(angle):
    """Ramène un angle (ou un tableau d'angles) dans ]-pi, pi]."""
    return np.angle(np.exp(1j * np.asarray(angle)))


def interpolate(traj, times):
    """Pose interpolée de traj aux instants times (cap interpolé sans saut de 2 pi)."""
    x = np.interp(times, traj[:, 0], traj[:, 1])
    y = np.interp(times, traj[:, 0], traj[:, 2])
    yaw = wrap(np.interp(times, traj[:, 0], np.unwrap(traj[:, 3])))
    return np.column_stack([times, x, y, yaw])


def associate(estimate, truth):
    """Interpole la vérité terrain aux instants de l'estimation (instants communs seulement)."""
    keep = (estimate[:, 0] >= truth[0, 0]) & (estimate[:, 0] <= truth[-1, 0])
    est = estimate[keep]
    return est, interpolate(truth, est[:, 0])


def transform(traj, theta, tx, ty):
    """Applique la transformation rigide plane (rotation theta, translation tx, ty)."""
    c, s = math.cos(theta), math.sin(theta)
    x = c * traj[:, 1] - s * traj[:, 2] + tx
    y = s * traj[:, 1] + c * traj[:, 2] + ty
    return np.column_stack([traj[:, 0], x, y, wrap(traj[:, 3] + theta)])


def align_umeyama(est, truth):
    """Renvoie la transformation rigide 2D (theta, tx, ty) qui superpose au mieux est sur truth.

    Moindres carrés sur les positions (méthode de Umeyama / Horn, sans facteur d'échelle).
    """
    p, q = est[:, 1:3], truth[:, 1:3]
    mp, mq = p.mean(axis=0), q.mean(axis=0)
    h = (p - mp).T @ (q - mq)
    u, _, vt = np.linalg.svd(h)
    r = vt.T @ u.T
    if np.linalg.det(r) < 0:  # pas de symétrie miroir
        vt[-1, :] *= -1
        r = vt.T @ u.T
    t = mq - r @ mp
    return math.atan2(r[1, 0], r[0, 0]), t[0], t[1]


def align_first_pose(est, truth):
    """Renvoie la transformation qui fait coïncider la première pose de est avec celle de truth."""
    theta = truth[0, 3] - est[0, 3]
    c, s = math.cos(theta), math.sin(theta)
    tx = truth[0, 1] - (c * est[0, 1] - s * est[0, 2])
    ty = truth[0, 2] - (s * est[0, 1] + c * est[0, 2])
    return theta, tx, ty


def position_errors(est, truth):
    return np.hypot(est[:, 1] - truth[:, 1], est[:, 2] - truth[:, 2])


def ate_rmse(est, truth):
    """ATE (RMSE, m) après alignement rigide optimal ; renvoie aussi l'estimation alignée."""
    aligned = transform(est, *align_umeyama(est, truth))
    err = position_errors(aligned, truth)
    return float(np.sqrt(np.mean(err ** 2))), aligned


def travelled_distance(traj):
    steps = np.hypot(np.diff(traj[:, 1]), np.diff(traj[:, 2]))
    return np.concatenate([[0.0], np.cumsum(steps)])


def relative_motion(a, b):
    """Déplacement de la pose a à la pose b, exprimé dans le repère de a : (dx, dy, dcap)."""
    c, s = math.cos(a[3]), math.sin(a[3])
    dx, dy = b[1] - a[1], b[2] - a[2]
    return c * dx + s * dy, -s * dx + c * dy, wrap(b[3] - a[3])


def rpe(est, truth, delta=1.0):
    """RPE sur des segments de `delta` mètres parcourus (vérité terrain).

    Renvoie (RMSE translation en m, RMSE rotation en degrés) sur tous les segments.
    """
    dist = travelled_distance(truth)
    trans, rot = [], []
    j = 0
    for i in range(len(truth)):
        while j < len(truth) and dist[j] - dist[i] < delta:
            j += 1
        if j >= len(truth):
            break
        gt = relative_motion(truth[i], truth[j])
        es = relative_motion(est[i], est[j])
        trans.append(math.hypot(es[0] - gt[0], es[1] - gt[1]))
        rot.append(abs(wrap(es[2] - gt[2])))
    if not trans:
        return float('nan'), float('nan')
    return (float(np.sqrt(np.mean(np.square(trans)))),
            float(np.degrees(np.sqrt(np.mean(np.square(rot))))))
