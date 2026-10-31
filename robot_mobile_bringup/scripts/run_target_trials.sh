#!/usr/bin/env bash
# Essais d'approche de la cible rouge depuis plusieurs positions de départ (étape E3).
#
# Usage : run_target_trials.sh <dossier_résultats>
#
# Scénarios (cible en (1.7, -0.3)) :
#   face_proche  cible droit devant à ~1,1 m
#   face_loin    cible droit devant à ~2,1 m (départ au nord)
#   laterale     cible hors du champ, à ~60° à gauche : recherche courte
#   arriere_cote cible juste hors du champ, à ~40° à gauche
#   dos          cible dans le dos : recherche d'un demi-tour
#   obstacle     pilier B entre le robot et la cible : limite attendue de l'approche directe
# Chaque essai est isolé (domaine ROS et partition Gazebo dédiés) ; un essai déjà fait est sauté.

set -euo pipefail

out=${1:?"usage : $0 <dossier_résultats>"}
export ROS_DOMAIN_ID=${EVAL_ROS_DOMAIN_ID:-44}
export GZ_PARTITION=${EVAL_GZ_PARTITION:-as52_trials}

scenarios=(
  "face_proche   3.0  -0.3  3.1416"
  "face_loin     1.7   2.0 -1.5708"
  "laterale      2.9   1.8  3.1416"
  "arriere_cote  2.9  -1.8  1.5708"
  "dos           1.7  -2.0 -1.5708"
  "obstacle     -2.5  -1.5  0.2783"
)

mkdir -p "$out"
for line in "${scenarios[@]}"; do
  read -r name x y yaw <<< "$line"
  if [ -f "$out/$name/result.json" ]; then
    echo "déjà fait : $name"
    continue
  fi
  echo "=== $name ($(date +%H:%M:%S))"
  ros2 launch robot_mobile_bringup target_trial.launch.py scenario:="$name" \
    x:="$x" y:="$y" yaw:="$yaw" output:="$out/$name" > "$out/$name.log" 2>&1 || true
  # serveur Gazebo de cet essai resté actif (voir journal E1) : on ne touche qu'à notre partition
  for pid in $(pgrep -x gz-sim-main || true); do
    if tr '\0' '\n' < "/proc/$pid/environ" 2>/dev/null | grep -q "GZ_PARTITION=$GZ_PARTITION"; then
      kill -9 "$pid" || true
    fi
  done
done

ros2 run robot_mobile_evaluation evaluate_target_trials "$out" "$out/figures"
