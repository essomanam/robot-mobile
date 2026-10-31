#!/usr/bin/env bash
# Essais de navigation avec obstacles non cartographiés (étape E4).
#
# Usage : run_navigation_trials.sh <dossier_résultats> <carte RTAB-Map .db> [répétitions]
#
# Monde worlds/as52_room_obstacles.sdf ; carte construite SANS ces obstacles (E2, deux sens).
#   carton : couloir sud, (-2.5, -1.5) -> (2.5, -1.5), carton de 40 cm sur le chemin
#   bas    : couloir nord, (2.5, 1.5) -> (-2.5, 1.5), obstacle de 8 cm (sous le plan du LiDAR)
# Chaque scénario est joué avec la caméra de profondeur (profondeur) et sans (lidar_seul).

set -euo pipefail

out=${1:?"usage : $0 <dossier_résultats> <carte.db> [répétitions]"}
map=${2:?"usage : $0 <dossier_résultats> <carte.db> [répétitions]"}
reps=${3:-2}
export ROS_DOMAIN_ID=${EVAL_ROS_DOMAIN_ID:-45}
export GZ_PARTITION=${EVAL_GZ_PARTITION:-as52_nav}

scenarios=(
  "carton -2.5 -1.5 0.0     [2.5,-1.5,0.0]"
  "bas     2.5  1.5 3.1416  [-2.5,1.5,3.1416]"
)

mkdir -p "$out"
for line in "${scenarios[@]}"; do
  read -r name x y yaw goal <<< "$line"
  for depth in true false; do
    variant=$([ "$depth" = true ] && echo profondeur || echo lidar_seul)
    for i in $(seq 1 "$reps"); do
      run="${name}_${variant}_${i}"
      if [ -f "$out/$run/result.json" ]; then
        echo "déjà fait : $run"
        continue
      fi
      echo "=== $run ($(date +%H:%M:%S))"
      timeout 600 ros2 launch robot_mobile_bringup navigation_trial.launch.py scenario:="$run" \
        map_database:="$map" x:="$x" y:="$y" yaw:="$yaw" goal:="$goal" use_depth:="$depth" \
        output:="$out/$run" > "$out/$run.log" 2>&1 || true
      rm -f "$out/$run/rtabmap.db"   # copie de travail de la carte (plusieurs centaines de Mo)
      for pid in $(pgrep -x gz-sim-main || true); do
        if tr '\0' '\n' < "/proc/$pid/environ" 2>/dev/null | grep -q "GZ_PARTITION=$GZ_PARTITION"; then
          kill -9 "$pid" || true
        fi
      done
    done
  done
done

ros2 run robot_mobile_evaluation evaluate_navigation_trials "$out" "$out/figures"
