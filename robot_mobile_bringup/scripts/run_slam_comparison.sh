#!/usr/bin/env bash
# Rejoue le jeu de données de référence avec chaque SLAM, puis compare les résultats (étape E2).
#
# Usage : run_slam_comparison.sh <dossier_résultats> [rosbag]
#
# Un sous-dossier par essai ; un essai déjà terminé (stats.json présent) n'est pas refait.
# Les configurations RTAB-Map B et D sont répétées 3 fois pour mesurer la variabilité.
# Les essais tournent dans un domaine ROS dédié pour ne pas capter une simulation en cours.

set -euo pipefail

out=${1:?"usage : $0 <dossier_résultats> [rosbag]"}
bag=${2:-$HOME/as52_data/rosbags/E1_reference_20261024}
export ROS_DOMAIN_ID=${EVAL_ROS_DOMAIN_ID:-42}

runs=(rtabmap_rgbd_1 rtabmap_rgbd_2 rtabmap_rgbd_3 rtabmap_rgbd_vo_1
      rtabmap_rgbd_scan_1 rtabmap_rgbd_scan_2 rtabmap_rgbd_scan_3 slam_toolbox_1)

mkdir -p "$out"
for run in "${runs[@]}"; do
  if [ -f "$out/$run/stats.json" ]; then
    echo "déjà fait : $run"
    continue
  fi
  method=${run%_*}
  echo "=== $run ($(date +%H:%M:%S))"
  ros2 launch robot_mobile_bringup slam_evaluation.launch.py \
    method:="$method" bag:="$bag" output:="$out/$run" > "$out/$run.log" 2>&1 || true
done

ros2 run robot_mobile_evaluation evaluate_slam "$out" "$out/figures"
