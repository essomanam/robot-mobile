#!/usr/bin/env bash
# Mesure de la détection de la cible (étape E3) : fréquence, taux de détection, latence.
#
# Usage : run_perception_check.sh <dossier_résultats>
#
# Robot immobile dans deux poses : cible en vue à 1,1 m (en_vue) et cible dans le dos (hors_vue).

set -euo pipefail

out=${1:?"usage : $0 <dossier_résultats>"}
export ROS_DOMAIN_ID=${EVAL_ROS_DOMAIN_ID:-46}
export GZ_PARTITION=${EVAL_GZ_PARTITION:-as52_perception}
config=$(ros2 pkg prefix robot_mobile_bringup)/share/robot_mobile_bringup/config

poses=(
  "en_vue   3.0 -0.3 3.1416"
  "hors_vue 1.7 -2.0 -1.5708"
)

mkdir -p "$out"
for line in "${poses[@]}"; do
  read -r name x y yaw <<< "$line"
  if [ -f "$out/$name.json" ]; then
    echo "déjà fait : $name"
    continue
  fi
  echo "=== $name ($(date +%H:%M:%S))"
  # setsid : chaque lancement dans son propre groupe de processus, arrêté d'un bloc ensuite
  # (une tâche de fond d'un script ignore Ctrl+C / SIGINT)
  setsid ros2 launch robot_mobile_bringup simulation.launch.py gui:=false x:="$x" y:="$y" \
    yaw:="$yaw" > "$out/$name.sim.log" 2>&1 &
  sim=$!
  setsid ros2 run robot_mobile_perception target_detector --ros-args \
    --params-file "$config/perception/target_detector.yaml" > "$out/$name.detector.log" 2>&1 &
  detector=$!
  sleep 15   # démarrage de la simulation
  ros2 run robot_mobile_evaluation perception_check --ros-args -p use_sim_time:=true \
    -p duration:=30.0 -p output:="$out/$name.json" 2>&1 | tee "$out/$name.log"
  kill -TERM -- -"$detector" -"$sim" 2>/dev/null || true
  sleep 5
  kill -KILL -- -"$detector" -"$sim" 2>/dev/null || true
  for pid in $(pgrep -x gz-sim-main || true); do
    if tr '\0' '\n' < "/proc/$pid/environ" 2>/dev/null | grep -q "GZ_PARTITION=$GZ_PARTITION"; then
      kill -9 "$pid" || true
    fi
  done
done
