#!/usr/bin/env bash
# Carte de coûts de Nav2 face à l'obstacle bas (étape E4), avec et sans caméra de profondeur.
#
# Usage : run_costmap_snapshots.sh <dossier_résultats> <carte RTAB-Map .db>
#
# Robot immobile au départ du scénario « bas » (2,5 ; 1,5), face à l'obstacle de 8 cm (à 2,2 m).

set -euo pipefail

out=${1:?"usage : $0 <dossier_résultats> <carte.db>"}
map=${2:?"usage : $0 <dossier_résultats> <carte.db>"}
export ROS_DOMAIN_ID=${EVAL_ROS_DOMAIN_ID:-47}
export GZ_PARTITION=${EVAL_GZ_PARTITION:-as52_costmap}
world=$(ros2 pkg prefix robot_mobile_bringup)/share/robot_mobile_bringup/worlds/as52_room_obstacles.sdf
# pose de départ dans le repère de la carte (origine de la carte : (-2.500876, -1.500003) monde)
initial="5.0009 3.0000 0 0 0 3.1416"

mkdir -p "$out"
for depth in true false; do
  variant=$([ "$depth" = true ] && echo profondeur || echo lidar_seul)
  title=$([ "$depth" = true ] && echo "LiDAR + profondeur" || echo "LiDAR seul")
  if [ -f "$out/E4_carte_de_couts_$variant.png" ]; then
    echo "déjà fait : $variant"
    continue
  fi
  echo "=== $variant ($(date +%H:%M:%S))"
  cp "$map" "$out/rtabmap.db"   # la localisation modifie la carte : on travaille sur une copie
  # setsid : chaque lancement dans son propre groupe de processus, arrêté d'un bloc ensuite
  setsid ros2 launch robot_mobile_bringup simulation.launch.py world:="$world" gui:=false \
    x:=2.5 y:=1.5 yaw:=3.1416 > "$out/$variant.sim.log" 2>&1 &
  sim=$!
  sleep 8
  setsid ros2 launch robot_mobile_bringup navigation.launch.py map_database:="$out/rtabmap.db" \
    use_depth:="$depth" initial_pose:="$initial" > "$out/$variant.nav.log" 2>&1 &
  nav=$!
  timeout 180 ros2 run robot_mobile_evaluation costmap_snapshot --ros-args -p use_sim_time:=true \
    -p delay:=30.0 -p output:="$out/E4_carte_de_couts_$variant.png" -p "zoom:=[-1.5, 3.2, 0.2, 2.6]" \
    -p title:="Carte de coûts globale de Nav2 — $title" || true
  kill -TERM -- -"$nav" -"$sim" 2>/dev/null || true
  sleep 5
  kill -KILL -- -"$nav" -"$sim" 2>/dev/null || true
  for pid in $(pgrep -x gz-sim-main || true); do
    if tr '\0' '\n' < "/proc/$pid/environ" 2>/dev/null | grep -q "GZ_PARTITION=$GZ_PARTITION"; then
      kill -9 "$pid" || true
    fi
  done
  rm -f "$out/rtabmap.db"
done
