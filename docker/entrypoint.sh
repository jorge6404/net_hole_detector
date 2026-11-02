#!/bin/bash
# Carga ROS + el paquete y lanza el detector. Los argumentos del launch salen de variables
# de entorno (vacías = lo que diga el YAML). Con argumentos, ejecuta ese comando en su lugar.
set -e

if [ -d /opt/gpu_libs ]; then
  gpu_dirs=$(find /opt/gpu_libs -name 'lib*.so*' -printf '%h\n' | sort -u | paste -sd:)
  export LD_LIBRARY_PATH="${gpu_dirs}${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source /opt/nhd_ws/setup.bash

if [ $# -gt 0 ]; then
  exec "$@"
fi

# La imagen de CPU arranca directamente en cpu (sin los dos WARN de la cadena)
if [ -z "$ENGINE" ] && [ "$NHD_ENGINE_DEFAULT" = cpu ]; then
  ENGINE=cpu
fi

args=()
add() { if [ -n "${!1}" ]; then args+=("$2:=${!1}"); fi; }
add NAMESPACE namespace
add PARAMS_FILE params_file
add IMAGE_TOPIC image_topic
add CAMERA_INFO_TOPIC camera_info_topic
add USE_SIM_TIME use_sim_time
add ENGINE engine
add MODEL_PATH model_path
add IMAGE_TRANSPORT image_transport
add IS_RECTIFIED is_rectified

echo "ros2 launch net_hole_detector_rust net_hole_detector.launch.py ${args[*]}"
exec ros2 launch net_hole_detector_rust net_hole_detector.launch.py "${args[@]}"
