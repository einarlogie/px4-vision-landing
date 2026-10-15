#!/bin/sh
# Start PX4 SITL + Gazebo for this project.
#
# Replaces the image's own entrypoint for two reasons:
#
# 1. MAVLink routing. The stock entrypoint points *every* MAVLink link at
#    host.docker.internal on Docker Desktop. That's right for QGroundControl
#    on the Windows host but wrong for our MAVSDK code, which runs in a
#    container sharing this network namespace. So only the GCS link (UDP
#    14550) goes to the host; the API/offboard link (UDP 14540) stays local.
#
# 2. Rendering. Under WSL, reading camera images back from the GPU (Mesa
#    D3D12 driver) is far slower than rendering them on the CPU, so the
#    Gazebo server, which renders the drone's camera, uses llvmpipe. The GUI
#    only draws to the screen, so it can use the GPU. PX4's launcher starts
#    both from one environment, so we run PX4 headless and start the GUI
#    ourselves.
set -e

PX4_PREFIX=/opt/px4-gazebo
RC_MAVLINK="$PX4_PREFIX/etc/init.d-posix/px4-rc.mavlink"

HOST_IP=$(getent ahostsv4 host.docker.internal 2>/dev/null | awk '/STREAM/ {print $1; exit}')
if [ -n "$HOST_IP" ] && ! grep -q "mavlink start -x -t" "$RC_MAVLINK"; then
    sed -i "s/mavlink start -x -u \$udp_gcs_port_local/mavlink start -x -t $HOST_IP -u \$udp_gcs_port_local/" "$RC_MAVLINK"
    echo "[sim-entrypoint] GCS MAVLink -> $HOST_IP:14550 (QGroundControl on the host)"
fi

if [ "${GUI:-1}" = "1" ] && [ -n "$DISPLAY$WAYLAND_DISPLAY" ]; then
    echo "[sim-entrypoint] starting Gazebo GUI (GALLIUM_DRIVER=${GUI_RENDER_DRIVER:-d3d12})"
    GALLIUM_DRIVER="${GUI_RENDER_DRIVER:-d3d12}" gz sim -g >/tmp/gz-gui.log 2>&1 &
fi

export HEADLESS=1
export GALLIUM_DRIVER="${SENSOR_RENDER_DRIVER:-llvmpipe}"
echo "[sim-entrypoint] Gazebo server rendering with GALLIUM_DRIVER=$GALLIUM_DRIVER"
exec "$PX4_PREFIX/bin/px4" "$@"
