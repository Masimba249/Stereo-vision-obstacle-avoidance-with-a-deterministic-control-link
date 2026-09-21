#!/usr/bin/env bash
# Bring up a virtual CAN interface so the full stack can be exercised without
# CAN hardware, using the real socketcan path rather than the loopback shim.
#
#   sudo ./pi/tools/vcan_up.sh vcan0
#
# On the real Pi with an MCP2515 HAT, use a real interface instead:
#   sudo ip link set can0 up type can bitrate 500000 restart-ms 100
set -euo pipefail

IFACE="${1:-vcan0}"

if ! lsmod | grep -q '^vcan'; then
    echo "loading the vcan kernel module"
    modprobe vcan
fi

if ip link show "$IFACE" >/dev/null 2>&1; then
    echo "$IFACE already exists"
else
    ip link add dev "$IFACE" type vcan
    echo "created $IFACE"
fi

ip link set up "$IFACE"
echo "$IFACE is up"
ip -details -brief link show "$IFACE"

cat <<'EOF'

Next:
  # terminal 1 - the simulated drive node
  python3 pi/tools/esp32_sim.py --interface socketcan --channel vcan0

  # terminal 2 - the vision pipeline
  python3 -m stereolink.main --source synthetic \
      --can-interface socketcan --can-channel vcan0 --no-mqtt

  # terminal 3 - watch the bus (from can-utils)
  candump -tz vcan0
EOF
