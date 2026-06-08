#!/usr/bin/env bash
set -euo pipefail

cat <<'EOF'
This temporarily disables IPv6 for the current boot and restarts Docker.

Why:
  Docker Hub is resolving to an unreachable IPv6 address on this network.

Effect:
  - net.ipv6.conf.all.disable_ipv6=1
  - net.ipv6.conf.default.disable_ipv6=1
  - restart docker.service

The sysctl changes are not persisted across reboot.
EOF

sudo sysctl -w net.ipv6.conf.all.disable_ipv6=1
sudo sysctl -w net.ipv6.conf.default.disable_ipv6=1
sudo systemctl restart docker

cat <<'EOF'

Docker restarted. Now retry:

  ./galactic/tools/galactic_docker.sh build

Run shell only after build succeeds:

  ./galactic/tools/galactic_docker.sh shell
EOF
