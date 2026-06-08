#!/usr/bin/env bash
set -euo pipefail

cat <<'EOF'
This configures Docker for slow/unstable networks:

  - max-concurrent-downloads: 1
  - max-concurrent-uploads: 1

It writes /etc/docker/daemon.json and restarts Docker.
EOF

tmp_file="$(mktemp)"
if [ -f /etc/docker/daemon.json ]; then
  sudo cp /etc/docker/daemon.json "/etc/docker/daemon.json.backup.$(date +%Y%m%d%H%M%S)"
fi

cat > "${tmp_file}" <<'EOF'
{
  "max-concurrent-downloads": 1,
  "max-concurrent-uploads": 1
}
EOF

sudo mkdir -p /etc/docker
sudo cp "${tmp_file}" /etc/docker/daemon.json
rm -f "${tmp_file}"
sudo systemctl restart docker

cat <<'EOF'

Docker restarted in slow-network mode.

Now retry:

  docker pull dockerproxy.net/library/ros:galactic-ros-base

or:

  AGV_GALACTIC_BASE_IMAGE=dockerproxy.net/library/ros:galactic-ros-base ./galactic/tools/galactic_docker.sh build
EOF
