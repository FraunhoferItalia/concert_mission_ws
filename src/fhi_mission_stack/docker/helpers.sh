#!/bin/bash
export SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"

function update_image() {
  case $1 in
  remote)
    docker compose -f $SCRIPT_DIR/docker-compose.yml up --build --pull always -d
    docker push fhi-git.fraunhofer.it:5050/rise-libs/ro/mi/fhi_mission_stack:devel
    docker compose -f $SCRIPT_DIR/docker-compose-release.yml up --build --pull always -d
    docker push fhi-git.fraunhofer.it:5050/rise-libs/ro/mi/fhi_mission_stack:release
    ;;
  **)
    docker compose -f $SCRIPT_DIR/docker-compose.yml up --build --pull always
    docker compose -f $SCRIPT_DIR/docker-compose-release.yml up --build --pull always
    ;;
  esac
}
