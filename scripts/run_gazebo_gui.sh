#!/usr/bin/env bash
set -eo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORLD_FILE="${1:-$PROJECT_ROOT/worlds/empty_no_shadows.sdf}"

source /opt/ros/jazzy/setup.bash
export MESA_GL_VERSION_OVERRIDE=3.3COMPAT

exec gz sim -v 3 --render-engine ogre "$WORLD_FILE"
