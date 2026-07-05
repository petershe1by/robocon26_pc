#!/usr/bin/env bash
# fix_setup.bash ¡ª Source this AFTER install/setup.bash to register missing packages
# Usage: source install/setup.bash; source scripts/fix_setup.bash

_COLCON_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"

for _pkg in robocom_bringup robocom_navigation robocom_motion_control; do
  _pkg_path="${_COLCON_WS}/install/${_pkg}"
  if [ -d "$_pkg_path" ]; then
    # Check if already in AMENT_PREFIX_PATH
    case ":$AMENT_PREFIX_PATH:" in
      *:"$_pkg_path":*) ;;
      *)
        export AMENT_PREFIX_PATH="${_pkg_path}:${AMENT_PREFIX_PATH}"
        echo "[fix_setup] Added $_pkg to AMENT_PREFIX_PATH"
        ;;
    esac
    # Source its local_setup.bash if exists
    if [ -f "$_pkg_path/local_setup.bash" ]; then
      source "$_pkg_path/local_setup.bash"
    fi
  fi
done

unset _pkg _pkg_path _COLCON_WS
