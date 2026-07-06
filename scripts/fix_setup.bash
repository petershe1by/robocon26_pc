#!/usr/bin/env bash
# fix_setup.bash ¡ª Source this AFTER install/setup.bash
# Fixes: AMENT_PREFIX_PATH, libexec dirs, and entry point stubs
# Usage: source install/setup.bash; source scripts/fix_setup.bash

_COLCON_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"

for _pkg_path in "$_COLCON_WS"/install/*/; do
  _pkg="$(basename "$_pkg_path")"
  [ -d "$_pkg_path" ] || continue
  case "$_pkg" in lib|share|bin|_local_setup_util_sh.py) continue ;; esac

  # Fix 1: Register in AMENT_PREFIX_PATH if missing
  case ":$AMENT_PREFIX_PATH:" in
    *:"$_pkg_path":*) ;;
    *)
      export AMENT_PREFIX_PATH="${_pkg_path}:${AMENT_PREFIX_PATH}"
      echo "[fix_setup] Added $_pkg to AMENT_PREFIX_PATH"
      ;;
  esac

  # Source its local_setup.bash
  if [ -f "$_pkg_path/local_setup.bash" ]; then
    source "$_pkg_path/local_setup.bash"
  fi

  # Fix 2: Ensure libexec directory exists
  if [ ! -d "$_pkg_path/lib/$_pkg" ]; then
    mkdir -p "$_pkg_path/lib/$_pkg"
    # Link from bin/ if available
    if [ -d "$_pkg_path/bin" ]; then
      for _entry in "$_pkg_path"/bin/*; do
        [ -f "$_entry" ] || continue
        _name="$(basename "$_entry")"
        ln -sf "$_entry" "$_pkg_path/lib/$_pkg/$_name"
      done
    fi
    # Create stubs from egg-info if libexec still empty
    if [ -z "$(ls -A "$_pkg_path/lib/$_pkg" 2>/dev/null)" ]; then
      for _edir in \
        "$_COLCON_WS/src/$_pkg"/*.egg-info \
        "$_COLCON_WS/build/$_pkg"/*.egg-info \
        "$_pkg_path"/*.egg-info; do
        [ -d "$_edir" ] || continue
        [ -f "$_edir/entry_points.txt" ] || continue
        grep "=" "$_edir/entry_points.txt" | while IFS="=" read -r _ename _emod; do
          _ename="$(echo "$_ename" | tr -d ' ')"
          _emod="$(echo "$_emod" | tr -d ' ')"
          [ -n "$_ename" ] || continue
          cat > "$_pkg_path/lib/$_pkg/$_ename" << SCRIPTEOF
#!/usr/bin/env python3
from ${_emod%:*} import ${_emod#*:}
import sys
sys.exit(${_emod#*:}())
SCRIPTEOF
          chmod +x "$_pkg_path/lib/$_pkg/$_ename"
          echo "[fix_setup] Stub: $_pkg/lib/$_pkg/$_ename"
        done
        break
      done
    fi
    echo "[fix_setup] Ensured libexec: $_pkg/lib/$_pkg"
  fi
done

unset _pkg _pkg_path _COLCON_WS _entry _name _edir _ename _emod
