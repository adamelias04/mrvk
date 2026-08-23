#!/usr/bin/env bash
# TEB local planner is NOT available as an apt binary for Humble, so it is built
# from source. Needs: sudo apt install ros-humble-libg2o
# Run this from the workspace `src/` dir, then rebuild the workspace with colcon.
set -e
chmod +x "$0" 2>/dev/null || true

cd "$(dirname "$0")"

# teb_local_planner: use the humble-devel branch (ros2-master targets a newer
# Nav2 and fails on Humble: nav2_core/controller_exceptions.hpp not found).
if [ ! -d teb_local_planner ]; then
  git clone -b humble-devel https://github.com/rst-tu-dortmund/teb_local_planner.git
fi

# costmap_converter (TEB dependency).
if [ ! -d costmap_converter ]; then
  git clone -b ros2 https://github.com/rst-tu-dortmund/costmap_converter.git
  # The installed cv_bridge (3.2.1, Humble) ships cv_bridge.h, not .hpp.
  grep -rl 'cv_bridge/cv_bridge.hpp' costmap_converter/ 2>/dev/null | \
    xargs -r sed -i 's#cv_bridge/cv_bridge\.hpp#cv_bridge/cv_bridge.h#g'
fi

echo "Cloned TEB sources. Now rebuild:"
echo "  cd /home/adam/ws && colcon build --symlink-install"
