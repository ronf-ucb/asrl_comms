#!/usr/bin/env sh
# generated from catkin/cmake/templates/env.sh.in

if [ $# -eq 0 ] ; then
  /bin/echo "Entering environment at '/home/linaro/fearing_rosbuild_overlay/asrl_comms/serial/build/catkin_generated', type 'exit' to leave"
  . "/home/linaro/fearing_rosbuild_overlay/asrl_comms/serial/build/catkin_generated/setup_cached.sh"
  "$SHELL" -i
  /bin/echo "Exiting environment at '/home/linaro/fearing_rosbuild_overlay/asrl_comms/serial/build/catkin_generated'"
else
  . "/home/linaro/fearing_rosbuild_overlay/asrl_comms/serial/build/catkin_generated/setup_cached.sh"
  exec "$@"
fi
