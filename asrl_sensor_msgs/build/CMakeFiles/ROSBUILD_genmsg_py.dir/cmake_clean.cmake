FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/asrl_sensor_msgs/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/asrl_sensor_msgs/msg/__init__.py"
  "../src/asrl_sensor_msgs/msg/_SunSensorData.py"
  "../src/asrl_sensor_msgs/msg/_SerialData.py"
  "../src/asrl_sensor_msgs/msg/_IpAddress.py"
  "../src/asrl_sensor_msgs/msg/_Packet.py"
  "../src/asrl_sensor_msgs/msg/_UtmInfo.py"
  "../src/asrl_sensor_msgs/msg/_InclinometerData.py"
  "../src/asrl_sensor_msgs/msg/_TcpPacket.py"
  "../src/asrl_sensor_msgs/msg/_GgaInfo.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
