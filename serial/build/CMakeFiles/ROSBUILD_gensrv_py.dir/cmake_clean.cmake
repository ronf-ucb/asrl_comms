FILE(REMOVE_RECURSE
  "../srv_gen"
  "../src/serial/srv"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/serial/srv/__init__.py"
  "../src/serial/srv/_Baud.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)