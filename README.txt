Requirements:
--------------
 - PCL 1.7.0 at least rev8129 and dependencies (pointclouds.org)
 - RGBDemo and dependencies (http://labs.manctl.com/rgbdemo/index.php/Main/HomePage)
 - CMake Buildsystem (cmake.org)

Get rgbdemo:
-------------
# git submodule init
# git submodule update --recursive

Build:
--------

# mkdir build
# cd build/
# cmake ../src (... better twice for subsequent deps)
# cmake ../src
# make

