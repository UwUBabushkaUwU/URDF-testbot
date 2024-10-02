cmake_minimum_required(VERSION 3.0.2)
project(keyboard)

find_package(catkin REQUIRED)

catkin_package()

catkin_python_setup()

install(PROGRAMS src/keyboard/keyboard.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
