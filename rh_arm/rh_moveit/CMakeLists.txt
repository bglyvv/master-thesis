cmake_minimum_required(VERSION 3.1.3)
project(rh_moveit)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
catkin_install_python(PROGRAMS script/ir_sensor_check.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

