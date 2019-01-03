cmake_minimum_required(VERSION 2.8.3)
project(borunte_hal)

find_package(catkin REQUIRED)

catkin_package()

# TODO: Machinekit config should be installed
catkin_install_python(
  PROGRAMS
        components/i620p_modbus.py
        components/lamp_control.py
        components/robotiq_modbus.py

  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
