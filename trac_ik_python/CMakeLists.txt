cmake_minimum_required(VERSION 3.8)
project(trac_ik_python)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# always keep pybind at first. error is raised if order is changed
find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(trac_ik_lib REQUIRED)

# build the python extension module (.so)
pybind11_add_module(trac_ik_python_module src/trac_ik_bindings.cpp)

target_link_libraries(trac_ik_python_module PRIVATE
  Eigen3::Eigen
  orocos-kdl
  kdl_parser::kdl_parser
  trac_ik_lib::trac_ik_lib)

# install the pure-python package (provides import path)
ament_python_install_package(${PROJECT_NAME})

# install the compiled extension into the python package directory
# ${PYTHON_INSTALL_DIR} is provided by ament_cmake_python
install(TARGETS trac_ik_python_module
  LIBRARY DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME})

# install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

# install the script so `ros2 run` can execute it
install(PROGRAMS scripts/test_ik_node.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
