cmake_minimum_required(VERSION 3.0.2)
project(ur_gazebo)

# 查找 catkin 和其他必要的组件
find_package(catkin REQUIRED
  roscpp
  std_msgs
  actionlib
  )

# 查找 Pinocchio 库
find_package(pinocchio REQUIRED)

# 查找 Eigen 库（如果不是自动的话）
find_package(Eigen3 REQUIRED NO_MODULE)

find_package(orocos_kdl REQUIRED)

find_package(fcl REQUIRED)

find_package(urdf REQUIRED)

find_package(kdl_parser REQUIRED)

# 包含目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PINOCCHIO_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR} # 添加 Eigen 的头文件路径
  ${orocos_kdl_INCLUDE_DIRS}
  ${FCL_INCLUDE_DIRS}
  ${kdl_parser_INCLUDE_DIRS}
)

catkin_package()

# add_executable(effort_pub src/effort_pub.cpp)
# target_link_libraries(effort_pub ${catkin_LIBRARIES})

# add_executable(te_tw src/te_tw.cpp)
# # 确保将 Pinocchio 库链接到 te_tw 目标
# target_link_libraries(te_tw ${catkin_LIBRARIES} ${PINOCCHIO_LIBRARIES})

# add_executable(effort_gazebo src/effort_gazebo.cpp)
# # 确保将 Pinocchio 库链接到 effort_gazebo 目标
# target_link_libraries(effort_gazebo ${catkin_LIBRARIES} ${PINOCCHIO_LIBRARIES})

# add_executable(FK src/FK.cpp)
# # 确保将 Pinocchio 库链接到 effort_gazebo 目标
# target_link_libraries(FK ${catkin_LIBRARIES} ${PINOCCHIO_LIBRARIES})

# add_executable(ik src/ik.cpp)
# # 确保将 Pinocchio 库链接到 effort_gazebo 目标
# target_link_libraries(ik ${catkin_LIBRARIES} ${PINOCCHIO_LIBRARIES})

# add_executable(wrench_generate src/wrench_generate.cpp)
# # 确保将 Pinocchio 库链接到 effort_gazebo 目标
# target_link_libraries(wrench_generate ${catkin_LIBRARIES} ${PINOCCHIO_LIBRARIES})

# add_executable(back_to_start src/back_to_start.cpp)
# # 确保将 Pinocchio 库链接到 effort_gazebo 目标
# target_link_libraries(back_to_start ${catkin_LIBRARIES} ${PINOCCHIO_LIBRARIES} ${actionlib_LIBRARIES})

# add_executable(rrt src/rrt.cpp)
# # 确保将 Pinocchio 库链接到 effort_gazebo 目标
# target_link_libraries(rrt ${catkin_LIBRARIES} ${PINOCCHIO_LIBRARIES} ${actionlib_LIBRARIES} ${orocos_kdl_LIBRARIES} ${FCL_LIBRARIES} ${kdl_parser_LIBRARIES} ${urdf_LIBRARIES} ) 

add_executable(path_planning src/path_planning.cpp src/rrt.cpp)
# 确保将 Pinocchio 库链接到 effort_gazebo 目标
target_link_libraries(path_planning ${catkin_LIBRARIES} ${PINOCCHIO_LIBRARIES} ${actionlib_LIBRARIES} ${orocos_kdl_LIBRARIES} ${FCL_LIBRARIES} ${kdl_parser_LIBRARIES} ${urdf_LIBRARIES})

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(tests/roslaunch_test_ur3.xml)
  roslaunch_add_file_check(tests/roslaunch_test_ur3e.xml)
  roslaunch_add_file_check(tests/roslaunch_test_ur5.xml)
  roslaunch_add_file_check(tests/roslaunch_test_ur5e.xml)
  roslaunch_add_file_check(tests/roslaunch_test_ur10.xml)
  roslaunch_add_file_check(tests/roslaunch_test_ur10e.xml)
  roslaunch_add_file_check(tests/roslaunch_test_ur16e.xml)
endif()

install(DIRECTORY config launch urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

