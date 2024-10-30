cmake_minimum_required(VERSION 3.0.2)
project(ur5_kdl_control)

## 以 C++11 进行编译，支持 ROS Kinetic 及更新的版本
# add_compile_options(-std=c++11)

## 查找 catkin 的宏和库
## 如果使用了类似 find_package(catkin REQUIRED COMPONENTS xyz) 的 COMPONENTS 列表，也会查找其他 catkin 包
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  moveit_core
  moveit_ros_planning_interface
  roscpp
  rospy
  tf
  tf_conversions
  interactive_markers
  trac_ik_lib
  geometry_msgs
  actionlib
  trajectory_msgs
  control_msgs
)

## 系统依赖项按照 CMake 的惯例找到
# find_package(Boost REQUIRED COMPONENTS system)


## 如果该包有 setup.py，则取消注释此行。此宏可确保其中声明的模块和全局脚本被安装
## 参见 http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## 声明 ROS 消息、服务和动作 ##
################################################

## 要在此包中声明和构建消息、服务或动作，请按照以下步骤：
## * 设 MSG_DEP_SET 为您在消息/服务/动作中使用的消息类型所在的包集合
##   （例如 std_msgs、actionlib_msgs 等）。
## * 在文件 package.xml 中：
##   * 添加一个针对 "message_generation" 的 build_depend 标签
##   * 对于 MSG_DEP_SET 中的每个包，添加一个 build_depend 和一个 exec_depend 标签
##   * 如果 MSG_DEP_SET 非空，以下依赖项已被引入，但仍可明确声明：
##     * 添加一个针对 "message_runtime" 的 exec_depend 标签
## * 在此文件（CMakeLists.txt）中：
##   * 在 find_package(catkin REQUIRED COMPONENTS ...) 中添加 "message_generation" 和 MSG_DEP_SET 中的每个包
##   * 在 catkin_package(CATKIN_DEPENDS ...) 中添加 "message_runtime" 和 MSG_DEP_SET 中的每个包
##   * 根据需要取消注释下面的 add_*_files 部分
##     并列出每个要处理的 .msg/.srv/.action 文件
##   * 取消注释下面的 generate_messages 条目
##   * 将 MSG_DEP_SET 中的每个包添加到 generate_messages(DEPENDENCIES ...) 中

## 在 'msg' 文件夹中生成消息
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## 在 'srv' 文件夹中生成服务
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## 在 'action' 文件夹中生成动作
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## 根据这里列出的任何依赖项生成已添加的消息和服务
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # 或包含消息的其他包
# )

################################################
## 声明 ROS 动态重配置参数 ##
################################################

## 要在此包中声明和构建动态重配置参数，请按照以下步骤：
## * 在文件 package.xml 中：
##   * 添加针对 "dynamic_reconfigure" 的 build_depend 和 exec_depend 标签
## * 在此文件（CMakeLists.txt）中：
##   * 在 find_package(catkin REQUIRED COMPONENTS ...) 中添加 "dynamic_reconfigure"
##   * 取消注释下面的 "generate_dynamic_reconfigure_options" 部分
##     并列出每个要处理的 .cfg 文件

## 在 'cfg' 文件夹中生成动态重配置参数
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin 特定的配置 ##
###################################
## catkin_package 宏为您的包生成 cmake 配置文件
## 声明要传递给依赖项目的内容
## INCLUDE_DIRS：如果您的包包含头文件，请取消注释此行
## LIBRARIES：您在此项目中创建的、依赖项目也需要的库
## CATKIN_DEPENDS：依赖项目也需要的 catkin 包
## DEPENDS：依赖项目也需要的此项目的系统依赖项
include_directories(/usr/local/include)

catkin_package(
   INCLUDE_DIRS include /usr/local/include
#  LIBRARIES ur5_kdl_control
#  CATKIN_DEPENDS gazebo_ros moveit_core moveit_ros_planning_interface roscpp rospy
#  DEPENDS system_lib
)

###########
## 构建 ##
###########

## 指定头文件的其他位置
## 您的包位置应列在其他位置之前
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## 声明一个 C++ 库
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/ur5_kdl_control.cpp
# )

## 添加库的 cmake 目标依赖项
## 例如，代码可能需要在库之前生成
## 可能来自消息生成或动态重配置
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## 声明一个 C++ 可执行文件
## 使用 catkin_make，所有包都在单一的 CMake 上下文中构建
## 推荐的前缀可确保跨包的目标名称不会冲突
# add_executable(${PROJECT_NAME}_node src/ur5_kdl_control_node.cpp)
add_executable(ur5_kdl_ik_solver 
  src/ur5_kdl_ik_solver.cpp
  src/ur5_ik_solver.cpp
)
target_link_libraries(ur5_kdl_ik_solver ${catkin_LIBRARIES})
add_executable(dual_ur5_ik_solver 
  src/dual_ur5_ik_solver.cpp
  src/ur5_ik_solver.cpp
)
target_link_libraries(dual_ur5_ik_solver ${catkin_LIBRARIES})

add_executable(my_tf_listener 
  src/my_tf_listener.cpp
)
target_link_libraries(my_tf_listener ${catkin_LIBRARIES})


add_executable(move_ur5_arms_to_zero src/move_ur5_arms_to_zero.cpp)

target_link_libraries(move_ur5_arms_to_zero
  ${catkin_LIBRARIES}
)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
add_executable(ik_test 
  src/ik_test.cpp
  src/ur5_dh_params.cpp
  src/math_utils.cpp
  src/inverse_kinematics.cpp
  src/forward_kinematics.cpp
)
target_link_libraries(ur5_kdl_ik_solver ${catkin_LIBRARIES})

## 重命名无前缀的 C++ 可执行文件
## 上述推荐的前缀会导致目标名称过长，以下将目标重命名回较短的版本，便于用户使用
## 例如，使用 "rosrun someones_pkg node" 而不是 "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## 添加可执行文件的 cmake 目标依赖项
## 与上面的库相同
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## 指定要链接库或可执行目标的库
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## 安装 ##
#############

# 所有的安装目标都应使用 catkin DESTINATION 变量
# 参见 http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## 标记可执行脚本（如 Python 等）以进行安装
## 与 setup.py 不同，您可以选择目标位置
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## 标记可执行文件以进行安装
## 参见 http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## 标记库以进行安装
## 参见 http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## 标记要安装的 C++ 头文件
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## 标记要安装的其他文件（例如 launch 和 bag 文件等）
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## 测试 ##
#############

## 添加基于 gtest 的 C++ 测试目标并链接库
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ur5_kdl_control.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## 添加要由 Python nosetests 运行的文件夹
# catkin_add_nosetests(test)
