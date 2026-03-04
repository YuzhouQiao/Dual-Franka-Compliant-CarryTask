cmake_minimum_required(VERSION 3.8)
project(dual_arm_control)

# 1. 基础编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 2. 寻找依赖库
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
# 如果你装好了 franka_msgs 就保留下面这行，没装好就注释掉
# find_package(franka_msgs REQUIRED) 

# 3. 包含目录
include_directories(include)

# 4. 编译可执行文件：双臂运动测试节点
add_executable(dual_arm_move_test src/dual_arm_move_test.cpp)
ament_target_dependencies(dual_arm_move_test
  rclcpp
  moveit_ros_planning_interface
  moveit_visual_tools
)

# 5. 安装规则
install(TARGETS
  dual_arm_move_test
  DESTINATION lib/${PROJECT_NAME}
)
# 安装 launch 文件夹
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
