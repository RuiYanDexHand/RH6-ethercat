cmake_minimum_required(VERSION 3.5)
project(rh6_ctrl)

# 输出系统架构信息（用于调试ARM平台问题）
message(STATUS "CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "CMAKE_SYSTEM_NAME: ${CMAKE_SYSTEM_NAME}")

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_options(-Wno-unused-variable -Wno-cpp )
  # add_compile_options(-Wno-cpp)
endif()


include_directories(include)
include_directories(lib)


# 根据系统架构选择合适的Ryhand库
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  # x86_64 系统
  message(STATUS "Detected x86_64 architecture, searching for x86_64 Ryhand library...")
  find_library(RUIYAN_SRC_LIB
    NAMES Ryhand64 libRyhand64.so libRyhand64
    PATHS ${CMAKE_CURRENT_SOURCE_DIR}/lib
    NO_DEFAULT_PATH
  )
  if(NOT RUIYAN_SRC_LIB)
    message(STATUS "x86_64 library not found, trying generic library...")
    # 尝试通用库
    find_library(RUIYAN_SRC_LIB
      NAMES Ryhand libRyhand.so libRyhand
      PATHS ${CMAKE_CURRENT_SOURCE_DIR}/lib
      NO_DEFAULT_PATH
    )
  endif()
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  # ARM64 系统
  message(STATUS "Detected ARM64 architecture, searching for ARM64 Ryhand library...")
  find_library(RUIYAN_SRC_LIB
    NAMES RyhandArm64 libRyhandArm64.so libRyhandArm64
    PATHS ${CMAKE_CURRENT_SOURCE_DIR}/lib
    NO_DEFAULT_PATH
  )
  if(NOT RUIYAN_SRC_LIB)
    message(STATUS "ARM64 library not found, trying ARM32 library...")
    # 尝试通用ARM库
    find_library(RUIYAN_SRC_LIB
      NAMES RyhandArm libRyhandArm.so libRyhandArm
      PATHS ${CMAKE_CURRENT_SOURCE_DIR}/lib
      NO_DEFAULT_PATH
    )
  endif()
  if(NOT RUIYAN_SRC_LIB)
    message(STATUS "ARM library not found, trying generic library...")
    # 最后尝试通用库
    find_library(RUIYAN_SRC_LIB
      NAMES Ryhand libRyhand.so libRyhand
      PATHS ${CMAKE_CURRENT_SOURCE_DIR}/lib
      NO_DEFAULT_PATH
    )
  endif()
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "arm")
  # ARM 系统（32位）
  message(STATUS "Detected ARM (32-bit) architecture, searching for ARM Ryhand library...")
  find_library(RUIYAN_SRC_LIB
    NAMES RyhandArm libRyhandArm.so libRyhandArm
    PATHS ${CMAKE_CURRENT_SOURCE_DIR}/lib
    NO_DEFAULT_PATH
  )
  if(NOT RUIYAN_SRC_LIB)
    message(STATUS "ARM library not found, trying generic library...")
    # 尝试通用库
    find_library(RUIYAN_SRC_LIB
      NAMES Ryhand libRyhand.so libRyhand
      PATHS ${CMAKE_CURRENT_SOURCE_DIR}/lib
      NO_DEFAULT_PATH
    )
  endif()
else()
  # 其他架构，尝试通用库
  message(STATUS "Unknown architecture ${CMAKE_SYSTEM_PROCESSOR}, searching for generic Ryhand library...")
  find_library(RUIYAN_SRC_LIB
    NAMES Ryhand libRyhand.so libRyhand Ryhand64 libRyhand64.so libRyhand64
    PATHS ${CMAKE_CURRENT_SOURCE_DIR}/lib
    NO_DEFAULT_PATH
  )
endif()

# 查找URDF相关系统库（支持ARM架构）
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  set(URDF_LIB_PATHS /usr/lib/x86_64-linux-gnu /usr/lib /usr/local/lib)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  set(URDF_LIB_PATHS /usr/lib/aarch64-linux-gnu /usr/lib /usr/local/lib)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "arm")
  set(URDF_LIB_PATHS /usr/lib/arm-linux-gnueabihf /usr/lib /usr/local/lib)
else()
  set(URDF_LIB_PATHS /usr/lib /usr/local/lib)
endif()

# 首先在架构特定路径中查找
find_library(URDFDOM_MODEL_LIB
  NAMES urdfdom_model
  PATHS ${URDF_LIB_PATHS}
  NO_DEFAULT_PATH
)

find_library(URDFDOM_SENSOR_LIB
  NAMES urdfdom_sensor
  PATHS ${URDF_LIB_PATHS}
  NO_DEFAULT_PATH
)

find_library(URDFDOM_WORLD_LIB
  NAMES urdfdom_world
  PATHS ${URDF_LIB_PATHS}
  NO_DEFAULT_PATH
)

# 如果没找到，尝试在标准路径中查找（回退机制）
if(NOT URDFDOM_MODEL_LIB)
  find_library(URDFDOM_MODEL_LIB
    NAMES urdfdom_model
    PATHS /usr/lib /usr/local/lib
  )
endif()

if(NOT URDFDOM_SENSOR_LIB)
  find_library(URDFDOM_SENSOR_LIB
    NAMES urdfdom_sensor
    PATHS /usr/lib /usr/local/lib
  )
endif()

if(NOT URDFDOM_WORLD_LIB)
  find_library(URDFDOM_WORLD_LIB
    NAMES urdfdom_world
    PATHS /usr/lib /usr/local/lib
  )
endif()

# 检查URDF库是否找到（可选，不强制要求）
if (URDFDOM_MODEL_LIB)
  message(STATUS "URDF model library found: ${URDFDOM_MODEL_LIB}")
else()
  message(WARNING "URDF model library not found, some features may not work")
endif()

# 确保库文件被找到
if (NOT RUIYAN_SRC_LIB)
    message(FATAL_ERROR "Ryhand library not found!")
    message(STATUS "Searched for library in: ${CMAKE_CURRENT_SOURCE_DIR}/lib")
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
      message(STATUS "Expected ARM64 library: libRyhandArm64.so or libRyhandArm.so")
    elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "arm")
      message(STATUS "Expected ARM library: libRyhandArm.so")
    elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
      message(STATUS "Expected x86_64 library: libRyhand64.so")
    endif()
endif()
message(STATUS "Ryhand library found: ${RUIYAN_SRC_LIB}")


include_directories(include)



# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(rh6_cmd REQUIRED)
find_package(rh6_msg REQUIRED)

find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)

# 查找pinocchio包
find_package(pinocchio REQUIRED)
message(STATUS "Using ROS2 pinocchio package")



add_executable(rh_ctrl src/ryhandlib_port.c src/can_socket.c src/rh6_ctrl.cpp )
target_include_directories(rh_ctrl PUBLIC include lib )
target_link_libraries(rh_ctrl 
  ${RUIYAN_SRC_LIB}
  rclcpp::rclcpp
  pinocchio::pinocchio
  ament_index_cpp::ament_index_cpp
)
ament_target_dependencies(rh_ctrl rclcpp rh6_cmd rh6_msg pinocchio ament_index_cpp geometry_msgs Eigen3 eigen3_cmake_module)


add_executable(rh_test src/rh6_test.cpp)
target_include_directories(rh_test PUBLIC include)
ament_target_dependencies(rh_test rclcpp rh6_cmd)



add_executable(rh6_kinematics src/rh6_kinematics.cpp)
target_link_libraries(rh6_kinematics
  rclcpp::rclcpp
  pinocchio::pinocchio
  ament_index_cpp::ament_index_cpp
  ${URDFDOM_MODEL_LIB}
  ${URDFDOM_SENSOR_LIB}
  ${URDFDOM_WORLD_LIB}
)
ament_target_dependencies(rh6_kinematics sensor_msgs rclcpp pinocchio ament_index_cpp geometry_msgs Eigen3 eigen3_cmake_module)




install(TARGETS
  rh_ctrl rh_test rh6_kinematics
  DESTINATION lib/${PROJECT_NAME}
)


install(FILES ${RUIYAN_SRC_LIB}
  DESTINATION lib/
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch.py")

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "*.yaml")

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)




if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

