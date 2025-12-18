cmake_minimum_required(VERSION 3.10)
project(rh6_ecat)

# 输出系统架构信息（用于调试ARM平台问题）
message(STATUS "CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "CMAKE_SYSTEM_NAME: ${CMAKE_SYSTEM_NAME}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 查找自定义包
find_package(rh6_msg REQUIRED)
find_package(rh6_cmd REQUIRED)

# 查找 EtherCAT 库（支持 IGH Etherlab）
# 首先尝试 pkg-config 方式
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
  pkg_check_modules(ECAT_PKG QUIET libethercat)
endif()

# 如果 pkg-config 找到了，使用它
if(ECAT_PKG_FOUND)
  set(ECAT_LIB ${ECAT_PKG_LIBRARIES})
  set(ECAT_INCLUDE_DIRS ${ECAT_PKG_INCLUDE_DIRS})
  message(STATUS "EtherCAT library found via pkg-config: ${ECAT_LIB}")
  add_compile_definitions(ECAT_LIB_FOUND)
else()
  # 手动查找库文件（支持ARM架构）
  set(ECAT_LIB_NAMES libethercat.so libethercat)
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    set(ECAT_LIB_PATHS 
      /opt/etherlab/lib
      /usr/lib 
      /usr/local/lib 
      /usr/lib/x86_64-linux-gnu
    )
  elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    set(ECAT_LIB_PATHS 
      /opt/etherlab/lib
      /usr/lib 
      /usr/local/lib 
      /usr/lib/aarch64-linux-gnu
    )
  elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "arm")
    set(ECAT_LIB_PATHS 
      /opt/etherlab/lib
      /usr/lib 
      /usr/local/lib 
      /usr/lib/arm-linux-gnueabihf
    )
  else()
    set(ECAT_LIB_PATHS 
      /opt/etherlab/lib
      /usr/lib 
      /usr/local/lib 
    )
  endif()

  find_library(ECAT_LIB 
    NAMES ${ECAT_LIB_NAMES}
    PATHS ${ECAT_LIB_PATHS}
    NO_DEFAULT_PATH
  )

  # 如果没找到，尝试默认路径
  if(NOT ECAT_LIB)
    find_library(ECAT_LIB 
      NAMES ${ECAT_LIB_NAMES}
      PATHS /usr/lib /usr/local/lib
    )
  endif()

  if(ECAT_LIB)
    message(STATUS "EtherCAT library found: ${ECAT_LIB}")
    add_compile_definitions(ECAT_LIB_FOUND)
    
    # 设置包含目录
    if(EXISTS "/opt/etherlab/include")
      set(ECAT_INCLUDE_DIRS "/opt/etherlab/include")
    elseif(EXISTS "/usr/include")
      # 在某些ARM平台上，EtherCAT头文件可能在标准路径
      set(ECAT_INCLUDE_DIRS "/usr/include")
    endif()
  else()
    message(WARNING "EtherCAT library not found. EtherCAT communication will not be available.")
    message(STATUS "Searched for libraries: ${ECAT_LIB_NAMES}")
    message(STATUS "Searched in paths: ${ECAT_LIB_PATHS}")
    message(STATUS "To enable EtherCAT support, ensure libethercat is installed and accessible")
  endif()
endif()

include_directories(
  include
  ${ECAT_INCLUDE_DIRS}
  ../rh6_ctrl/lib  # 添加ryhandlib.h路径
)

# 生成服务文件
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetHandMode.srv"
  DEPENDENCIES std_msgs
)

# 手部控制节点已移除，只使用桥梁架构

# 桥梁架构 - 通信桥梁工厂
add_library(communication_bridge_factory_lib
  src/communication_bridge_factory.cpp
)
target_include_directories(communication_bridge_factory_lib PUBLIC include)
ament_target_dependencies(communication_bridge_factory_lib rclcpp std_msgs sensor_msgs)

# 桥梁架构 - EtherCAT 桥梁（条件编译）
if(ECAT_LIB)
  add_library(ethercat_bridge_lib
    src/ethercat_bridge.cpp
    src/ethercat_slave_config.cpp
    src/shm_utils.c
  )
  target_include_directories(ethercat_bridge_lib PUBLIC include ${ECAT_INCLUDE_DIRS})
  target_compile_definitions(ethercat_bridge_lib PRIVATE ECAT_LIB_FOUND)
  ament_target_dependencies(ethercat_bridge_lib rclcpp std_msgs sensor_msgs)
  target_link_libraries(ethercat_bridge_lib rt pthread ${ECAT_LIB})
  
  # 如果使用 pkg-config，添加额外的链接库
  if(ECAT_PKG_FOUND)
    target_link_libraries(ethercat_bridge_lib ${ECAT_PKG_LIBRARIES})
    target_include_directories(ethercat_bridge_lib PUBLIC ${ECAT_PKG_INCLUDE_DIRS})
  endif()
else()
  # 创建空的 EtherCAT 桥梁库（用于编译）
  add_library(ethercat_bridge_lib
    src/ethercat_bridge_stub.cpp
  )
  target_include_directories(ethercat_bridge_lib PUBLIC include)
  ament_target_dependencies(ethercat_bridge_lib rclcpp std_msgs sensor_msgs)
  target_link_libraries(ethercat_bridge_lib rt pthread)
endif()

# 桥梁架构 - ROS 手部接口
add_library(ros_hand_interface_lib
  src/ros_hand_interface.cpp
  src/shm_utils.c
)
target_include_directories(ros_hand_interface_lib PUBLIC include)
ament_target_dependencies(ros_hand_interface_lib rclcpp std_msgs sensor_msgs rh6_msg)
target_link_libraries(ros_hand_interface_lib rt pthread)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(ros_hand_interface_lib "${cpp_typesupport_target}")

# 桥梁架构 - 完整系统
add_executable(hand_system_bridge
  src/hand_system_bridge.cpp
)
target_include_directories(hand_system_bridge PUBLIC include)
ament_target_dependencies(hand_system_bridge rclcpp std_msgs sensor_msgs)
target_link_libraries(hand_system_bridge 
  communication_bridge_factory_lib
  ethercat_bridge_lib
  ros_hand_interface_lib
  rt
  pthread
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(hand_system_bridge "${cpp_typesupport_target}")

# EtherCAT 主站程序已移除，使用系统主站

# 手部模式控制示例
add_executable(hand_mode_control examples/hand_mode_control.cpp)
target_include_directories(hand_mode_control PUBLIC include)
ament_target_dependencies(hand_mode_control rclcpp std_msgs)

# EtherCAT正弦波测试节点
add_executable(ethercat_sine_wave_test examples/ethercat_sine_wave_test.cpp)
target_include_directories(ethercat_sine_wave_test PUBLIC include)
ament_target_dependencies(ethercat_sine_wave_test rclcpp std_msgs rh6_msg rh6_cmd)

# C/C++ 兼容性测试
add_executable(compatibility_test 
  examples/compatibility_test.cpp
  src/shm_utils.c
)
target_include_directories(compatibility_test PUBLIC include)
target_link_libraries(compatibility_test 
  rt  # 共享内存支持
  pthread  # 线程支持
)

# EtherCAT 诊断工具已移除 - 使用主程序进行测试

# 链接睿研库（可选）
# 根据系统架构选择合适的库文件
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  # x86_64 系统
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/lib/libRyhand64.so)
    message(STATUS "Linking x86_64 Ryhand library")
    target_link_libraries(hand_system_bridge 
      ${CMAKE_CURRENT_SOURCE_DIR}/lib/libRyhand64.so
    )
  else()
    message(WARNING "x86_64 Ryhand library not found")
  endif()
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  # ARM64 系统
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/lib/libRyhandArm64.so)
    message(STATUS "Linking ARM64 Ryhand library")
    target_link_libraries(hand_system_bridge 
      ${CMAKE_CURRENT_SOURCE_DIR}/lib/libRyhandArm64.so
    )
  else()
    message(WARNING "ARM64 Ryhand library not found")
  endif()
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "arm")
  # ARM 系统
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/lib/libRyhandArm.so)
    message(STATUS "Linking ARM Ryhand library")
    target_link_libraries(hand_system_bridge 
      ${CMAKE_CURRENT_SOURCE_DIR}/lib/libRyhandArm.so
    )
  else()
    message(WARNING "ARM Ryhand library not found")
  endif()
else()
  # 其他架构，尝试通用库
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/lib/libRyhand.so)
    message(STATUS "Linking generic Ryhand library")
    target_link_libraries(hand_system_bridge 
      ${CMAKE_CURRENT_SOURCE_DIR}/lib/libRyhand.so
    )
  else()
    message(WARNING "Generic Ryhand library not found")
  endif()
endif()

# 安装库文件（只安装存在的文件）
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/lib/libRyhand.so)
  install(FILES lib/libRyhand.so DESTINATION lib/${PROJECT_NAME})
endif()
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/lib/libRyhand64.so)
  install(FILES lib/libRyhand64.so DESTINATION lib/${PROJECT_NAME})
endif()
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/lib/libRyhandArm.so)
  install(FILES lib/libRyhandArm.so DESTINATION lib/${PROJECT_NAME})
endif()
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/lib/libRyhandArm64.so)
  install(FILES lib/libRyhandArm64.so DESTINATION lib/${PROJECT_NAME})
endif()

install(TARGETS 
  hand_mode_control 
  compatibility_test
  hand_system_bridge
  communication_bridge_factory_lib
  ethercat_bridge_lib
  ros_hand_interface_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 诊断工具已移除

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch params scripts
  DESTINATION share/${PROJECT_NAME}
)

# 设置脚本执行权限
install(PROGRAMS scripts/start_bridge_system.sh scripts/stop_bridge_system.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()



