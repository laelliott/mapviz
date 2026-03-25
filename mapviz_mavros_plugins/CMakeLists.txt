### SET CMAKE POLICIES ###
cmake_minimum_required(VERSION 3.10...3.17)

if(${CMAKE_VERSION} VERSION_LESS 3.12)
    cmake_policy(VERSION ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION})
endif()

if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(${CMAKE_VERSION} VERSION_EQUAL "3.11.0" OR ${CMAKE_VERSION} VERSION_GREATER "3.11.0")
  cmake_policy(SET CMP0072 NEW)
endif()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
### END CMAKE POLICIES ###

project(mapviz_mavros_plugins LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(mapviz REQUIRED)
find_package(mapviz_plugins REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5OpenGL REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(rclcpp REQUIRED)
find_package(swri_transform_util REQUIRED)
find_package(tf2 REQUIRED)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

# Build plugin library
add_library(${PROJECT_NAME} SHARED
  src/waypoint_list_plugin.cpp
  include/${PROJECT_NAME}/waypoint_list_plugin.h
  ui/waypoint_list_config.ui
)

set_target_properties(${PROJECT_NAME} PROPERTIES
  AUTOUIC_SEARCH_PATHS "${CMAKE_CURRENT_SOURCE_DIR}/ui"
)

# mapviz_plugins exported target sets INTERFACE_INCLUDE_DIRECTORIES to the
# generic ROS include root, but package-specific headers (including the
# auto-generated ui_topicselect.h) live under the package subdirectory.
# Add the package-specific include dir so <mapviz_plugins/topic_select.h> works.
set(_mapviz_plugins_inc_dir "${mapviz_plugins_DIR}/../../../include/mapviz_plugins")
get_filename_component(_mapviz_plugins_inc_dir "${_mapviz_plugins_inc_dir}" ABSOLUTE)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
    $<INSTALL_INTERFACE:include>
)
target_include_directories(${PROJECT_NAME} SYSTEM
  PUBLIC ${_mapviz_plugins_inc_dir}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${mapviz_TARGETS}
  ${mapviz_plugins_TARGETS}
  ${mavros_msgs_TARGETS}
  ${swri_transform_util_TARGETS}
  pluginlib::pluginlib
  Qt5::Core
  Qt5::Gui
  Qt5::OpenGL
  Qt5::Widgets
  rclcpp::rclcpp
  tf2::tf2
)

### Install ###
install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

pluginlib_export_plugin_description_file(mapviz mapviz_plugins.xml)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  mapviz
  mapviz_plugins
  mavros_msgs
  pluginlib
  Qt5Core
  Qt5Gui
  Qt5OpenGL
  Qt5Widgets
  rclcpp
  swri_transform_util
  tf2
)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
