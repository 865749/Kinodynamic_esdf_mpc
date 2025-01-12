cmake_minimum_required(VERSION 3.8)
project(planner_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
# find_package(ACADO REQUIRED)
# set( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} )
# set(CMAKE_CXX_STANDARD 17)

include_directories(/usr/local/include)
link_directories(/usr/local/lib)

include_directories(
  include
  getting_started_export
  ${ACADO_ENV_INCLUDE_DIRS}
)

ament_auto_find_build_dependencies()
add_library(ACADO STATIC IMPORTED)
# find dependencies
ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

# target_link_libraries(${PROJECT_NAME} ipopt ${ACADO_ENV_SHARED_LIBRARIES})
set_target_properties(ACADO PROPERTIES IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/omni_model/libacado_exported_rti.a")

target_link_libraries(${PROJECT_NAME} ACADO ${ACADO_ENV_SHARED_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME} ${ACADO_ENV_SHARED_LIBRARIES})

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN fast_planner::kino_replan_fsm
  EXECUTABLE ${PROJECT_NAME}_node
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  include
  launch
  config
)
