cmake_minimum_required(VERSION 3.8)
project(brake)

# Compiler settings
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Logic shared by all targets
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Command.srv"
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")


include_directories(
  include
)

set(project_SOURCE_FILES
  src/interface.cpp
)

set(fake_SOURCE_FILES
  ${project_SOURCE_FILES}
  src/fake_interface.cpp
  src/fake_node.cpp
  src/fake_factory.cpp
)

set(project_DEPENDENCIES
  rclcpp
  std_msgs
)

# Executable target
add_executable("fake" src/fake.cpp ${fake_SOURCE_FILES})
target_link_libraries("fake" ${rclcpp_LIBRARIES} "${cpp_typesupport_target}")
ament_target_dependencies("fake" ${project_DEPENDENCIES})
install(TARGETS
  "fake"
  DESTINATION lib/${PROJECT_NAME}
)

# Library target
add_library(library ${project_SOURCE_FILES})
target_link_libraries(library ${rclcpp_LIBRARIES} "${cpp_typesupport_target}")
ament_target_dependencies(library ${project_DEPENDENCIES})
ament_export_targets(${PROJECT_NAME}_library HAS_LIBRARY_TARGET)
install(TARGETS library
  EXPORT ${PROJECT_NAME}_library
  LIBRARY DESTINATION lib
)
add_library(fake_library ${fake_SOURCE_FILES})
target_link_libraries(fake_library ${rclcpp_LIBRARIES} "${cpp_typesupport_target}")
ament_target_dependencies(fake_library ${project_DEPENDENCIES})
ament_export_targets(${PROJECT_NAME}_fake_library HAS_LIBRARY_TARGET)
install(TARGETS fake_library
  EXPORT ${PROJECT_NAME}_fake_library
  LIBRARY DESTINATION lib
)

ament_export_dependencies(${project_DEPENDENCIES})
ament_export_include_directories(include)
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
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

ament_package()
