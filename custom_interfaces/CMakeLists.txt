cmake_minimum_required(VERSION 3.8)
project(custom_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_msgs REQUIRED)

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

# For the use of non-standard messages and services in custom interfaces,
# you have to add the package prefix before the msg or srv.
# e.g. builtin_interfaces/Time timestamp
# https://docs.ros.org/en/rolling/Concepts/About-ROS-Interfaces.html
# https://answers.ros.org/question/362473/how-to-add-a-time-field-in-a-custom-message-file/

#set(msg_files
#  # No comma after each entry
#  "msg/Placeholder.msg"
#)

set(srv_files
  # No comma after each entry
  "srv/GetPose.srv"
)

# You must add DEPENDENCIES for msg and srv types, even if they are ROS/2
# implemented types. Do this in the "find_package()" section above as well.
# https://answers.ros.org/question/296086/using-custom-msg-in-custom-srv/
# https://github.com/ros2/common_interfaces/blob/1d80aae3de9b0c03052e13ec04a0c32693b38f61/sensor_msgs/CMakeLists.txt#L53=L58

rosidl_generate_interfaces(${PROJECT_NAME}
#  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces geometry_msgs diagnostic_msgs std_msgs
  ADD_LINTER_TESTS
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

# I'm not sure entirely sure what this command does.
# Seems to install the directories of the install directory of the workspace.
# Makes in the butt for debugging so I leave it off.
# ament_export_include_directories(include)

ament_package()
