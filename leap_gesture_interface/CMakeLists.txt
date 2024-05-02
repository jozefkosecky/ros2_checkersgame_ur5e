cmake_minimum_required(VERSION 3.8)

# Define the project name, version, and supported languages
project(leap_gesture_interface VERSION "1.0.0" LANGUAGES CXX C)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find ROS2 dependencies
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
# Setup LeapSDK root path based on the OS
if (WIN32)
   set(ULTRALEAP_PATH_ROOT "$ENV{ProgramFiles}/Ultraleap")
elseif (APPLE)
   set(ULTRALEAP_PATH_ROOT "/Applications/Ultraleap\ Hand\ Tracking\ Service.app/Contents/LeapSDK/lib/cmake/LeapSDK")
else()  # For Linux and other OSs
   set(ULTRALEAP_PATH_ROOT "")
endif()

# If on a UNIX system, locate the Threads library (useful for multi-threading)
if (UNIX)    
    find_package(Threads REQUIRED)    
endif (UNIX)

# Locate the LeapSDK on the system
find_package(LeapSDK 5 REQUIRED PATHS "${ULTRALEAP_PATH_ROOT}")

# Define include directories for header files
include_directories(
    ${rclcpp_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${mavros_msgs_INCLUDE_DIRS}
    ${tf2_ros_INCLUDE_DIRS}
    ${tf2_INCLUDE_DIRS}

    /usr/include  # For LeapC.h
)

# Compile the ExampleConnection library as an object library
add_library(
    libExampleConnection
    OBJECT
    "/usr/share/doc/ultraleap-hand-tracking-service/samples/ExampleConnection.c"
)

# Link the LeapSDK library to the ExampleConnection library
target_link_libraries(
    libExampleConnection
    PUBLIC
    LeapSDK::LeapC
)

# If on a UNIX system, link the Threads library to the ExampleConnection library
if (UNIX)
    target_link_libraries(
        libExampleConnection
        PRIVATE
        Threads::Threads
    )
endif()

# Define where to find the header files for the ExampleConnection library
target_include_directories(
    libExampleConnection
    PUBLIC
    "/usr/share/doc/ultraleap-hand-tracking-service/samples"  # For ExampleConnection.h
)

# Define the ROS node executable
add_executable(gesture_interface_node src/gesture_interface_node.cpp)
add_executable(leap_example_node src/leap_interface_example.cpp)

# Specify dependencies for the ROS node and link necessary libraries
ament_target_dependencies(gesture_interface_node rclcpp std_msgs geometry_msgs)
target_link_libraries(gesture_interface_node libExampleConnection LeapSDK::LeapC)

ament_target_dependencies(leap_example_node rclcpp geometry_msgs tf2_ros tf2)
target_link_libraries(leap_example_node libExampleConnection LeapSDK::LeapC)

# Specify where to install the ROS node executable
install(TARGETS
  gesture_interface_node
  leap_example_node
  DESTINATION lib/${PROJECT_NAME}
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


# Let ROS2 know that this package is a valid ament package
ament_package()
