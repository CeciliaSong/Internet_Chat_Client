cmake_minimum_required(VERSION 3.8)
project(internet_chat_client)

# Export compile commands for IntelliSense
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(wxWidgets COMPONENTS core base QUIET)

# include directory
include_directories(include)

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

# Create shared library
add_library(chat_node_lib SHARED src/chat_node_impl.cpp)
ament_target_dependencies(chat_node_lib rclcpp std_msgs)
target_include_directories(chat_node_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
set_target_properties(chat_node_lib PROPERTIES
  SOVERSION 0
  VERSION 0.1.0
)

# CLI executable (old chat_node)
add_executable(chat_node src/chat_node.cpp)
target_link_libraries(chat_node chat_node_lib)
ament_target_dependencies(chat_node rclcpp std_msgs)
target_include_directories(chat_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
set_target_properties(chat_node PROPERTIES
  INSTALL_RPATH "$ORIGIN"
)

# GUI executable (if wxWidgets is found)
if(wxWidgets_FOUND)
  include(${wxWidgets_USE_FILE})
  
  add_executable(chat_gui src/chat_gui_main.cpp)
  target_link_libraries(chat_gui chat_node_lib ${wxWidgets_LIBRARIES})
  ament_target_dependencies(chat_gui rclcpp std_msgs)
  target_include_directories(chat_gui PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  set_target_properties(chat_gui PROPERTIES
    INSTALL_RPATH "$ORIGIN"
  )
  
  # Launcher executable
  add_executable(chat_launcher src/chat_launcher.cpp)
  target_link_libraries(chat_launcher chat_node_lib ${wxWidgets_LIBRARIES} wx_gtk3u_richtext-3.0)
  ament_target_dependencies(chat_launcher rclcpp std_msgs)
  target_include_directories(chat_launcher PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  set_target_properties(chat_launcher PROPERTIES
    INSTALL_RPATH "$ORIGIN"
  )
  
  install(TARGETS chat_gui chat_launcher DESTINATION lib/${PROJECT_NAME})
  message(STATUS "wxWidgets found. Building chat_gui and chat_launcher...")
else()
  message(WARNING "wxWidgets not found. Skipping GUI build. Install wxWidgets to enable GUI.")
endif()

install(TARGETS
  chat_node
  chat_node_lib
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
