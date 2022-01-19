cmake_minimum_required(VERSION 3.8)
project(uav-rai-demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

option(USE_PYBIND OFF)
option(USE_LIBFRANKA OFF)
add_subdirectory(deps/botop EXCLUDE_FROM_ALL)

add_executable(demo1
  src/demo1.cpp
)
target_link_libraries(demo1
  rai
)
target_include_directories(demo1 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/deps/botop/src>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/deps/botop/rai/rai>
  $<INSTALL_INTERFACE:include>)
target_compile_features(demo1 PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS demo1
  DESTINATION lib/${PROJECT_NAME})

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
