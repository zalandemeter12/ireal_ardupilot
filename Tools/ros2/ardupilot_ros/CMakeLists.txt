cmake_minimum_required(VERSION 3.20)
project(ardupilot_ros)

# --------------------------------------------------------------------------- #
# Find dependencies.
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# --------------------------------------------------------------------------- #
# Add custom targets to configure and build ardupilot.

# TODO(srmainwaring): add cache variables for vehicle type, debug etc.
# set(ARDUPILOT_VEHICLE "copter" CACHE STRING "Vehicle type")

set(ENVIRONMENT_CONFIG "SITL" CACHE STRING "Environment configuration (SITL or LINUX)")

if(ENVIRONMENT_CONFIG STREQUAL "SITL")
    set(ARDUPILOT_BOARD "sitl")
elseif(ENVIRONMENT_CONFIG STREQUAL "LINUX")
    set(ARDUPILOT_BOARD "linux")
else()
    message(FATAL_ERROR "ENVIRONMENT_CONFIG must be SITL or LINUX")
endif()

option(ARDUPILOT_USE_NATIVE_TOOLCHAIN "Use native toolchain" ON)
option(ARDUPILOT_DISABLE_NETWORKING "Disable the networking API code" OFF)
option(ARDUPILOT_DISABLE_SCRIPTING "Disable the scripting API code" OFF)
option(ARDUPILOT_DISABLE_WATCHDOG "Build with watchdog disabled" OFF)
option(ARDUPILOT_ENABLE_DDS "Enable the DDS client" ON)
option(ARDUPILOT_ENABLE_NETWORKING_TESTS "Enable the networking test code" OFF)
option(ARDUPILOT_ENABLE_PPP "Enable PPP networking" OFF)
set(ARDUPILOT_WAF_CONFIGURE_ARGS "" CACHE STRING "Arbitrary waf configure arguments")
set(ARDUPILOT_WAF_BUILD_ARGS "" CACHE STRING  "Arbitrary waf build arguments")
set(ARDUPILOT_VEHICLE "heli" CACHE STRING "ArduPilot vehicle type")

# NOTE: look for `waf` and set source and build directories to top level.
# ${PROJECT_SOURCE_DIR} = ./Tools/ros2/ardupilot
#
cmake_path(SET ARDUPILOT_ROOT NORMALIZE ${PROJECT_SOURCE_DIR}/../../..)

find_program(WAF_COMMAND waf HINTS ${ARDUPILOT_ROOT})
message(STATUS "WAF_COMMAND: ${WAF_COMMAND}")

# Set the build config.
  set(WAF_CONFIG $<$<OR:$<CONFIG:Debug>,$<CONFIG:RelWithDebInfo>>:"--debug">)
set(WAF_DISABLE_NETWORKING $<$<BOOL:${ARDUPILOT_DISABLE_NETWORKING}>:"--disable-networking">)
set(WAF_DISABLE_SCRIPTING $<$<BOOL:${ARDUPILOT_DISABLE_SCRIPTING}>:"--disable-scripting">)
set(WAF_DISABLE_WATCHDOG $<$<BOOL:${ARDUPILOT_DISABLE_WATCHDOG}>:"--disable-watchdog">)
set(WAF_ENABLE_DDS $<$<BOOL:${ARDUPILOT_ENABLE_DDS}>:"--enable-DDS">)
set(WAF_ENABLE_NETWORKING_TESTS $<$<BOOL:${ARDUPILOT_ENABLE_NETWORKING_TESTS}>:"--enable-networking-tests">)
set(WAF_ENABLE_PPP $<$<BOOL:${ARDUPILOT_ENABLE_PPP}>:"--enable-PPP">)
set(WAF_TOOLCHAIN $<$<BOOL:${ARDUPILOT_USE_NATIVE_TOOLCHAIN}>:"--toolchain=native">
)
add_custom_target(ardupilot_configure ALL
  ${WAF_COMMAND} configure --board ${ARDUPILOT_BOARD}
    ${WAF_TOOLCHAIN}
    ${WAF_CONFIG}
    ${WAF_DISABLE_NETWORKING}
    ${WAF_DISABLE_SCRIPTING}
    ${WAF_DISABLE_WATCHDOG}
    ${WAF_ENABLE_DDS}
    ${WAF_ENABLE_NETWORKING_TESTS}
    ${WAF_ENABLE_PPP}
    ${ARDUPILOT_WAF_CONFIGURE_ARGS}
  WORKING_DIRECTORY ${ARDUPILOT_ROOT}
)

add_custom_target(ardupilot_build ALL
  ${WAF_COMMAND} ${ARDUPILOT_VEHICLE} --enable-DDS ${WAF_CONFIG} ${ARDUPILOT_WAF_BUILD_ARGS}
  WORKING_DIRECTORY ${ARDUPILOT_ROOT}
)

add_dependencies(ardupilot_build ardupilot_configure)

# --------------------------------------------------------------------------- #
#  Install.

# Install executables.
install(PROGRAMS
  # ${ARDUPILOT_ROOT}/build/${ARDUPILOT_BOARD}/bin/antennatracker
  # ${ARDUPILOT_ROOT}/build/${ARDUPILOT_BOARD}/bin/arducopter
  ${ARDUPILOT_ROOT}/build/${ARDUPILOT_BOARD}/bin/arducopter-heli
  # ${ARDUPILOT_ROOT}/build/${ARDUPILOT_BOARD}/bin/arduplane
  # ${ARDUPILOT_ROOT}/build/${ARDUPILOT_BOARD}/bin/ardurover
  # ${ARDUPILOT_ROOT}/build/${ARDUPILOT_BOARD}/bin/ardusub
  # ${ARDUPILOT_ROOT}/build/${ARDUPILOT_BOARD}/bin/blimp
  DESTINATION bin/
)

# Install libs.
install(DIRECTORY
  ${ARDUPILOT_ROOT}/build/${ARDUPILOT_BOARD}/lib
  DESTINATION ./
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install additional default params.
install(DIRECTORY
  config/default_params
  DESTINATION share/${PROJECT_NAME}/config
)

# Install autotest default params.
install(DIRECTORY
  ${ARDUPILOT_ROOT}/Tools/autotest/default_params
  DESTINATION share/${PROJECT_NAME}/config/
)

# Install additional autotest model params.
install(DIRECTORY
  ${ARDUPILOT_ROOT}/Tools/autotest/models
  DESTINATION share/${PROJECT_NAME}/config/
)

# Install Python package.
ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR src/${PROJECT_NAME}
)

# --------------------------------------------------------------------------- #
# build tests

if(BUILD_TESTING)
  # Add linters.
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Add python tests.
  find_package(ament_cmake_pytest REQUIRED)
  set(_pytest_tests
    # Add tests here, for example:
    # tests/test_a.py
  )
  foreach(_test_path ${_pytest_tests})
    get_filename_component(_test_name ${_test_path} NAME_WE)
    ament_add_pytest_test(${_test_name} ${_test_path}
      APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
      TIMEOUT 60
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )
  endforeach()
endif()

# --------------------------------------------------------------------------- #
# Call last.

ament_package()
