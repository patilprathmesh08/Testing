# =============================================================================
# Host (PC) unit tests for application/ code.
# NOT part of the firmware build: compiled with the PC gcc, not arm-none-eabi.
#
# Build and run (from the CnrMotorCnt folder):
#   cmake -S test -B build_test -G Ninja -DCMAKE_C_COMPILER=gcc
#   cmake --build build_test
#   ctest --test-dir build_test --output-on-failure
# Coverage report:
#   gcovr -r . --filter application/ --html-details build_test/coverage.html --branches
# =============================================================================
cmake_minimum_required(VERSION 3.20)
project(cnr_unit_tests C)

# Unity test framework (downloaded once at configure time)
include(FetchContent)
FetchContent_Declare(unity
  GIT_REPOSITORY https://github.com/ThrowTheSwitch/Unity.git
  GIT_TAG        v2.6.0)
FetchContent_MakeAvailable(unity)

set(APP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../application)

# ---- command handler tests ----
add_executable(test_command_handler
  test_command_handler.c
  ${APP_DIR}/Src/cnr_command_handler.c)

# stubs/ first, so "main.h" and "cmsis_os.h" resolve to the PC fakes
target_include_directories(test_command_handler PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/stubs
  ${APP_DIR}/Inc)
target_link_libraries(test_command_handler PRIVATE unity)

# coverage: instrument the code, no optimisation so lines map 1:1
target_compile_options(test_command_handler PRIVATE --coverage -O0 -g)
target_link_options(test_command_handler PRIVATE --coverage)

enable_testing()
add_test(NAME command_handler COMMAND test_command_handler)
