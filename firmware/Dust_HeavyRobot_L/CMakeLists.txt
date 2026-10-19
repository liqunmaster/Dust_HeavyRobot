cmake_minimum_required(VERSION 3.20.0)

set(ROBOT_BOARD L)
include(${CMAKE_CURRENT_LIST_DIR}/../../cmake/mcu_common.cmake)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(Dust_HeavyRobot_L)

mcu_add_common_sources()
target_sources(app PRIVATE main.cpp
    App/app_chassis.cpp
    App/app_liftup.cpp
    App/app_booster.cpp
    App/board_input.cpp
    App/mcu_common.cpp)
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/App)
