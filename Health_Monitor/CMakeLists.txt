
target_include_directories(app PUBLIC ${CMAKE_CURRENT_LIST_DIR})
target_sources(app PRIVATE motor_health.cpp cpu_usage.cpp can_diagnostics.cpp)
