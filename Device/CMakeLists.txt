target_sources(app PRIVATE
    servo.cpp
    icm42688p_hxy.cpp
    ws2812b.cpp
)
target_include_directories(app PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
