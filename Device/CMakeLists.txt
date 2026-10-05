target_sources(app PRIVATE
    cubemars.cpp
    dji_motor.cpp
    dji_c610.cpp
    dji_c620.cpp
    dm_motor.cpp
    servo.cpp
    icm42688p_hxy.cpp
    remote_dt7.cpp
    remote_vt02.cpp
    remote_vt03.cpp
    ws2812b.cpp
)
target_include_directories(app PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
