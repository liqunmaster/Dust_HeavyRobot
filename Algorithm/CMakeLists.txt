target_include_directories(app PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_sources(app PRIVATE
    adrc.cpp
    double_buffer.cpp
    eso.cpp
    least_squares.cpp
    pid.cpp
    ring_buffer.cpp
    crc.cpp
    kalman.cpp
    quaternion.cpp
)

target_compile_options(app PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:-fno-threadsafe-statics>)
