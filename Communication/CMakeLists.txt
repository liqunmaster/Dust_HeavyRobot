target_sources(app PRIVATE
    remote_port.cpp
    imu_port.cpp
    fdcan_port.cpp
    usb_port.cpp
)
target_include_directories(app PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
