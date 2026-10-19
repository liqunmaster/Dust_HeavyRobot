target_sources(app PRIVATE
    canfd_bus.cpp
    remote_port.cpp
    imu_port.cpp
    motor_port.cpp
    usb_port.cpp
    rs485_bus.cpp
)
target_include_directories(app PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
