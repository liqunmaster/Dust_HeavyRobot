target_include_directories(app PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(TOPIC_MSG_FILES)
set(TOPIC_GENERATED_HEADERS)
set(TOPIC_GENERATED_ROOT ${CMAKE_BINARY_DIR}/generated)
foreach(name IN ITEMS remote_topic chassis_setpoint lift_setpoint chassis_status)
    list(APPEND TOPIC_MSG_FILES ${CMAKE_CURRENT_SOURCE_DIR}/msg/${name}.msg)
    list(APPEND TOPIC_GENERATED_HEADERS ${TOPIC_GENERATED_ROOT}/msg/${name}.hpp)
endforeach()
add_custom_command(
    OUTPUT ${TOPIC_GENERATED_HEADERS}
    COMMAND ${Python3_EXECUTABLE} ${ROBOT_ROOT}/Script/tools/generate_messages.py
        --output-dir ${TOPIC_GENERATED_ROOT}/msg ${TOPIC_MSG_FILES}
    DEPENDS ${TOPIC_MSG_FILES} ${ROBOT_ROOT}/Script/tools/generate_messages.py
    COMMENT "Generating C++ messages from .msg definitions"
    VERBATIM)
add_custom_target(topic_messages DEPENDS ${TOPIC_GENERATED_HEADERS})
add_dependencies(app topic_messages)
target_include_directories(app PUBLIC ${TOPIC_GENERATED_ROOT})
target_sources(app PRIVATE motor_channel.cpp imu_channel.cpp remote_channel.cpp chassis_channel.cpp)
