target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(TOPIC_MSG_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/msg/remote_topic.msg
)
set(TOPIC_GENERATED_ROOT ${CMAKE_BINARY_DIR}/generated)
set(TOPIC_GENERATED_HEADERS
    ${TOPIC_GENERATED_ROOT}/msg/remote_topic.hpp
)

add_custom_command(
    OUTPUT ${TOPIC_GENERATED_HEADERS}
    COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_SOURCE_DIR}/Script/tools/generate_messages.py
        --output-dir ${TOPIC_GENERATED_ROOT}/msg
        ${TOPIC_MSG_FILES}
    DEPENDS
        ${TOPIC_MSG_FILES}
        ${CMAKE_SOURCE_DIR}/Script/tools/generate_messages.py
    COMMENT "Generating C++ messages from .msg definitions"
    VERBATIM
)
add_custom_target(topic_messages DEPENDS ${TOPIC_GENERATED_HEADERS})
add_dependencies(app topic_messages)
target_include_directories(app PUBLIC ${TOPIC_GENERATED_ROOT})

target_sources(app PRIVATE
    fdcan_channel.cpp
    imu_channel.cpp
    remote_channel.cpp
)
