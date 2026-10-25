find_package(Python3 REQUIRED COMPONENTS Interpreter)
file(GLOB ROBOT_MSG_FILES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.msg")
set(ROBOT_GENERATED_ROOT "${CMAKE_BINARY_DIR}/generated")
set(ROBOT_GENERATED_CPP "${ROBOT_GENERATED_ROOT}/uORBTopics.cpp")
set(ROBOT_GENERATED_HEADERS "${ROBOT_GENERATED_ROOT}/uORB/topics/uORBTopics.hpp")
foreach(message IN LISTS ROBOT_MSG_FILES)
	get_filename_component(name "${message}" NAME_WE)
	string(REGEX REPLACE "([A-Z])" "_\\1" name "${name}")
	string(REGEX REPLACE "^_" "" name "${name}")
	string(TOLOWER "${name}" name)
	list(APPEND ROBOT_GENERATED_HEADERS "${ROBOT_GENERATED_ROOT}/uORB/topics/${name}.h")
endforeach()
add_custom_command(OUTPUT ${ROBOT_GENERATED_CPP} ${ROBOT_GENERATED_HEADERS}
	COMMAND ${Python3_EXECUTABLE} "${ROBOT_ROOT}/Tools/msg/generate_messages.py"
		--output-dir "${ROBOT_GENERATED_ROOT}" ${ROBOT_MSG_FILES}
	DEPENDS ${ROBOT_MSG_FILES} "${ROBOT_ROOT}/Tools/msg/generate_messages.py"
	VERBATIM)
add_custom_target(robot_messages DEPENDS ${ROBOT_GENERATED_CPP} ${ROBOT_GENERATED_HEADERS})
add_dependencies(app robot_messages)
target_sources(app PRIVATE ${ROBOT_GENERATED_CPP})
target_include_directories(robot_headers INTERFACE "${ROBOT_GENERATED_ROOT}")
