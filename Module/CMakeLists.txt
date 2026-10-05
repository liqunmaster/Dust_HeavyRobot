target_sources(app PRIVATE
    input.cpp
    input_parser.cpp
    ins.cpp
)
target_include_directories(app PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
