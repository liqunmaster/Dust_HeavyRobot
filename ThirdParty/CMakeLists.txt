set(THIRDPARTY_ROOT ${CMAKE_CURRENT_LIST_DIR})

set(EIGEN_INCLUDE_DIR ${THIRDPARTY_ROOT}/eigen)
set(SEGGER_RTT_INCLUDE_DIR ${THIRDPARTY_ROOT}/SEGGER/RTT)
set(SEGGER_CONFIG_INCLUDE_DIR ${THIRDPARTY_ROOT}/SEGGER/Config)
set(TINYUSB_INCLUDE_DIR ${THIRDPARTY_ROOT}/tinyusb/src)
set(ANDES_DSP_INCLUDE_DIR ${THIRDPARTY_ROOT}/Andes_DSP/include)
set(ANDES_DSP_LIBRARY ${THIRDPARTY_ROOT}/Andes_DSP/lib/gcc/libdspf.a)

foreach(include_dir IN ITEMS
        ${EIGEN_INCLUDE_DIR}
        ${SEGGER_RTT_INCLUDE_DIR}
        ${SEGGER_CONFIG_INCLUDE_DIR}
        ${TINYUSB_INCLUDE_DIR}
        ${ANDES_DSP_INCLUDE_DIR}
        ${TINYUSB_INCLUDE_DIR}/common
        ${TINYUSB_INCLUDE_DIR}/device
        ${TINYUSB_INCLUDE_DIR}/host
        ${TINYUSB_INCLUDE_DIR}/portable)
    if(NOT IS_DIRECTORY ${include_dir})
        message(FATAL_ERROR "Third-party include directory does not exist: ${include_dir}")
    endif()
endforeach()

target_include_directories(app SYSTEM PRIVATE
    ${EIGEN_INCLUDE_DIR}
    ${SEGGER_RTT_INCLUDE_DIR}
    ${SEGGER_CONFIG_INCLUDE_DIR}
    ${TINYUSB_INCLUDE_DIR}
    ${TINYUSB_INCLUDE_DIR}/common
    ${TINYUSB_INCLUDE_DIR}/device
    ${TINYUSB_INCLUDE_DIR}/host
    ${TINYUSB_INCLUDE_DIR}/portable
    ${ANDES_DSP_INCLUDE_DIR}
)

target_compile_definitions(app PRIVATE
    EIGEN_NO_MALLOC
    EIGEN_DONT_VECTORIZE
)

# The prebuilt library targets the Andes DSP toolchain.  Keep it available to
# the application, but link it only when the board/toolchain explicitly opts
# in through CONFIG_HPM_RISCV_DSP.  The default Zephyr GCC build does not
# advertise the Andes DSP ISA and must use the normal FPU/libm path.
if(CONFIG_HPM_RISCV_DSP)
    if(NOT EXISTS ${ANDES_DSP_LIBRARY})
        message(FATAL_ERROR "Andes DSP library does not exist: ${ANDES_DSP_LIBRARY}")
    endif()
    target_link_libraries(app PRIVATE ${ANDES_DSP_LIBRARY})
    target_compile_definitions(app PRIVATE HPM_EN_MATH_DSP_LIB=1)
endif()
