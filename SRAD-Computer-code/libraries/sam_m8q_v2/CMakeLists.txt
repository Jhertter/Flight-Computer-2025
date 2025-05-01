add_library(sam_m8q_v2 INTERFACE)

target_sources(sam_m8q_v2 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/sam_m8q_v2.cpp
)

target_link_libraries(sam_m8q_v2
    INTERFACE
    pico_stdlib
    hardware_i2c
    hardware_gpio
)