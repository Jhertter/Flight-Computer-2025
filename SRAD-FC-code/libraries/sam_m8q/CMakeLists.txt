add_library(sam_m8q INTERFACE)

target_sources(sam_m8q INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/sam_m8q.cpp
)

target_link_libraries(sam_m8q
    INTERFACE
    pico_stdlib
    hardware_i2c
)