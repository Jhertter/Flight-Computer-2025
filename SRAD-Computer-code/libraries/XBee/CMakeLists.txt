add_library(XBee INTERFACE)

target_sources(XBee INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/XBee.cpp
)

target_link_libraries(XBee
    INTERFACE
    hardware_uart
    pico_stdlib
)