add_library(fsm INTERFACE)

target_sources(fsm INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/fsmtable.cpp
    ${CMAKE_CURRENT_LIST_DIR}/fsm.cpp
)

target_link_libraries(fsm
    INTERFACE
)