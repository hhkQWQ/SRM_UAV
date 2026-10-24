add_library(usermod_rm_vo INTERFACE)
target_sources(usermod_rm_vo INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/modvo.c
    ${CMAKE_CURRENT_LIST_DIR}/vo_core.c
    ${CMAKE_CURRENT_LIST_DIR}/vo_fast.c)
target_include_directories(usermod_rm_vo INTERFACE ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(usermod INTERFACE usermod_rm_vo)
