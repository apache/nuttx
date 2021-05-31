add_subdirectory(src)

if (NOT CONFIG_BUILD_FLAT)
  add_subdirectory(kernel)
  set_property(GLOBAL PROPERTY LD_SCRIPT_USER ${CMAKE_CURRENT_LIST_DIR}/scripts/memory.ld ${CMAKE_CURRENT_LIST_DIR}/scripts/user-space.ld)
endif()
