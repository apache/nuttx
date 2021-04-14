
add_subdirectory(libc)
add_subdirectory(libxx)

if("${CONFIG_LIBDSP}" STREQUAL "y")
	add_subdirectory(libdsp)
endif()

if("${CONFIG_NX}" STREQUAL "y")
	add_subdirectory(libnx)
endif()
