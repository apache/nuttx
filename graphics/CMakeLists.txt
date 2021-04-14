

if("${CONFIG_NX}" STREQUAL "y")

	include_directories(BEFORE ${CMAKE_CURRENT_SOURCE_DIR})

	set(SRCS)

	include_directories(nxbe)
	include_directories(nxmu)

	add_subdirectory(nxbe)
	add_subdirectory(nxglib)
	add_subdirectory(nxmu)
	add_subdirectory(nxterm)
	add_subdirectory(vnc)

	nuttx_add_library(graphics ${SRCS})
	target_include_directories(graphics INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

endif()
