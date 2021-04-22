if(CONFIG_NX)
  nuttx_add_library(graphics)

	add_subdirectory(nxbe)
	add_subdirectory(nxglib)
	add_subdirectory(nxmu)
	add_subdirectory(nxterm)
	add_subdirectory(vnc)
endif()
