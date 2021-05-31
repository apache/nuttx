if(CONFIG_NX)
  nuttx_add_kernel_library(graphics)

  target_include_directories(graphics PRIVATE nxmu nxbe)

	add_subdirectory(nxbe)
	add_subdirectory(nxglib)
	add_subdirectory(nxmu)
	add_subdirectory(nxterm)
	add_subdirectory(vnc)
endif()
