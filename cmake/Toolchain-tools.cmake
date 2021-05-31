set(CMAKE_FIND_ROOT_PATH get_file_component(${CMAKE_C_COMPILER} PATH))
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

# REQUIRED OS tools
foreach(tool make)
	string(TOUPPER ${tool} TOOL)
	message(STATUS "TOOL: ${TOOL}")
	find_program(${TOOL} ${tool} REQUIRED)
endforeach()

# OPTIONAL tools
foreach(tool srec_cat)
	string(TOUPPER ${tool} TOOL)
	find_program(${TOOL} ${tool})
endforeach()

## optional compiler tools
#foreach(tool gdb gdbtui)
#	string(TOUPPER ${tool} TOOL)
#	find_program(${TOOL} arm-none-eabi-${tool})
#endforeach()
