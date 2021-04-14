# For a list of files, prepend the path of the current directory
# to each entry and add the result to SRCS variable.
# This is used to aggregate source files in a subdirectory
# to a NuttX library defined higher up.

macro(set_parent_srcs)

	get_filename_component(CURR_DIR ${CMAKE_CURRENT_LIST_DIR} NAME)

	foreach(file ${ARGN})
		list(APPEND SRCS ${CURR_DIR}/${file})
		set(SRCS "${SRCS}" PARENT_SCOPE)
	endforeach()

endmacro()
