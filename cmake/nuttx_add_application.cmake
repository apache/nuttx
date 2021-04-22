define_property(GLOBAL PROPERTY NUTTX_APP_LIBS
	BRIEF_DOCS "NuttX application libs"
	FULL_DOCS "List of all NuttX application libraries"
)

#	nuttx_add_application
#
# Description:
#   Declares a NuttX application as a static library. The corresponding target
#   will be named apps_<NAME>. The first entry into the source list is assumed
#   to be the one containing main() and will thus receive a -Dmain=app_main
#   definition during build.
#
#	Usage:
#		nuttx_add_application(
#			NAME <string>
#			[ PRIORITY <string> ]
#			[ STACKSIZE <string> ]
#			[ COMPILE_FLAGS <list> ]
#			[ DEPENDS <string> ]
#			[ SRCS <list> ]
#			)
#
#	Parameters:
#		NAME          : unique name of application
#		PRIORITY		  : priority
#		STACKSIZE		  : stack size
#		COMPILE_FLAGS	: compile flags
#		SRCS			    : source files
#		DEPENDS			  : targets which this module depends on
#
#	Example:
#  nuttx_add_application(
#    NAME test
#    SRCS file.cpp
#    STACKSIZE 1024
#    DEPENDS nshlib
#  )

function(nuttx_add_application)

  # parse arguments into variables

	nuttx_parse_function_args(
		FUNC nuttx_add_application
		ONE_VALUE NAME PRIORITY STACKSIZE
		MULTI_VALUE COMPILE_FLAGS SRCS DEPENDS
		OPTIONS REQUIRED NAME SRCS ARGN ${ARGN}
  )

	set(TARGET "apps_${NAME}")

  # create target

  nuttx_add_user_library(${TARGET} ${SRCS})

  # add to list of application libraries
	set_property(GLOBAL APPEND PROPERTY NUTTX_APP_LIBS ${TARGET})

  # store parameters into properties (used during builtin list generation)

	set_target_properties(${TARGET} PROPERTIES APP_MAIN ${NAME}_main)
	set_target_properties(${TARGET} PROPERTIES APP_NAME ${NAME})

	if(PRIORITY)
		set_target_properties(${TARGET} PROPERTIES APP_PRIORITY ${PRIORITY})
	else()
		set_target_properties(${TARGET} PROPERTIES APP_PRIORITY SCHED_PRIORITY_DEFAULT)
	endif()

	if(STACKSIZE)
		set_target_properties(${TARGET} PROPERTIES APP_STACK ${STACKSIZE})
	else()
		set_target_properties(${TARGET} PROPERTIES APP_STACK 1024)
  endif()

  # provide main() alias

  list(GET SRCS 0 MAIN_SRC)
  set_property(SOURCE ${MAIN_SRC} APPEND PROPERTY COMPILE_DEFINITIONS main=${NAME}_main)

  # compile options

	if(COMPILE_FLAGS)
		target_compile_options(${TARGET} PRIVATE ${COMPILE_FLAGS})
  endif()

  # add supplied dependencies

	if(DEPENDS)
		# using target_link_libraries for dependencies provides linking
		#  as well as interface include and libraries
		foreach(dep ${DEPENDS})
			get_target_property(dep_type ${dep} TYPE)
			#if (${dep_type} STREQUAL "STATIC_LIBRARY")
			#	target_link_libraries(${TARGET} PRIVATE ${dep})
			#else()
				add_dependencies(${TARGET} ${dep})
			#endif()
		endforeach()
  endif()
endfunction()
