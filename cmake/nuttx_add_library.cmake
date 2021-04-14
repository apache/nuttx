# Define NUTTX_LIBRARIES to hold all NuttX internal libraries to be linked

define_property(GLOBAL PROPERTY NUTTX_LIBRARIES
                BRIEF_DOCS "NuttX libs"
                FULL_DOCS "List of all NuttX libraries"
)

# Wrapper of cmake add_library which adds appropriate internal dependencies
# and adds the entry into NUTTX_LIBRARIES 

# TODO: call this once at top level without sources and then use target_sources()
# when needed

function(nuttx_add_library target)
	add_library(${target} ${ARGN})

	add_dependencies(${target} nuttx_context)

	set_property(GLOBAL APPEND PROPERTY NUTTX_LIBRARIES ${target})
endfunction()
