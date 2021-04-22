# Define NUTTX_LIBRARIES to hold all NuttX internal libraries to be linked

define_property(GLOBAL PROPERTY NUTTX_LIBRARIES
                BRIEF_DOCS "NuttX libs"
                FULL_DOCS "List of all NuttX libraries"
)

define_property(GLOBAL PROPERTY NUTTX_USER_LIBRARIES
                BRIEF_DOCS "NuttX user libs"
                FULL_DOCS "List of all NuttX user libraries"
)

# TODO: call this once at top level without sources and then use target_sources()
# when needed

function(nuttx_add_library_internal target)
  # ensure nuttx_context is created before this
	add_dependencies(${target} nuttx_context)

  # add main include directories
  target_include_directories(${target} SYSTEM PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_BINARY_DIR}/include
    ${CMAKE_BINARY_DIR}/include_arch
  )

  # set global compile options & definitions
  get_property(options GLOBAL PROPERTY NUTTX_COMPILE_OPTIONS)
  target_compile_options(${target} PRIVATE ${options})

  get_property(definitions GLOBAL PROPERTY NUTTX_DEFINITIONS)
  target_compile_definitions(${target} PRIVATE ${definitions})
endfunction()

function(nuttx_add_user_library target)
  # declare target
	add_library(${target} OBJECT ${ARGN})

  nuttx_add_library_internal(${target} ${ARGN})

  # link to final libapps
  target_link_libraries(apps ${target})

  # add apps/include to include path
  target_include_directories(${target} PRIVATE ${NUTTX_APPS_ABS_DIR}/include)
endfunction()

function(nuttx_add_library target)

  # declare target
	add_library(${target} ${ARGN})

  # add library to build
  nuttx_add_library_internal(${target} ${ARGN})

  # add kernel options & definitions
  get_property(definitions GLOBAL PROPERTY NUTTX_KERNEL_DEFINITIONS)
  target_compile_definitions(${target} PRIVATE ${definitions})

  get_property(options GLOBAL PROPERTY NUTTX_KERNEL_COMPILE_OPTIONS)
  target_compile_options(${target} PRIVATE ${options})

  # add to list of libraries to link to final nuttx binary
	set_property(GLOBAL APPEND PROPERTY NUTTX_LIBRARIES ${target})

  # install to library dir
  install(TARGETS ${target} DESTINATION lib)
endfunction()

