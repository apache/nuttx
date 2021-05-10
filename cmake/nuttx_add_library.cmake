# Define NUTTX_LIBRARIES to hold all NuttX internal libraries to be linked

define_property(GLOBAL PROPERTY NUTTX_LIBRARIES
                BRIEF_DOCS "NuttX libs"
                FULL_DOCS "List of all NuttX libraries"
)

define_property(GLOBAL PROPERTY NUTTX_USER_LIBRARIES
                BRIEF_DOCS "NuttX user libs"
                FULL_DOCS "List of all NuttX user libraries"
)

# Internal utility function

function(nuttx_add_library_internal target)
  # ensure nuttx_context is created before this
	add_dependencies(${target} nuttx_context)

  # add main include directories
  target_include_directories(${target} SYSTEM PUBLIC
    $<$<BOOL:${CONFIG_HAVE_CXX}>:${CMAKE_BINARY_DIR}/include_cxx>
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_BINARY_DIR}/include
    ${CMAKE_BINARY_DIR}/include_arch
  )

  # Set global compile options & definitions
  # We use the "nuttx" target to hold these properties
  # so that libraries added after this property is set can read
  # the final value at build time. The GENEX_EVAL allows the property
  # to hold generator expression itself
  target_compile_options(${target} PRIVATE $<GENEX_EVAL:$<TARGET_PROPERTY:nuttx,NUTTX_COMPILE_OPTIONS>>)
  target_compile_definitions(${target} PRIVATE $<GENEX_EVAL:$<TARGET_PROPERTY:nuttx,NUTTX_DEFINITIONS>>)
  target_include_directories(${target} PRIVATE $<GENEX_EVAL:$<TARGET_PROPERTY:nuttx,NUTTX_INCLUDE_DIRECTORIES>>)
endfunction()

# Auxiliary libraries
# The whole purpose of this is to overcome the limitation of CMake 3.16
# to set source file properties from directories different from the one
# defining the target where the source file is used. This auxiliary
# library acts as an intermediate target the is usually linked to the
# system/kernel library defined at a higher level.

function(nuttx_add_aux_library target)
  # declare target
	add_library(${target} OBJECT ${ARGN})

  nuttx_add_library_internal(${target} ${ARGN})
endfunction()

# User (application) libraries

function(nuttx_add_user_library target)
  # declare target
	add_library(${target} OBJECT ${ARGN})

  nuttx_add_library_internal(${target} ${ARGN})

  # link to final libapps
  target_link_libraries(apps PRIVATE ${target})

  # add apps/include to include path
  target_include_directories(${target} PRIVATE ${NUTTX_APPS_ABS_DIR}/include)
endfunction()

# System Libraries

function(nuttx_add_library target)
  # declare target
	add_library(${target} ${ARGN})

  # add library to build
  nuttx_add_library_internal(${target} ${ARGN})

  # add to list of libraries to link to final nuttx binary
	set_property(GLOBAL APPEND PROPERTY NUTTX_LIBRARIES ${target})

  # install to library dir
  install(TARGETS ${target} DESTINATION lib)
endfunction()

# Kernel Libraries

function(nuttx_add_kernel_library target)
  nuttx_add_library(${target} ${ARGN})

  # Add kernel options & definitions
  # See note above in nuttx_add_library_internal()
  # on syntax and nutx target use
  target_compile_options(${target} PRIVATE $<GENEX_EVAL:$<TARGET_PROPERTY:nuttx,NUTTX_KERNEL_COMPILE_OPTIONS>>)
  target_compile_definitions(${target} PRIVATE $<GENEX_EVAL:$<TARGET_PROPERTY:nuttx,NUTTX_KERNEL_DEFINITIONS>>)
  target_include_directories(${target} PRIVATE $<GENEX_EVAL:$<TARGET_PROPERTY:nuttx,NUTTX_KERNEL_INCLUDE_DIRECTORIES>>)
endfunction()
