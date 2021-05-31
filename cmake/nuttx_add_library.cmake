# Internal utility function
#
# Used by functions below, not to be used directly

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
#
# The whole purpose of this is to overcome the limitation of CMake 3.16
# to set source file properties from directories different from the one
# defining the target where the source file is used. This auxiliary
# library acts as an intermediate target that is usually linked to the
# system/kernel library defined at a higher level.

function(nuttx_add_aux_library target)
  # declare target
	add_library(${target} OBJECT ${ARGN})

  nuttx_add_library_internal(${target} ${ARGN})
endfunction()

# User (application) libraries
#
# An user library is a target which is defined as a collection of object files
# which is ultimately archived into the apps library

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
#
# A system library is a library which is built into the OS but does not receive
# kernel-level flags (such as __KERNEL__). This is will be part of the userspace
# blob in kernel builds

function(nuttx_add_system_library target)
  # declare target
	add_library(${target} ${ARGN})

  # add library to build
  nuttx_add_library_internal(${target} ${ARGN})

  # add to list of libraries to link to final nuttx binary
	set_property(GLOBAL APPEND PROPERTY NUTTX_SYSTEM_LIBRARIES ${target})

  # install to library dir
  install(TARGETS ${target} DESTINATION lib)
endfunction()

# Kernel Libraries
#
# nuttx_add_kernel_library(target [SPLIT] [SAME_SOURCES] [sources ...])
#
# For non-kernel builds, this defines an OS library which will receive
# kernel-level flags (such as __KERNEL__) and will be linked into nuttx binary
# For kernel builds, the same happens unless SPLIT is specified. In this
# case both a <target> and a k<target> library will be defined,
# but only the latter having the kernel-level flags. In this case,
# both libraries will receive the same set of sources (the original <target>
# should be used by the user to add sources).

function(nuttx_add_kernel_library target)
  cmake_parse_arguments(ARGS SPLIT "" "" ${ARGN})
  set(SRCS ${ARGS_UNPARSED_ARGUMENTS})

  if (ARGS_SPLIT AND NOT CONFIG_BUILD_FLAT)
    set(kernel_target k${target})

    # add non-kernel (system) library
    nuttx_add_system_library(${target} ${SRCS})
  else()
    set(kernel_target ${target})
  endif()

  # add kernel library
  add_library(${kernel_target} ${SRCS})
  nuttx_add_library_internal(${kernel_target} ${SRCS})
  set_property(GLOBAL APPEND PROPERTY NUTTX_KERNEL_LIBRARIES ${kernel_target})

  # Add kernel options & definitions
  # See note above in nuttx_add_library_internal()
  # on syntax and nuttx target use
  target_compile_options(${kernel_target} PRIVATE $<GENEX_EVAL:$<TARGET_PROPERTY:nuttx,NUTTX_KERNEL_COMPILE_OPTIONS>>)
  target_compile_definitions(${kernel_target} PRIVATE $<GENEX_EVAL:$<TARGET_PROPERTY:nuttx,NUTTX_KERNEL_DEFINITIONS>>)
  target_include_directories(${kernel_target} PRIVATE $<GENEX_EVAL:$<TARGET_PROPERTY:nuttx,NUTTX_KERNEL_INCLUDE_DIRECTORIES>>)

  if (NOT "${target}" STREQUAL "${kernel_target}")
    # The k${target} lib will have the same sources added to that ${target}
    # lib. This allows to do target_sources(${target} ..) easily
    target_sources(${kernel_target} PRIVATE $<TARGET_PROPERTY:${target},SOURCES>)

    # same for include directories
    target_include_directories(${kernel_target} PRIVATE $<TARGET_PROPERTY:${target},INCLUDE_DIRECTORIES>)
  endif()
endfunction()

