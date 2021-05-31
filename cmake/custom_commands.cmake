# version.sh script to generate .version

add_custom_command(
	OUTPUT ${CMAKE_BINARY_DIR}/.version
	COMMAND tools/version.sh -b ${CONFIG_VERSION_BUILD} -v ${CONFIG_VERSION_STRING}  ${CMAKE_BINARY_DIR}/.version
	WORKING_DIRECTORY ${NUTTX_DIR}
  DEPENDS ${CMAKE_BINARY_DIR}/.config
)

# setup target to generate config.h and version.h from mkconfig and mkversion

add_custom_command(
	OUTPUT
		${CMAKE_BINARY_DIR}/include/nuttx/config.h
		${CMAKE_BINARY_DIR}/include/nuttx/version.h
	COMMAND
		${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/include/nuttx
	COMMAND
		${CMAKE_COMMAND} -E copy ${CMAKE_BINARY_DIR}/config.h ${CMAKE_BINARY_DIR}/include/nuttx/
	COMMAND
		${CMAKE_BINARY_DIR}/bin/mkversion ${CMAKE_BINARY_DIR} > ${CMAKE_BINARY_DIR}/include/nuttx/version.h
	DEPENDS
		nuttx_host_tools
		${CMAKE_BINARY_DIR}/.config
		${CMAKE_BINARY_DIR}/.version
	WORKING_DIRECTORY ${NUTTX_DIR}
)

# Setup symbolic link generation 

add_custom_command(
  COMMENT "Generate symbolic links"
	OUTPUT nuttx_symlinks.stamp

	COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/include
  COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/include_arch/arch
  COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/include_nuttx
  COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/include_apps

  COMMAND ${CMAKE_COMMAND} -E create_symlink ${NUTTX_DIR}/arch/${CONFIG_ARCH}/include ${CMAKE_BINARY_DIR}/include/arch # include/arch
  COMMAND ${CMAKE_COMMAND} -E create_symlink ${NUTTX_DIR}/boards/${NUTTX_BOARD}/include ${CMAKE_BINARY_DIR}/include_arch/arch/board	# include/arch/board
  COMMAND ${CMAKE_COMMAND} -E create_symlink ${NUTTX_DIR}/arch/${CONFIG_ARCH}/include/${CONFIG_ARCH_CHIP} ${CMAKE_BINARY_DIR}/include_arch/arch/chip	# include/arch/chip
  COMMAND ${CMAKE_COMMAND} -E create_symlink ${NUTTX_DIR}/include/nuttx ${CMAKE_BINARY_DIR}/include_nuttx/nuttx # include/nuttx

	COMMAND ${CMAKE_COMMAND} -E touch nuttx_symlinks.stamp
	DEPENDS
		${CMAKE_BINARY_DIR}/.config
)

# Optional symbolic links

# Target used to copy include/nuttx/lib/stdarg.h.  If CONFIG_ARCH_STDARG_H is
# defined, then there is an architecture specific stdarg.h header file
# that will be included indirectly from include/lib/stdarg.h.  But first, we
# have to copy stdarg.h from include/nuttx/. to include/.

if (CONFIG_ARCH_STDARG_H)
  add_custom_command(
    OUTPUT include/stdarg.h
    COMMAND ${CMAKE_COMMAND} -E create_symlink ${NUTTX_DIR}/include/nuttx/lib/stdarg.h ${CMAKE_BINARY_DIR}/include/stdarg.h
  )
else()
  file(REMOVE ${CMAKE_BINARY_DIR}/include/stdarg.h)
endif()

# Target used to copy include/nuttx/lib/math.h.  If CONFIG_ARCH_MATH_H is
# defined, then there is an architecture specific math.h header file
# that will be included indirectly from include/math.h.  But first, we
# have to copy math.h from include/nuttx/. to include/.  Logic within
# include/nuttx/lib/math.h will hand the redirection to the architecture-
# specific math.h header file.
#
# If the CONFIG_LIBM is defined, the Rhombus libm will be built at libc/math.
# Definitions and prototypes for the Rhombus libm are also contained in
# include/nuttx/lib/math.h and so the file must also be copied in that case.
#
# If neither CONFIG_ARCH_MATH_H nor CONFIG_LIBM is defined, then no math.h
# header file will be provided.  You would want that behavior if (1) you
# don't use libm, or (2) you want to use the math.h and libm provided
# within your toolchain.

if (CONFIG_ARCH_MATH_H OR CONFIG_LIBM)
  set(NEED_MATH_H true)
  message(STATUS "Use NuttX math.h: yes")
else()
  message(STATUS "Use NuttX math.h: no")
endif()

if (NEED_MATH_H)
  add_custom_command(
    OUTPUT include/math.h
    COMMAND ${CMAKE_COMMAND} -E create_symlink ${NUTTX_DIR}/include/nuttx/lib/math.h ${CMAKE_BINARY_DIR}/include/math.h
    DEPENDS nuttx_symlinks.stamp
  )
else()
  file(REMOVE ${CMAKE_BINARY_DIR}/include/math.h)
endif()

# The float.h header file defines the properties of your floating point
# implementation.  It would always be best to use your toolchain's float.h
# header file but if none is available, a default float.h header file will
# provided if this option is selected.  However there is no assurance that
# the settings in this float.h are actually correct for your platform!

if (CONFIG_ARCH_FLOAT_H)
  add_custom_command(
    OUTPUT include/float.h
    COMMAND ${CMAKE_COMMAND} -E create_symlink ${NUTTX_DIR}/include/nuttx/lib/float.h ${CMAKE_BINARY_DIR}/include/float.h
    DEPENDS nuttx_symlinks.stamp
  )
else()
  file(REMOVE ${CMAKE_BINARY_DIR}/include/float.h)
endif()

# Target used to copy include/nuttx/lib/setjmp.h.  If CONFIG_ARCH_SETJMP_H is
# defined, then there is an architecture specific setjmp.h header file
# that will be included indirectly from include/lib/setjmp.h.  But first, we
# have to copy setjmp.h from include/nuttx/. to include/.

if (CONFIG_ARCH_SETJMP_H)
  add_custom_command(
    OUTPUT include/setjmp.h
    COMMAND ${CMAKE_COMMAND} -E create_symlink ${NUTTX_DIR}/include/nuttx/lib/setjmp.h ${CMAKE_BINARY_DIR}/include/setjmp.h
    DEPENDS nuttx_symlinks.stamp
  )
else()
  file(REMOVE ${CMAKE_BINARY_DIR}/include/setjmp.h)
endif()

# Add final context target that ties together all of the above
# The context target is invoked on each target build to assure that NuttX is
# properly configured.  The basic configuration steps include creation of the
# the config.h and version.h header files in the include/nuttx directory and
# the establishment of symbolic links to configured directories.

add_custom_target(nuttx_context
	DEPENDS
		nuttx_symlinks.stamp
		${CMAKE_BINARY_DIR}/include/nuttx/config.h
		${CMAKE_BINARY_DIR}/include/nuttx/version.h
    $<$<BOOL:${CONFIG_ARCH_STDARG_H}>:${CMAKE_BINARY_DIR}/include/stdarg.h>
    $<$<BOOL:${NEED_MATH_H}>:${CMAKE_BINARY_DIR}/include/math.h>
    $<$<BOOL:${CONFIG_ARCH_FLOAT_H}>:${CMAKE_BINARY_DIR}/include/float.h>
    $<$<BOOL:${CONFIG_ARCH_SETJMP_H}>:${CMAKE_BINARY_DIR}/include/setjmp.h>
)

