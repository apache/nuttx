# version.sh script to generate .version

# TODO
# set(VERSION_ARGS "-v ${CONFIG_VERSION_STRING} -b ${CONFIG_VERSION_BUILD}")

add_custom_command(
	OUTPUT ${CMAKE_BINARY_DIR}/.version
	COMMAND tools/version.sh ${VERSION_ARGS} ${CMAKE_BINARY_DIR}/.version
#	DEPENDS ${NUTTX_DIR}/configs/${NUTTX_BOARD}/${NUTTX_CONFIG}/defconfig
	WORKING_DIRECTORY ${NUTTX_DIR}
)

include(ExternalProject)
ExternalProject_Add(nuttx_host_tools
	SOURCE_DIR ${CMAKE_SOURCE_DIR}/tools
	INSTALL_DIR ${CMAKE_BINARY_DIR}
	CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}
	USES_TERMINAL_CONFIGURE true
	USES_TERMINAL_BUILD true
	USES_TERMINAL_INSTALL true
)

# setup target to generate config.h and version.h from mkconfig and mkversion

add_custom_command(
	OUTPUT
		${CMAKE_BINARY_DIR}/include/nuttx/config.h
		${CMAKE_BINARY_DIR}/include/nuttx/version.h
	COMMAND
		${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/include/nuttx
	COMMAND
		${CMAKE_BINARY_DIR}/bin/mkconfig ${CMAKE_BINARY_DIR} > ${CMAKE_BINARY_DIR}/include/nuttx/config.h
	COMMAND
		${CMAKE_BINARY_DIR}/bin/mkversion ${CMAKE_BINARY_DIR} > ${CMAKE_BINARY_DIR}/include/nuttx/version.h
	DEPENDS
		nuttx_host_tools
#		${CMAKE_BINARY_DIR}/.config
		${CMAKE_BINARY_DIR}/.version
	WORKING_DIRECTORY ${NUTTX_DIR}
)

# Generate .config, from refreshed board defconfig

#add_custom_command(
#	OUTPUT
#		${CMAKE_BINARY_DIR}/.config
#	COMMAND
#		${CMAKE_COMMAND} -E copy_if_different ${NUTTX_DEFCONFIG} ${NUTTX_DIR}/.config
#	COMMAND
#		APPSDIR=${NUTTX_APPS_DIR} kconfig-conf --olddefconfig Kconfig
#	COMMAND
#		${CMAKE_COMMAND} -E copy_if_different ${NUTTX_DIR}/.config ${CMAKE_BINARY_DIR}/.config
##	DEPENDS ${NUTTX_DEFCONFIG}
#	WORKING_DIRECTORY ${NUTTX_DIR}
#)

# Setup symbolic link generation 

add_custom_command(
  COMMENT "Generate symbolic links"
	OUTPUT nuttx_symlinks.stamp
	#COMMAND ${CMAKE_COMMAND} -E remove
	#${NUTTX_DIR}/include/arch
	#${NUTTX_DIR}/arch/${CONFIG_ARCH}/include/board
	#${NUTTX_DIR}/arch/${CONFIG_ARCH}/include/chip
	#${NUTTX_DIR}/arch/${CONFIG_ARCH}/src/board
	#${NUTTX_DIR}/arch/${CONFIG_ARCH}/src/chip

	#COMMAND ${CMAKE_COMMAND} -E create_symlink ${NUTTX_DIR}/arch/${CONFIG_ARCH}/include			${NUTTX_DIR}/include/arch			# include/arch
	#COMMAND ${CMAKE_COMMAND} -E create_symlink ${NUTTX_DIR}/configs/${NUTTX_BOARD}/include			${NUTTX_DIR}/arch/${CONFIG_ARCH}/include/board	# arch/arm/include/board
	#COMMAND ${CMAKE_COMMAND} -E create_symlink ${NUTTX_DIR}/arch/${CONFIG_ARCH}/include/${CONFIG_ARCH_CHIP}	${NUTTX_DIR}/arch/${CONFIG_ARCH}/include/chip	# arch/arm/include/chip
	#COMMAND ${CMAKE_COMMAND} -E create_symlink ${NUTTX_DIR}/configs/${NUTTX_BOARD}/src			${NUTTX_DIR}/arch/${CONFIG_ARCH}/src/board	# arch/arm/src/board
	#COMMAND ${CMAKE_COMMAND} -E create_symlink ${NUTTX_DIR}/arch/${CONFIG_ARCH}/src/${CONFIG_ARCH_CHIP}	${NUTTX_DIR}/arch/${CONFIG_ARCH}/src/chip	# arch/arm/src/chip

	COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/include

	COMMAND ${CMAKE_COMMAND} -E create_symlink ${NUTTX_DIR}/arch/${CONFIG_ARCH}/include ${CMAKE_BINARY_DIR}/include/arch # include/arch
	COMMAND ${CMAKE_COMMAND} -E create_symlink ${NUTTX_DIR}/boards/${NUTTX_BOARD}/include ${CMAKE_BINARY_DIR}/include/arch/board	# include/arch/board
	COMMAND ${CMAKE_COMMAND} -E create_symlink ${NUTTX_DIR}/arch/${CONFIG_ARCH}/include/${CONFIG_ARCH_CHIP} ${CMAKE_BINARY_DIR}/include/arch/chip	# include/arch/chip

	COMMAND ${CMAKE_COMMAND} -E touch nuttx_symlinks.stamp
	DEPENDS
		${CMAKE_BINARY_DIR}/.config
)

# add final context target that ties together all of the above

add_custom_target(nuttx_context
	DEPENDS
		nuttx_symlinks.stamp
		${CMAKE_BINARY_DIR}/include/nuttx/config.h
		${CMAKE_BINARY_DIR}/include/nuttx/version.h
)

