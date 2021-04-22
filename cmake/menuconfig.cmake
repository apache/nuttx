# menuconfig target
# this triggers a reconfiguration (TODO: do only if config changes)

add_custom_target(menuconfig
  COMMAND
		KCONFIG_CONFIG=${CMAKE_BINARY_DIR}/.config EXTERNALDIR=dummy DRIVERS_PLATFORM_DIR=dummy APPSDIR=${NUTTX_APPS_DIR} kconfig-mconf Kconfig
	COMMAND
		${CMAKE_COMMAND} -E remove -f ${CMAKE_BINARY_DIR}/include/nuttx/config.h	# invalidate existing config
  COMMAND
    ${CMAKE_COMMAND} -E touch ${CMAKE_PARENT_LIST_FILE}
	WORKING_DIRECTORY ${NUTTX_DIR}
	USES_TERMINAL
)

# qconfig target

add_custom_target(qconfig
  COMMAND
		KCONFIG_CONFIG=${CMAKE_BINARY_DIR}/.config EXTERNALDIR=dummy DRIVERS_PLATFORM_DIR=dummy APPSDIR=${NUTTX_APPS_DIR} kconfig-qconf Kconfig
	COMMAND
		${CMAKE_COMMAND} -E remove -f ${CMAKE_BINARY_DIR}/include/nuttx/config.h	# invalidate existing config
  COMMAND
    ${CMAKE_COMMAND} -E touch ${CMAKE_PARENT_LIST_FILE}
	WORKING_DIRECTORY ${NUTTX_DIR}
	USES_TERMINAL
)

# savedefconfig
# TODO: update
#add_custom_target(savedefconfig
#	#COMMAND
#	#	${CMAKE_COMMAND} -E copy_if_different ${CMAKE_BINARY_DIR}/.config .config
#	COMMAND
#		KCONFIG_CONFIG=${CMAKE_BINARY_DIR}/.config APPSDIR=${NUTTX_APPS_DIR} kconfig-conf --savedefconfig defconfig.tmp Kconfig
#	COMMAND
#		sed -i -e "/CONFIG_APPS_DIR=/d" defconfig.tmp			# remove CONFIG_APPS_DIR
#	COMMAND
#		grep "CONFIG_ARCH=" .config >> defconfig.tmp || true		# preserve CONFIG_ARCH=
#	COMMAND
#		grep "^CONFIG_ARCH_CHIP_" .config >> defconfig.tmp || true	# preserve CONFIG_ARCH_CHIP_
#	COMMAND
#		grep "CONFIG_ARCH_BOARD=" .config >> defconfig.tmp || true	# preserve CONFIG_ARCH_BOARD
#	COMMAND
#		grep "^CONFIG_ARCH_CUSTOM" .config >> defconfig.tmp || true	# preserve CONFIG_ARCH_CUSTOM
#	COMMAND
#		#sed -i -e "/^#/d" defconfig.tmp					# remove commented lines
#	COMMAND
#		cat defconfig.tmp | LC_ALL=C sort | uniq > defconfig.updated	# sort and save back to original defconfig
#	COMMAND
#		${CMAKE_COMMAND} -E copy_if_different defconfig.updated ${NUTTX_DEFCONFIG}
#	COMMAND
#		${CMAKE_COMMAND} -E remove -f defconfig.tmp defconfig.updated			# cleanup
#	DEPENDS
#		${CMAKE_BINARY_DIR}/.config
#	COMMENT "Compressing .config and saving back to ${NUTTX_DEFCONFIG}"
#	WORKING_DIRECTORY ${NUTTX_DIR}
#)

# utility target to restore .config from board's defconfig
add_custom_target(resetconfig
  COMMAND ${CMAKE_COMMAND} -E copy ${NUTTX_DEFCONFIG} ${CMAKE_BINARY_DIR}/.config
  COMMAND KCONFIG_CONFIG=${CMAKE_BINARY_DIR}/.config EXTERNALDIR=dummy DRIVERS_PLATFORM_DIR=dummy APPSDIR=${NUTTX_APPS_DIR} kconfig-conf --olddefconfig Kconfig
  COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_BINARY_DIR}/.config ${CMAKE_BINARY_DIR}/.config.orig
  COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_PARENT_LIST_FILE}
  WORKING_DIRECTORY ${NUTTX_DIR}
)

add_custom_target(diffconfig
  COMMAND kconfig-diff .config.orig .config
)
