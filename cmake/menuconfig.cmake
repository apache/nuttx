# menuconfig target

add_custom_target(menuconfig
	COMMAND
		${CMAKE_COMMAND} -E copy_if_different ${CMAKE_BINARY_DIR}/.config .config
	COMMAND
		APPSDIR=${NUTTX_APPS_DIR} kconfig-mconf Kconfig
	COMMAND
		${CMAKE_COMMAND} -E copy_if_different .config ${CMAKE_BINARY_DIR}/.config
	COMMAND
		${CMAKE_COMMAND} -E remove -f ${CMAKE_BINARY_DIR}/include/nuttx/config.h	# invalidate existing config
	DEPENDS
		${CMAKE_BINARY_DIR}/.config
	WORKING_DIRECTORY ${NUTTX_DIR}
	USES_TERMINAL
)

# qconfig target

add_custom_target(qconfig
	COMMAND
		${CMAKE_COMMAND} -E copy_if_different ${CMAKE_BINARY_DIR}/.config .config
	COMMAND
		APPSDIR=${NUTTX_APPS_DIR} kconfig-qconf Kconfig
	COMMAND
		${CMAKE_COMMAND} -E copy_if_different .config ${CMAKE_BINARY_DIR}/.config	# copy back to binary directory
	COMMAND
		${CMAKE_COMMAND} -E remove -f ${CMAKE_BINARY_DIR}/include/nuttx/config.h	# invalidate existing config
	DEPENDS
		${CMAKE_BINARY_DIR}/.config
	WORKING_DIRECTORY ${NUTTX_DIR}
	USES_TERMINAL
)

# savedefconfig
add_custom_target(savedefconfig
	COMMAND
		${CMAKE_COMMAND} -E copy_if_different ${CMAKE_BINARY_DIR}/.config .config
	COMMAND
		APPSDIR=${NUTTX_APPS_DIR} kconfig-conf --savedefconfig defconfig.tmp Kconfig
	COMMAND
		sed -i -e "/CONFIG_APPS_DIR=/d" defconfig.tmp			# remove CONFIG_APPS_DIR
	COMMAND
		grep "CONFIG_ARCH=" .config >> defconfig.tmp || true		# preserve CONFIG_ARCH=
	COMMAND
		grep "^CONFIG_ARCH_CHIP_" .config >> defconfig.tmp || true	# preserve CONFIG_ARCH_CHIP_
	COMMAND
		grep "CONFIG_ARCH_BOARD=" .config >> defconfig.tmp || true	# preserve CONFIG_ARCH_BOARD
	COMMAND
		grep "^CONFIG_ARCH_CUSTOM" .config >> defconfig.tmp || true	# preserve CONFIG_ARCH_CUSTOM
	COMMAND
		#sed -i -e "/^#/d" defconfig.tmp					# remove commented lines
	COMMAND
		cat defconfig.tmp | LC_ALL=C sort | uniq > defconfig.updated	# sort and save back to original defconfig
	COMMAND
		${CMAKE_COMMAND} -E copy_if_different defconfig.updated ${NUTTX_DEFCONFIG}
	COMMAND
		${CMAKE_COMMAND} -E remove -f defconfig.tmp defconfig.updated			# cleanup
	DEPENDS
		${CMAKE_BINARY_DIR}/.config
	COMMENT "Compressing .config and saving back to ${NUTTX_DEFCONFIG}"
	WORKING_DIRECTORY ${NUTTX_DIR}
)

# menuconfig and save
add_custom_target(menuconfig_save
	COMMAND
		${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR} -- savedefconfig # this is hacky, but forces menuconfig to run before savedefconfig
	DEPENDS menuconfig
)
# qconfig and save
add_custom_target(qconfig_save
	COMMAND
		${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR} -- savedefconfig # this is hacky, but forces qconfig to run before savedefconfig
	DEPENDS qconfig
)
