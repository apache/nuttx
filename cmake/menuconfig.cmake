# menuconfig target
# this triggers a reconfiguration (TODO: do only if config changes)

add_custom_target(menuconfig
  COMMAND
		KCONFIG_CONFIG=${CMAKE_BINARY_DIR}/.config EXTERNALDIR=dummy DRIVERS_PLATFORM_DIR=dummy APPSDIR=${NUTTX_APPS_DIR}
    HOST_LINUX=$<IF:$<BOOL:{LINUX}>,y,n> HOST_MACOS=$<IF:$<BOOL:${APPLE}>,y,n> HOST_WINDOWS=$<IF:$<BOOL:${WIN32}>,y,n> HOST_OTHER=$<IF:$<BOOL:${OTHER_OS}>,y,n>
      kconfig-mconf Kconfig
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
		KCONFIG_CONFIG=${CMAKE_BINARY_DIR}/.config EXTERNALDIR=dummy DRIVERS_PLATFORM_DIR=dummy APPSDIR=${NUTTX_APPS_DIR}
    HOST_LINUX=$<IF:$<BOOL:{LINUX}>,y,n> HOST_MACOS=$<IF:$<BOOL:${APPLE}>,y,n> HOST_WINDOWS=$<IF:$<BOOL:${WIN32}>,y,n> HOST_OTHER=$<IF:$<BOOL:${OTHER_OS}>,y,n>
      kconfig-qconf Kconfig
	COMMAND
		${CMAKE_COMMAND} -E remove -f ${CMAKE_BINARY_DIR}/include/nuttx/config.h	# invalidate existing config
  COMMAND
    ${CMAKE_COMMAND} -E touch ${CMAKE_PARENT_LIST_FILE}
	WORKING_DIRECTORY ${NUTTX_DIR}
	USES_TERMINAL
)

# utility target to restore .config from board's defconfig
add_custom_target(resetconfig
  COMMAND ${CMAKE_COMMAND} -E copy ${NUTTX_DEFCONFIG} ${CMAKE_BINARY_DIR}/.config
  COMMAND
    KCONFIG_CONFIG=${CMAKE_BINARY_DIR}/.config EXTERNALDIR=dummy DRIVERS_PLATFORM_DIR=dummy APPSDIR=${NUTTX_APPS_DIR}
    HOST_LINUX=$<IF:$<BOOL:{LINUX}>,y,n> HOST_MACOS=$<IF:$<BOOL:${APPLE}>,y,n> HOST_WINDOWS=$<IF:$<BOOL:${WIN32}>,y,n> HOST_OTHER=$<IF:$<BOOL:${OTHER_OS}>,y,n>
      kconfig-conf --olddefconfig Kconfig
  COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_BINARY_DIR}/.config ${CMAKE_BINARY_DIR}/.config.orig
  COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_PARENT_LIST_FILE}
  WORKING_DIRECTORY ${NUTTX_DIR}
)

add_custom_target(diffconfig
  COMMAND kconfig-diff .config.orig .config
)
