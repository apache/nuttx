
if(NOT "${CONFIG_DISABLE_ENVIRON}" STREQUAL "y")

	set(CSRCS)
	list(APPEND CSRCS
		env_clearenv.c
		env_dup.c
		env_findvar.c
		env_foreach.c
		env_getenv.c
		env_getenvironptr.c
		env_putenv.c
		env_release.c
		env_removevar.c
		env_setenv.c
		env_unsetenv.c
	)
	set_parent_srcs(${CSRCS})

endif()
