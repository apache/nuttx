
if(NOT CONFIG_DISABLE_ENVIRON)
  target_sources(sched PRIVATE
    env_getenvironptr.c env_dup.c env_release.c env_findvar.c
    env_removevar.c env_clearenv.c env_getenv.c env_putenv.c
    env_setenv.c env_unsetenv.c env_foreach.c)
endif()
