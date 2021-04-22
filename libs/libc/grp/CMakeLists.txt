set(SRCS
  lib_getgrgid.c lib_getgrgidr.c lib_getgrnam.c lib_getgrnamr.c
  lib_initgroups.c)

if (CONFIG_LIBC_GROUP_FILE)
  list(APPEND SRCS lib_find_grpfile.c lib_grp_globals.c)
else()
  list(APPEND SRCS lib_getgrbuf.c lib_getgrbufr.c)
endif()

target_sources(c PRIVATE ${SRCS})
