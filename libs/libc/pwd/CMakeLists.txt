set(SRCS lib_getpwnam.c lib_getpwnamr.c lib_getpwuid.c lib_getpwuidr.c)

if (CONFIG_LIBC_PASSWD_FILE)
  list(APPEND SRCS lib_find_pwdfile.c lib_pwd_globals.c)
else()
  list(APPEND SRCS lib_getpwbuf.c lib_getpwbufr.c)
endif()

target_sources(c PRIVATE ${SRCS})
