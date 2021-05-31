if (CONFIG_LIBC_DLFCN)
  target_sources(c PRIVATE
    lib_dlopen.c lib_dlclose.c lib_dlsym.c lib_dlerror.c lib_dlsymtab.c)
endif()
