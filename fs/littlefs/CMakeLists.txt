if (CONFIG_FS_LITTLEFS)
  set(LITTLEFS_VERSION 2.4.0)

  FetchContent_Declare(littlefs
    URL https://github.com/ARMmbed/littlefs/archive/v${LITTLEFS_VERSION}.tar.gz
    SOURCE_DIR littlefs
  )
  FetchContent_MakeAvailable(littlefs)

  target_compile_definitions(fs PRIVATE
    -DLFS_TRACE=finfo
    -DLFS_DEBUG=finfo
    -DLFS_WARN=fwarn
    -DLFS_ERROR=ferr
    -DLFS_ASSERT=DEBUGASSERT)

  FetchContent_GetProperties(littlefs SOURCE_DIR LITTLEFS_SOURCE_DIR)

  target_sources(fs PRIVATE
    lfs_vfs.c
    ${LITTLEFS_SOURCE_DIR}/lfs.c ${LITTLEFS_SOURCE_DIR}/lfs_util.c)

  target_include_directories(fs PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
endif()
