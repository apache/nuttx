if (CONFIG_LIB_ZONEINFO_ROMFS)
  FetchContent_Declare(tzcode URL http://ftp.iana.org/tz/tzcode-latest.tar.gz)
  FetchContent_Declare(tzdata URL http://ftp.iana.org/tz/tzdata-latest.tar.gz)
  FetchContent_Populate(tzcode)
  FetchContent_Populate(tzdata)

  file(COPY ${tzcode_SOURCE_DIR}/ DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/tz)
  file(COPY ${tzdata_SOURCE_DIR}/ DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/tz)

  execute_process(COMMAND make -C ${CMAKE_CURRENT_BINARY_DIR}/tz install TOPDIR=${CMAKE_CURRENT_BINARY_DIR}/tzbin)

  nuttx_add_romfs(
    NAME zoneinfo
    PATH ${CMAKE_CURRENT_BINARY_DIR}/tzbin/usr/share/zoneinfo
    HEADER
    PREFIX zoneinfo
    NONCONST)

  set_property(SOURCE romfs_zoneinfo.h PROPERTY GENERATED)

  nuttx_add_aux_library(zoneinfo tzromfs.c)
  target_include_directories(zoneinfo PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
  target_link_libraries(c PRIVATE zoneinfo)
endif()
