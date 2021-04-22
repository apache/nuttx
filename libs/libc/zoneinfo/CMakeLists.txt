if (CONFIG_LIB_ZONEINFO_ROMFS)
  message(FATAL_ERROR "zoneinfo romfs not supported yet")

#  tzcode:
#    mkdir tzcode

#  tzbin:
#    mkdir tzbin

#  tzcode-latest.tar.gz:
#    $(Q) curl -L -O ftp://ftp.iana.org/tz/tzcode-latest.tar.gz

#  tzdata-latest.tar.gz:
#    $(Q) curl -L -O ftp://ftp.iana.org/tz/tzdata-latest.tar.gz

#  .tzunpack: tzcode tzcode-latest.tar.gz tzdata-latest.tar.gz
#    $(Q) tar zx -C tzcode -f tzcode-latest.tar.gz
#    $(Q) tar zx -C tzcode -f tzdata-latest.tar.gz
#    $(Q) touch .tzunpack

#  .tzbuilt: tzcode tzbin .tzunpack
#    $(Q) $(MAKE) -C tzcode TOPDIR=$(TZBIN_PATH) install
#    $(Q) touch .tzbuilt

  add_custom_command(
    OUTPUT romfs_zoneinfo.h
    COMMAND ${GENROMFS} -f romfs_zoneinfo.img -d ${CMAKE_CURRENT_SOURCE_DIR}/tzbin/usr/share/zoneinfo -V "TZDatbase"
    COMMAND ${XXD} -i romfs_zoneinfo.img > romfs_zoneinfo.h)

  target_sources(c PRIVATE tzromfs.c)
endif()
