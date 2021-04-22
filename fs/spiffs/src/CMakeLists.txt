if(CONFIG_FS_SPIFFS)
  target_sources(fs PRIVATE
    spiffs_vfs.c spiffs_volume.c spiffs_core.c spiffs_gc.c
    spiffs_cache.c spiffs_check.c spiffs_mtd.c)
endif()
