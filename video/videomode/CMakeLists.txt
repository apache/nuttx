if (CONFIG_VIDEO_EDID)
  target_sources(video PRIVATE
    edid_parse.c edid_dump.c
    videomode_lookup.c videomode_sort.c videomode_dump.c
    vesagtf.c)
endif()
