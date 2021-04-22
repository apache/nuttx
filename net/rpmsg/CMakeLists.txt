if (CONFIG_NET_RPMSG)
  target_sources(net PRIVATE rpmsg_sockif.c)
endif()
