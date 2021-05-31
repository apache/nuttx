if (CONFIG_IEEE802154_XBEE)
  target_sources(drivers PRIVATE
    xbee_ioctl.c xbee_mac.c xbee_netdev.c xbee.c)
endif()
