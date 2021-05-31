if (CONFIG_IEEE802154_MRF24J40)
  target_sources(drivers PRIVATE
    mrf24j40_getset.c mrf24j40_interrupt.c mrf24j40_radif.c
    mrf24j40_regops.c mrf24j40.c)
endif()
