if (CONFIG_IEEE80211_BROADCOM_FULLMAC)
  set(SRCS)

  list(APPEND SRCS
    bcmf_driver.c bcmf_cdc.c bcmf_bdc.c bcmf_utils.c bcmf_netdev.c)

  if (CONFIG_IEEE80211_BROADCOM_FULLMAC_SDIO)
    list(APPEND SRCS mmc_sdio.c bcmf_sdio.c bcmf_core.c bcmf_sdpcm.c)
  endif()

  if (CONFIG_IEEE80211_BROADCOM_BCM43362)
    list(APPEND SRCS bcmf_chip_43362.c)
  endif()

  if (CONFIG_IEEE80211_BROADCOM_BCM43438)
    list(APPEND SRCS bcmf_chip_43438.c)
  endif()

  target_sources(drivers PRIVATE ${SRCS})
endif()
