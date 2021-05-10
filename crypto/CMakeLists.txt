if(CONFIG_CRYPTO)
  nuttx_add_kernel_library(crypto)

  set(SRCS crypto.c testmngr.c)

  # cryptodev support

  if (CONFIG_CRYPTO_CRYPTODEV)
    list(APPEND SRCS cryptodev.c)
  endif()

  # Software AES library

  if (CONFIG_CRYPTO_SW_AES)
    list(APPEND SRCS aes.c)
  endif()

  # BLAKE2s hash algorithm

  if (CONFIG_CRYPTO_BLAKE2S)
    list(APPEND SRCS blake2s.c)
  endif()

  # Entropy pool random number generator

  if (CONFIG_CRYPTO_RANDOM_POOL)
    list(APPEND SRCS random_pool.c)
  endif()

  target_sources(crypto PRIVATE ${SRCS})
endif()
