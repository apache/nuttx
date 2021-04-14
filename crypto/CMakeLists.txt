if("${CONFIG_CRYPTO}" STREQUAL "y")

	set(CSRCS)
	list(APPEND CSRCS crypto.c testmngr.c)

	# cryptodev support

	if ("${CONFIG_CRYPTO_CRYPTODEV}" STREQUAL "y")
		list(APPEND CSRCS cryptodev.c)
	endif()

	# Software AES library

	if ("${CONFIG_CRYPTO_SW_AES}" STREQUAL "y")
		list(APPEND CSRCS aes.c)
	endif()

	# BLAKE2s hash algorithm

	if ("${CONFIG_CRYPTO_BLAKE2S}" STREQUAL "y")
		list(APPEND CSRCS blake2s.c)
	endif

	# Entropy pool random number generator

	if ("${CONFIG_CRYPTO_RANDOM_POOL}" STREQUAL "y")
		list(APPEND CSRCS random_pool.c)
	endif()
	
	if(CSRCS)
		nuttx_add_library(crypto ${CSRCS})
	endif()
endif()
