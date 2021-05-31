if (CONFIG_AUDIO_SRC)
  set(LIBSRC_VERSION 0.1.9)

  FetchContent_Declare(libsrc
    URL https://codeload.github.com/libsndfile/libsamplerate/zip/master)
  FetchContent_Populate(libsrc)
  FetchContent_GetProperties(libsrc)

  set(LIBSRC_DEFINITIONS -DPACKAGE="libsamplerate" -DVERSION="${LIBSRC_VERSION}")

  if (CONFIG_SINC_FAST_CONVERTER)
    list(APPEND LIBSRC_DEFINITIONS -DENABLE_SINC_FAST_CONVERTER)
  endif()

  if (CONFIG_SINC_MEDIUM_CONVERTER)
    list(APPEND LIBSRC_DEFINITIONS -DENABLE_SINC_MEDIUM_CONVERTER)
  endif()

  if (CONFIG_SINC_BEST_CONVERTER)
    list(APPEND LIBSRC_DEFINITIONS -DENABLE_SINC_BEST_CONVERTER)
  endif()

  set(SRCS samplerate.c src_sinc.c src_linear.c src_zoh.c)
  list(TRANSFORM SRCS PREPEND ${libsrc_SOURCE_DIR}/src/)
  target_sources(c PRIVATE ${SRCS})

  target_compile_definitions(c PRIVATE ${LIBSRC_DEFINITIONS})
  target_include_directories(c PRIVATE ${libsrc_SOURCE_DIR}/include)
endif()
