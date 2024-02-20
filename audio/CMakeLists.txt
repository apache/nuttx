# ##############################################################################
# audio/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
#
# ##############################################################################

if(CONFIG_AUDIO)
  nuttx_add_kernel_library(audio)

  set(SRCS audio.c)

  if(CONFIG_AUDIO_COMP)
    list(APPEND SRCS audio_comp.c)
  endif()

  if(CONFIG_AUDIO_FORMAT_PCM)
    list(APPEND SRCS pcm_decode.c)
  endif()

  target_sources(audio PRIVATE ${SRCS})
endif()
