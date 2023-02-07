# ##############################################################################
# crypto/CMakeLists.txt
#
# Licensed to the Apache Software Foundation (ASF) under one or more contributor
# license agreements.  See the NOTICE file distributed with this work for
# additional information regarding copyright ownership.  The ASF licenses this
# file to you under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License.  You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations under
# the License.
#
# ##############################################################################
if(CONFIG_CRYPTO)
  nuttx_add_kernel_library(crypto)

  set(SRCS crypto.c testmngr.c)

  # cryptodev support

  if(CONFIG_CRYPTO_CRYPTODEV)
    list(APPEND SRCS cryptodev.c)
  endif()

  # Software AES library

  if(CONFIG_CRYPTO_SW_AES)
    list(APPEND SRCS aes.c)
  endif()

  # BLAKE2s hash algorithm

  if(CONFIG_CRYPTO_BLAKE2S)
    list(APPEND SRCS blake2s.c)
  endif()

  # Entropy pool random number generator

  if(CONFIG_CRYPTO_RANDOM_POOL)
    list(APPEND SRCS random_pool.c)
  endif()

  target_sources(crypto PRIVATE ${SRCS})
endif()
