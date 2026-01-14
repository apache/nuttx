# ##############################################################################
# crypto/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
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
    if(CONFIG_CRYPTO_CRYPTODEV_SOFTWARE)
      list(APPEND SRCS cryptosoft.c)
      list(APPEND SRCS xform.c)
    endif()
  endif()

  # Software crypto library

  if(CONFIG_CRYPTO_SW_AES)
    list(APPEND SRCS aes.c)
  endif()
  list(APPEND SRCS blake2s.c)
  list(APPEND SRCS blf.c)
  list(APPEND SRCS cast.c)
  list(APPEND SRCS chachapoly.c)
  list(APPEND SRCS ecb_enc.c)
  list(APPEND SRCS ecb3_enc.c)
  list(APPEND SRCS set_key.c)
  list(APPEND SRCS md5.c)
  list(APPEND SRCS poly1305.c)
  list(APPEND SRCS rijndael.c)
  list(APPEND SRCS rmd160.c)
  list(APPEND SRCS sha1.c)
  list(APPEND SRCS sha2.c)
  list(APPEND SRCS gmac.c)
  list(APPEND SRCS cmac.c)
  list(APPEND SRCS hmac.c)
  if(CONFIG_CRYPTO_RANDOM_POOL)
    list(APPEND SRCS idgen.c)
    list(APPEND SRCS curve25519.c)
  endif()
  list(APPEND SRCS key_wrap.c)
  list(APPEND SRCS siphash.c)
  list(APPEND SRCS hmac_buff.c)
  list(APPEND SRCS bn.c)

  # Entropy pool random number generator

  if(CONFIG_CRYPTO_RANDOM_POOL)
    list(APPEND SRCS random_pool.c)
  endif()

  target_sources(crypto PRIVATE ${SRCS})
endif()
