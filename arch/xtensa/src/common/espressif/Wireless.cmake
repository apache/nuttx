# ##############################################################################
# arch/xtensa/src/common/espressif/Wireless.cmake
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

# Wi-Fi and Bluetooth, as Wireless.mk (and the chips' Make.defs for the
# Bluetooth controller libraries)

# arch/xtensa/src/common/espressif/Wireless.mk SPDX-License-Identifier:
# Apache-2.0 Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with this
# work for additional information regarding copyright ownership.  The ASF
# licenses this file to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# http://www.apache.org/licenses/LICENSE-2.0 Unless required by applicable law
# or agreed to in writing, software distributed under the License is distributed
# on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
# express or implied.  See the License for the specific language governing
# permissions and limitations under the License.
if(CONFIG_ARCH_CHIP_ESP32)
  target_include_directories(
    arch
    PRIVATE
      ${ESP_HAL_3RDPARTY_REPO}/components/bt/include/${CHIP_SERIES}/include)
endif()
if(CONFIG_ARCH_CHIP_ESP32S3)
  target_include_directories(
    arch PRIVATE ${ESP_HAL_3RDPARTY_REPO}/components/bt/include/esp32c3/include)
endif()
target_include_directories(
  arch
  PRIVATE
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_coex/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_wifi/wifi_apps/roaming_app/include
    ${ESP_HAL_3RDPARTY_REPO}/components/soc/${CHIP_SERIES}/include
    ${ESP_HAL_3RDPARTY_REPO}/nuttx/${CHIP_SERIES}/include
    ${ESP_HAL_3RDPARTY_REPO}/nuttx/include/esp_wifi)
# Prebuilt libraries

set(ESP_WIFI_LIB_DIR
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_wifi/lib/${CHIP_SERIES})
set(ESP_PHY_LIB_DIR
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_phy/lib/${CHIP_SERIES})
set(ESP_COEX_LIB_DIR
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_coex/lib/${CHIP_SERIES})

if(CONFIG_ARCH_CHIP_ESP32)
  nuttx_add_extra_library(${ESP_PHY_LIB_DIR}/librtc.a)
endif()

if(NOT CONFIG_ARCH_CHIP_ESP32S2)
  nuttx_add_extra_library(${ESP_COEX_LIB_DIR}/libcoexist.a)
endif()

nuttx_add_extra_library(${ESP_PHY_LIB_DIR}/libphy.a)

if(CONFIG_ESPRESSIF_WIFI)
  nuttx_add_extra_library(
    ${ESP_WIFI_LIB_DIR}/libcore.a ${ESP_WIFI_LIB_DIR}/libnet80211.a
    ${ESP_WIFI_LIB_DIR}/libpp.a ${ESP_WIFI_LIB_DIR}/libespnow.a)
endif()

if(CONFIG_WPA_WAPI_PSK)
  nuttx_add_extra_library(${ESP_WIFI_LIB_DIR}/libwapi.a)
endif()

if(CONFIG_ESPRESSIF_BLE)
  if(CONFIG_ARCH_CHIP_ESP32)
    nuttx_add_extra_library(
      ${ESP_HAL_3RDPARTY_REPO}/components/bt/controller/lib_esp32/${CHIP_SERIES}/libbtdm_app.a
    )
  elseif(CONFIG_ARCH_CHIP_ESP32S3)
    nuttx_add_extra_library(
      ${ESP_PHY_LIB_DIR}/libbtbb.a
      ${ESP_HAL_3RDPARTY_REPO}/components/bt/controller/lib_esp32c3_family/${CHIP_SERIES}/libbtdm_app.a
    )
  endif()
endif()
target_include_directories(
  arch
  PRIVATE
    ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/include
    ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/src
    ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/esp_supplicant/include)
list(APPEND SRCS ${ESP_HAL_3RDPARTY_REPO}/components/esp_wifi/src/wifi_init.c)
if(CONFIG_ESPRESSIF_WIFI)
  target_include_directories(
    arch
    PRIVATE
      ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls/library
      ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/port/include
      ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/port/include/aes
      ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/port/psa_driver/include
      ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls/tf-psa-crypto/drivers/builtin/include
      ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls/tf-psa-crypto/drivers/builtin/src
      ${ESP_HAL_3RDPARTY_REPO}/nuttx/include/mbedtls)
  target_compile_definitions(
    arch PRIVATE "MBEDTLS_CONFIG_FILE=<mbedtls/esp_config.h>")
  list(
    APPEND
    SRCS
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls/tf-psa-crypto/drivers/builtin/src/aes.c
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls/tf-psa-crypto/drivers/builtin/src/aria.c
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls/tf-psa-crypto/drivers/builtin/src/bignum_core.c
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls/tf-psa-crypto/drivers/builtin/src/bignum.c
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls/tf-psa-crypto/drivers/builtin/src/ccm.c
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls/tf-psa-crypto/drivers/builtin/src/cipher_wrap.c
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls/tf-psa-crypto/drivers/builtin/src/cipher.c
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls/tf-psa-crypto/drivers/builtin/src/cmac.c
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls/tf-psa-crypto/drivers/builtin/src/constant_time.c
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls/tf-psa-crypto/drivers/builtin/src/ctr_drbg.c
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls/tf-psa-crypto/drivers/builtin/src/ecp_curves.c
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls/tf-psa-crypto/drivers/builtin/src/ecp.c
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls/tf-psa-crypto/drivers/builtin/src/entropy.c
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls/tf-psa-crypto/drivers/builtin/src/gcm.c
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls/tf-psa-crypto/drivers/builtin/src/md.c
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls/tf-psa-crypto/drivers/builtin/src/pkcs5.c
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls/tf-psa-crypto/drivers/builtin/src/platform_util.c
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls/tf-psa-crypto/drivers/builtin/src/platform.c
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls/tf-psa-crypto/drivers/builtin/src/sha1.c
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls/tf-psa-crypto/drivers/builtin/src/sha3.c
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls/tf-psa-crypto/drivers/builtin/src/sha256.c
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls/tf-psa-crypto/drivers/builtin/src/sha512.c
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls/tf-psa-crypto/drivers/builtin/src/pk.c
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls/tf-psa-crypto/drivers/builtin/src/pk_wrap.c
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls/tf-psa-crypto/drivers/builtin/src/pkparse.c
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls/tf-psa-crypto/drivers/builtin/src/ecdsa.c
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls/tf-psa-crypto/drivers/builtin/src/asn1parse.c
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls/tf-psa-crypto/drivers/builtin/src/asn1write.c
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls/tf-psa-crypto/drivers/builtin/src/rsa.c
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls/tf-psa-crypto/drivers/builtin/src/md5.c
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls/tf-psa-crypto/drivers/builtin/src/oid.c
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls/tf-psa-crypto/drivers/builtin/src/pem.c
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls/tf-psa-crypto/drivers/builtin/src/hmac_drbg.c
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls/tf-psa-crypto/drivers/builtin/src/rsa_alt_helpers.c
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls/tf-psa-crypto/drivers/builtin/src/ecdh.c
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls/tf-psa-crypto/drivers/builtin/src/pk_ecc.c
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls/tf-psa-crypto/drivers/builtin/src/pk_rsa.c
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls/tf-psa-crypto/drivers/builtin/src/psa_util.c
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls/tf-psa-crypto/drivers/builtin/src/psa_crypto_ffdh.c
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls/tf-psa-crypto/drivers/builtin/src/psa_crypto_ecp.c
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls/tf-psa-crypto/drivers/builtin/src/psa_crypto_rsa.c
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls/tf-psa-crypto/drivers/builtin/src/psa_crypto_cipher.c
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls/tf-psa-crypto/drivers/builtin/src/psa_crypto_mac.c
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls/tf-psa-crypto/drivers/builtin/src/psa_crypto_hash.c
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls/tf-psa-crypto/core/psa_crypto_client.c
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls/tf-psa-crypto/core/psa_crypto_driver_wrappers_no_static.c
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls/tf-psa-crypto/core/psa_crypto_slot_management.c
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls/tf-psa-crypto/core/psa_crypto_storage.c
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls/tf-psa-crypto/core/psa_crypto.c
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls/tf-psa-crypto/core/psa_its_file.c
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls/tf-psa-crypto/core/tf_psa_crypto_config.c
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls/tf-psa-crypto/core/tf_psa_crypto_version.c
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/port/esp_psa_crypto_init.c
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/port/esp_hardware.c
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/port/esp_mem.c
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/port/esp_timing.c)
  # Ensure PSA crypto initialization is included in the build
  target_link_options(nuttx PRIVATE
                      "SHELL:-u mbedtls_psa_crypto_init_include_impl")
  if(NOT CONFIG_ARCH_CHIP_ESP32)
    list(
      APPEND
      SRCS
      ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/port/psa_driver/esp_mac/psa_crypto_driver_esp_hmac_opaque.c
    )
  endif()
  list(
    APPEND
    SRCS
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/port/psa_driver/esp_md/psa_crypto_driver_esp_md5.c
  )
  target_compile_definitions(
    arch
    PRIVATE __ets__
            CONFIG_CRYPTO_MBEDTLS
            CONFIG_ECC
            CONFIG_IEEE80211W
            CONFIG_WPA3_SAE
            EAP_PEER_METHOD
            ESP_PLATFORM=1
            "TF_PSA_CRYPTO_USER_CONFIG_FILE=\"mbedtls/esp_config.h\""
            ESP_SUPPLICANT
            ESPRESSIF_USE
            IEEE8021X_EAPOL
            USE_WPA2_TASK
            CONFIG_SHA256
            USE_WPS_TASK)
  if(CONFIG_ESPRESSIF_WIFI_SOFTAP_SAE_SUPPORT)
    target_compile_definitions(arch PRIVATE CONFIG_SAE)
  endif()
  if(CONFIG_ESPRESSIF_WIFI_ENABLE_SAE_PK)
    target_compile_definitions(arch PRIVATE CONFIG_SAE_PK)
  endif()
  if(CONFIG_ESPRESSIF_WIFI_ENABLE_SAE_H2E)
    target_compile_definitions(arch PRIVATE CONFIG_SAE_H2E)
  endif()
  if(CONFIG_ESP_WIFI_ENABLE_WPA3_OWE_STA)
    target_compile_definitions(arch PRIVATE CONFIG_OWE_STA)
  endif()
  if(CONFIG_ESPRESSIF_WIFI_GCMP_SUPPORT)
    target_compile_definitions(arch PRIVATE CONFIG_GCMP)
  endif()
  if(CONFIG_ESPRESSIF_WIFI_GMAC_SUPPORT)
    target_compile_definitions(arch PRIVATE CONFIG_GMAC)
  endif()
  target_include_directories(
    arch PRIVATE ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/src/ap)
  list(
    APPEND
    SRCS
    ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/src/ap/ap_config.c
    ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/src/ap/ieee802_11.c
    ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/src/ap/comeback_token.c
    ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/src/ap/pmksa_cache_auth.c
    ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/src/ap/sta_info.c
    ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/src/ap/wpa_auth_ie.c
    ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/src/ap/wpa_auth.c)
  target_include_directories(
    arch PRIVATE ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/src/common)
  list(
    APPEND
    SRCS
    ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/src/common/dragonfly.c
    ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/src/common/sae.c
    ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/src/common/wpa_common.c)
  if(CONFIG_ESPRESSIF_WIFI_ENABLE_SAE_PK)
    list(APPEND SRCS
         ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/src/common/sae_pk.c)
  endif()
  list(
    APPEND
    SRCS
    ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/src/common/bss.c
    ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/src/common/scan.c
    ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/src/common/ieee802_11_common.c
    ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/src/crypto/aes-ccm.c
    ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/src/crypto/aes-gcm.c
    ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/src/crypto/aes-unwrap.c
    ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/src/crypto/aes-wrap.c
    ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/src/crypto/ccmp.c
    ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/src/crypto/crypto_ops.c
    ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/src/crypto/des-internal.c
    ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/src/crypto/dh_groups.c
    ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/src/crypto/rc4.c
    ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/src/crypto/sha1-prf.c
    ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/src/crypto/sha256-kdf.c
    ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/src/crypto/sha256-prf.c
    ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/src/eap_peer/chap.c
    ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/src/eap_peer/eap_common.c
    ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/src/eap_peer/eap_mschapv2.c
    ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/src/eap_peer/eap_peap_common.c
    ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/src/eap_peer/eap_peap.c
    ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/src/eap_peer/eap_tls_common.c
    ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/src/eap_peer/eap_tls.c
    ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/src/eap_peer/eap_ttls.c
    ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/src/eap_peer/eap.c
    ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/src/eap_peer/mschapv2.c
    ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/src/rsn_supp/pmksa_cache.c
    ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/src/rsn_supp/wpa_ie.c
    ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/src/rsn_supp/wpa.c)
  target_include_directories(
    arch PRIVATE ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/src/utils)
  list(
    APPEND
    SRCS
    ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls/tf-psa-crypto/drivers/builtin/src/base64.c
    ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/src/utils/bitfield.c
    ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/src/utils/common.c
    ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/src/utils/ext_password.c
    ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/src/utils/json.c
    ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/src/utils/uuid.c
    ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/src/utils/wpa_debug.c
    ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/src/utils/wpabuf.c)
  target_include_directories(
    arch
    PRIVATE ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/port/include)
  list(APPEND SRCS
       ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/port/eloop.c
       ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/port/os_xtensa.c)
  target_include_directories(
    arch
    PRIVATE
      ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/esp_supplicant/src)
  list(
    APPEND
    SRCS
    ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/esp_supplicant/src/esp_common.c
    ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/esp_supplicant/src/esp_hostap.c
    ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/esp_supplicant/src/esp_wpa_main.c
    ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/esp_supplicant/src/esp_wpa3.c
    ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/esp_supplicant/src/esp_wpas_glue.c
    ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/esp_supplicant/src/esp_owe.c
    ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/esp_supplicant/src/esp_scan.c
    ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/esp_supplicant/src/esp_wps.c
  )
  target_include_directories(
    arch PRIVATE ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/src/crypto)
  list(
    APPEND
    SRCS
    ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/esp_supplicant/src/crypto/crypto_mbedtls-bignum.c
    ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/esp_supplicant/src/crypto/crypto_mbedtls-ec.c
    ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/esp_supplicant/src/crypto/crypto_mbedtls-rsa.c
    ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/esp_supplicant/src/crypto/crypto_mbedtls.c
    ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/esp_supplicant/src/crypto/tls_mbedtls.c
    ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant/src/crypto/aes-siv.c
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_wifi/src/lib_printf.c
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_wifi/regulatory/esp_wifi_regulatory.c
  )
  if(CONFIG_ESPRESSIF_WIFI_BT_COEXIST)
    list(APPEND SRCS
         ${ESP_HAL_3RDPARTY_REPO}/components/esp_coex/src/lib_printf.c)
  endif()
endif()
# Linker scripts
if(CONFIG_ARCH_CHIP_ESP32S3)
  set_property(
    GLOBAL APPEND
    PROPERTY
      LD_SCRIPT
      ${ESP_HAL_3RDPARTY_REPO}/components/esp_rom/${CHIP_SERIES}/ld/${CHIP_SERIES}.rom.bt_funcs.ld
  )
endif()
