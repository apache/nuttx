# ##############################################################################
# arch/risc-v/src/common/espressif/Wireless.cmake
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

# ##############################################################################
# Include Paths for Wireless
# ##############################################################################

target_include_directories(
  arch
  PRIVATE
    ${ESP_HAL_3RDPARTY_REPO}/components/bt/include/${CHIP_SERIES}/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_coex/include
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_wifi/wifi_apps/roaming_app/include
    ${ESP_HAL_3RDPARTY_REPO}/components/soc/${CHIP_SERIES}/include
    ${ESP_HAL_3RDPARTY_REPO}/nuttx/${CHIP_SERIES}/include
    ${ESP_HAL_3RDPARTY_REPO}/nuttx/include/esp_wifi)

# ##############################################################################
# Link Libraries
# ##############################################################################

nuttx_add_extra_library(
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_phy/lib/${CHIP_SERIES}/libphy.a
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_coex/lib/${CHIP_SERIES}/libcoexist.a
  ${ESP_HAL_3RDPARTY_REPO}/components/esp_wifi/lib/${CHIP_SERIES}/libmesh.a)

if(CONFIG_ESPRESSIF_BLE)
  nuttx_add_extra_library(
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_phy/lib/${CHIP_SERIES}/libbtbb.a)
  if(CONFIG_ARCH_CHIP_ESP32C3)
    nuttx_add_extra_library(
      ${ESP_HAL_3RDPARTY_REPO}/components/bt/controller/lib_esp32c3_family/esp32c3/libbtdm_app.a
    )
  endif()
endif()

if(CONFIG_ESPRESSIF_WIFI)
  nuttx_add_extra_library(
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_wifi/lib/${CHIP_SERIES}/libcore.a
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_wifi/lib/${CHIP_SERIES}/libnet80211.a
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_wifi/lib/${CHIP_SERIES}/libpp.a
    ${ESP_HAL_3RDPARTY_REPO}/components/esp_wifi/lib/${CHIP_SERIES}/libespnow.a)
  if(CONFIG_WPA_WAPI_PSK)
    nuttx_add_extra_library(
      ${ESP_HAL_3RDPARTY_REPO}/components/esp_wifi/lib/${CHIP_SERIES}/libwapi.a)
  endif()

  # ############################################################################
  # ESP-IDF's mbedTLS
  # ############################################################################

  set(MBEDTLS_DIR ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/mbedtls)

  # Include directories for mbedTLS Use BEFORE so port/include and
  # builtin/include come before hal's mbedtls paths; then #include_next
  # "mbedtls/private/*.h" from port bignum.h finds builtin/include.
  target_include_directories(
    arch BEFORE
    PRIVATE ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/port/include
            ${MBEDTLS_DIR}/tf-psa-crypto/drivers/builtin/include
            ${MBEDTLS_DIR}/tf-psa-crypto/drivers/builtin/src
            ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/port/include/aes
            ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/port/psa_driver/include
            ${MBEDTLS_DIR}/include
            ${MBEDTLS_DIR}/library
            ${ESP_HAL_3RDPARTY_REPO}/nuttx/include/mbedtls)

  # Define Espressif's configs for mbedTLS
  target_compile_definitions(arch
                             PUBLIC MBEDTLS_CONFIG_FILE="mbedtls/esp_config.h")
  target_compile_definitions(
    arch PRIVATE TF_PSA_CRYPTO_USER_CONFIG_FILE=\"mbedtls/esp_config.h\")

  # Ensure PSA crypto initialization is included in the build
  target_link_options(arch PRIVATE -u mbedtls_psa_crypto_init_include_impl)

  # mbedTLS sources: exact match to Wireless.mk (lines 59–103). All from
  # tf-psa-crypto/drivers/builtin/src (VPATH in mk).
  set(MBEDTLS_BUILTIN_DIR ${MBEDTLS_DIR}/tf-psa-crypto/drivers/builtin/src)
  set(MBEDTLS_SRCS
      ${MBEDTLS_BUILTIN_DIR}/aes.c
      ${MBEDTLS_BUILTIN_DIR}/aria.c
      ${MBEDTLS_BUILTIN_DIR}/bignum_core.c
      ${MBEDTLS_BUILTIN_DIR}/bignum.c
      ${MBEDTLS_BUILTIN_DIR}/ccm.c
      ${MBEDTLS_BUILTIN_DIR}/cipher_wrap.c
      ${MBEDTLS_BUILTIN_DIR}/cipher.c
      ${MBEDTLS_BUILTIN_DIR}/cmac.c
      ${MBEDTLS_BUILTIN_DIR}/constant_time.c
      ${MBEDTLS_BUILTIN_DIR}/ctr_drbg.c
      ${MBEDTLS_BUILTIN_DIR}/ecp_curves.c
      ${MBEDTLS_BUILTIN_DIR}/ecp.c
      ${MBEDTLS_BUILTIN_DIR}/entropy.c
      ${MBEDTLS_BUILTIN_DIR}/gcm.c
      ${MBEDTLS_BUILTIN_DIR}/md.c
      ${MBEDTLS_BUILTIN_DIR}/pkcs5.c
      ${MBEDTLS_BUILTIN_DIR}/platform_util.c
      ${MBEDTLS_BUILTIN_DIR}/platform.c
      ${MBEDTLS_BUILTIN_DIR}/sha1.c
      ${MBEDTLS_BUILTIN_DIR}/sha3.c
      ${MBEDTLS_BUILTIN_DIR}/sha256.c
      ${MBEDTLS_BUILTIN_DIR}/sha512.c
      ${MBEDTLS_BUILTIN_DIR}/pk.c
      ${MBEDTLS_BUILTIN_DIR}/pk_wrap.c
      ${MBEDTLS_BUILTIN_DIR}/pkparse.c
      ${MBEDTLS_BUILTIN_DIR}/ecdsa.c
      ${MBEDTLS_BUILTIN_DIR}/asn1parse.c
      ${MBEDTLS_BUILTIN_DIR}/asn1write.c
      ${MBEDTLS_BUILTIN_DIR}/rsa.c
      ${MBEDTLS_BUILTIN_DIR}/md5.c
      ${MBEDTLS_BUILTIN_DIR}/oid.c
      ${MBEDTLS_BUILTIN_DIR}/pem.c
      ${MBEDTLS_BUILTIN_DIR}/hmac_drbg.c
      ${MBEDTLS_BUILTIN_DIR}/rsa_alt_helpers.c
      ${MBEDTLS_BUILTIN_DIR}/ecdh.c
      ${MBEDTLS_BUILTIN_DIR}/pk_ecc.c
      ${MBEDTLS_BUILTIN_DIR}/pk_rsa.c
      # Required by pem.c (esp_mbedtls_base64_decode); in same builtin tree, not
      # in Wireless.mk mbedtls list
      ${MBEDTLS_BUILTIN_DIR}/base64.c
      ${MBEDTLS_BUILTIN_DIR}/psa_util.c
      ${MBEDTLS_BUILTIN_DIR}/psa_crypto_ffdh.c
      ${MBEDTLS_BUILTIN_DIR}/psa_crypto_ecp.c
      ${MBEDTLS_BUILTIN_DIR}/psa_crypto_rsa.c
      ${MBEDTLS_BUILTIN_DIR}/psa_crypto_cipher.c
      ${MBEDTLS_BUILTIN_DIR}/psa_crypto_mac.c
      ${MBEDTLS_BUILTIN_DIR}/psa_crypto_hash.c)

  # TF-PSA crypto core (Wireless.mk lines 105–114)
  set(TF_PSA_CORE_SRCS
      ${MBEDTLS_DIR}/tf-psa-crypto/core/psa_crypto_client.c
      ${MBEDTLS_DIR}/tf-psa-crypto/core/psa_crypto_driver_wrappers_no_static.c
      ${MBEDTLS_DIR}/tf-psa-crypto/core/psa_crypto_slot_management.c
      ${MBEDTLS_DIR}/tf-psa-crypto/core/psa_crypto_storage.c
      ${MBEDTLS_DIR}/tf-psa-crypto/core/psa_crypto.c
      ${MBEDTLS_DIR}/tf-psa-crypto/core/psa_its_file.c
      ${MBEDTLS_DIR}/tf-psa-crypto/core/tf_psa_crypto_config.c
      ${MBEDTLS_DIR}/tf-psa-crypto/core/tf_psa_crypto_version.c)

  # mbedTLS port (Wireless.mk lines 116–127)
  set(MBEDTLS_PORT_SRCS
      ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/port/esp_psa_crypto_init.c
      ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/port/esp_hardware.c
      ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/port/esp_mem.c
      ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/port/esp_timing.c
      ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/port/psa_driver/esp_mac/psa_crypto_driver_esp_hmac_opaque.c
      ${ESP_HAL_3RDPARTY_REPO}/components/mbedtls/port/psa_driver/esp_md/psa_crypto_driver_esp_md5.c
  )

  target_sources(arch PRIVATE ${MBEDTLS_SRCS} ${TF_PSA_CORE_SRCS}
                              ${MBEDTLS_PORT_SRCS})

  # ############################################################################
  # WPA Supplicant
  # ############################################################################

  set(WPA_SUPPLICANT_DIR ${ESP_HAL_3RDPARTY_REPO}/components/wpa_supplicant)

  target_compile_definitions(
    arch
    PRIVATE __ets__
            CONFIG_CRYPTO_MBEDTLS
            CONFIG_ECC
            CONFIG_IEEE80211W
            CONFIG_WPA3_SAE
            EAP_PEER_METHOD
            ESP_PLATFORM=1
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

  if(CONFIG_ESPRESSIF_WIFI_ENABLE_WPA3_OWE_STA)
    target_compile_definitions(arch PRIVATE CONFIG_OWE_STA)
  endif()

  if(CONFIG_ESPRESSIF_WIFI_GCMP_SUPPORT)
    target_compile_definitions(arch PRIVATE CONFIG_GCMP)
  endif()

  if(CONFIG_ESPRESSIF_WIFI_GMAC_SUPPORT)
    target_compile_definitions(arch PRIVATE CONFIG_GMAC)
  endif()

  target_include_directories(
    arch
    PRIVATE ${WPA_SUPPLICANT_DIR}/include ${WPA_SUPPLICANT_DIR}/src
            ${WPA_SUPPLICANT_DIR}/src/ap ${WPA_SUPPLICANT_DIR}/src/common
            ${WPA_SUPPLICANT_DIR}/src/utils)

  # WPA Supplicant AP sources
  set(WPA_AP_SRCS
      ${WPA_SUPPLICANT_DIR}/src/ap/ap_config.c
      ${WPA_SUPPLICANT_DIR}/src/ap/ieee802_11.c
      ${WPA_SUPPLICANT_DIR}/src/ap/comeback_token.c
      ${WPA_SUPPLICANT_DIR}/src/ap/pmksa_cache_auth.c
      ${WPA_SUPPLICANT_DIR}/src/ap/sta_info.c
      ${WPA_SUPPLICANT_DIR}/src/ap/wpa_auth_ie.c
      ${WPA_SUPPLICANT_DIR}/src/ap/wpa_auth.c)

  # WPA Supplicant common sources
  set(WPA_COMMON_SRCS
      ${WPA_SUPPLICANT_DIR}/src/common/dragonfly.c
      ${WPA_SUPPLICANT_DIR}/src/common/sae.c
      ${WPA_SUPPLICANT_DIR}/src/common/wpa_common.c
      ${WPA_SUPPLICANT_DIR}/src/common/bss.c
      ${WPA_SUPPLICANT_DIR}/src/common/scan.c
      ${WPA_SUPPLICANT_DIR}/src/common/ieee802_11_common.c)

  if(CONFIG_ESPRESSIF_WIFI_ENABLE_SAE_PK)
    list(APPEND WPA_COMMON_SRCS ${WPA_SUPPLICANT_DIR}/src/common/sae_pk.c)
  endif()

  # WPA Supplicant crypto sources (wpa_supplicant/src/crypto; aes-siv.c is in
  # esp_supplicant)
  set(WPA_CRYPTO_SRCS
      ${WPA_SUPPLICANT_DIR}/src/crypto/aes-ccm.c
      ${WPA_SUPPLICANT_DIR}/src/crypto/aes-gcm.c
      ${WPA_SUPPLICANT_DIR}/src/crypto/aes-unwrap.c
      ${WPA_SUPPLICANT_DIR}/src/crypto/aes-wrap.c
      ${WPA_SUPPLICANT_DIR}/src/crypto/ccmp.c
      ${WPA_SUPPLICANT_DIR}/src/crypto/crypto_ops.c
      ${WPA_SUPPLICANT_DIR}/src/crypto/des-internal.c
      ${WPA_SUPPLICANT_DIR}/src/crypto/dh_groups.c
      ${WPA_SUPPLICANT_DIR}/src/crypto/rc4.c
      ${WPA_SUPPLICANT_DIR}/src/crypto/sha1-prf.c
      ${WPA_SUPPLICANT_DIR}/src/crypto/sha256-kdf.c
      ${WPA_SUPPLICANT_DIR}/src/crypto/sha256-prf.c)

  # WPA Supplicant EAP peer sources
  set(WPA_EAP_SRCS
      ${WPA_SUPPLICANT_DIR}/src/eap_peer/chap.c
      ${WPA_SUPPLICANT_DIR}/src/eap_peer/eap_common.c
      ${WPA_SUPPLICANT_DIR}/src/eap_peer/eap_mschapv2.c
      ${WPA_SUPPLICANT_DIR}/src/eap_peer/eap_peap_common.c
      ${WPA_SUPPLICANT_DIR}/src/eap_peer/eap_peap.c
      ${WPA_SUPPLICANT_DIR}/src/eap_peer/eap_tls_common.c
      ${WPA_SUPPLICANT_DIR}/src/eap_peer/eap_tls.c
      ${WPA_SUPPLICANT_DIR}/src/eap_peer/eap_ttls.c
      ${WPA_SUPPLICANT_DIR}/src/eap_peer/eap.c
      ${WPA_SUPPLICANT_DIR}/src/eap_peer/mschapv2.c)

  # WPA Supplicant RSN sources
  set(WPA_RSN_SRCS
      ${WPA_SUPPLICANT_DIR}/src/rsn_supp/pmksa_cache.c
      ${WPA_SUPPLICANT_DIR}/src/rsn_supp/wpa_ie.c
      ${WPA_SUPPLICANT_DIR}/src/rsn_supp/wpa.c)

  # WPA Supplicant utils sources
  target_include_directories(arch PRIVATE ${WPA_SUPPLICANT_DIR}/src/utils)

  set(WPA_UTILS_SRCS
      ${WPA_SUPPLICANT_DIR}/src/utils/base64.c
      ${WPA_SUPPLICANT_DIR}/src/utils/bitfield.c
      ${WPA_SUPPLICANT_DIR}/src/utils/common.c
      ${WPA_SUPPLICANT_DIR}/src/utils/ext_password.c
      ${WPA_SUPPLICANT_DIR}/src/utils/json.c
      ${WPA_SUPPLICANT_DIR}/src/utils/uuid.c
      ${WPA_SUPPLICANT_DIR}/src/utils/wpa_debug.c
      ${WPA_SUPPLICANT_DIR}/src/utils/wpabuf.c)

  # WPA Supplicant port sources
  target_include_directories(arch PRIVATE ${WPA_SUPPLICANT_DIR}/port/include)

  set(WPA_PORT_SRCS ${WPA_SUPPLICANT_DIR}/port/eloop.c
                    ${WPA_SUPPLICANT_DIR}/port/os_xtensa.c)

  # ESP Supplicant sources
  target_include_directories(
    arch
    PRIVATE ${WPA_SUPPLICANT_DIR}/esp_supplicant/include
            ${WPA_SUPPLICANT_DIR}/esp_supplicant/src
            ${WPA_SUPPLICANT_DIR}/src/crypto)

  set(ESP_SUPPLICANT_SRCS
      ${WPA_SUPPLICANT_DIR}/esp_supplicant/src/esp_common.c
      ${WPA_SUPPLICANT_DIR}/esp_supplicant/src/esp_hostap.c
      ${WPA_SUPPLICANT_DIR}/esp_supplicant/src/esp_wpa_main.c
      ${WPA_SUPPLICANT_DIR}/esp_supplicant/src/esp_wpa3.c
      ${WPA_SUPPLICANT_DIR}/esp_supplicant/src/esp_wpas_glue.c
      ${WPA_SUPPLICANT_DIR}/esp_supplicant/src/esp_owe.c
      ${WPA_SUPPLICANT_DIR}/esp_supplicant/src/esp_scan.c
      ${WPA_SUPPLICANT_DIR}/esp_supplicant/src/esp_wps.c
      ${WPA_SUPPLICANT_DIR}/esp_supplicant/src/crypto/crypto_mbedtls-bignum.c
      ${WPA_SUPPLICANT_DIR}/esp_supplicant/src/crypto/crypto_mbedtls-ec.c
      ${WPA_SUPPLICANT_DIR}/esp_supplicant/src/crypto/crypto_mbedtls-rsa.c
      ${WPA_SUPPLICANT_DIR}/esp_supplicant/src/crypto/crypto_mbedtls.c
      ${WPA_SUPPLICANT_DIR}/esp_supplicant/src/crypto/tls_mbedtls.c
      ${WPA_SUPPLICANT_DIR}/src/crypto/aes-siv.c)

  # ESP WiFi sources
  set(ESP_WIFI_SRCS
      ${ESP_HAL_3RDPARTY_REPO}/components/esp_wifi/src/wifi_init.c
      ${ESP_HAL_3RDPARTY_REPO}/components/esp_wifi/src/lib_printf.c
      ${ESP_HAL_3RDPARTY_REPO}/components/esp_wifi/regulatory/esp_wifi_regulatory.c
  )

  # Add all WPA supplicant sources
  target_sources(
    arch
    PRIVATE ${WPA_AP_SRCS}
            ${WPA_COMMON_SRCS}
            ${WPA_CRYPTO_SRCS}
            ${WPA_EAP_SRCS}
            ${WPA_RSN_SRCS}
            ${WPA_UTILS_SRCS}
            ${WPA_PORT_SRCS}
            ${ESP_SUPPLICANT_SRCS}
            ${ESP_WIFI_SRCS})

endif() # CONFIG_ESPRESSIF_WIFI
