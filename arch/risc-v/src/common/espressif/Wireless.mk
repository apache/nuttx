############################################################################
# arch/risc-v/src/common/espressif/Wireless.mk
#
# SPDX-License-Identifier: Apache-2.0
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.  The
# ASF licenses this file to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the
# License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations
# under the License.
#
############################################################################

INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)bt$(DELIM)include$(DELIM)$(CHIP_SERIES)$(DELIM)include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_coex$(DELIM)include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)soc$(DELIM)$(CHIP_SERIES)$(DELIM)include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)nuttx$(DELIM)$(CHIP_SERIES)$(DELIM)include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)nuttx$(DELIM)include$(DELIM)esp_wifi

EXTRA_LIBPATHS += -L $(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)bt$(DELIM)controller$(DELIM)lib_esp32c3_family$(DELIM)$(CHIP_SERIES)
EXTRA_LIBPATHS += -L $(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_coex$(DELIM)lib$(DELIM)$(CHIP_SERIES)
EXTRA_LIBPATHS += -L $(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_phy$(DELIM)lib$(DELIM)$(CHIP_SERIES)
EXTRA_LIBPATHS += -L $(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_wifi$(DELIM)lib$(DELIM)$(CHIP_SERIES)

EXTRA_LIBS += -lphy -lcoexist -lmesh

ifeq ($(CONFIG_ESPRESSIF_WIFI),y)

ifeq ($(CONFIG_WPA_WAPI_PSK),y)
EXTRA_LIBS += -lwapi
endif

## ESP-IDF's mbedTLS

VPATH += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)mbedtls$(DELIM)mbedtls$(DELIM)library

INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)mbedtls$(DELIM)mbedtls$(DELIM)include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)mbedtls$(DELIM)mbedtls$(DELIM)library
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)mbedtls$(DELIM)port$(DELIM)include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)nuttx$(DELIM)include$(DELIM)mbedtls

### Define Espressif's configs for mbedTLS

CFLAGS += $(DEFINE_PREFIX)MBEDTLS_CONFIG_FILE="<mbedtls/esp_config.h>"

CHIP_CSRCS += aes.c
CHIP_CSRCS += aria.c
CHIP_CSRCS += bignum_core.c
CHIP_CSRCS += bignum.c
CHIP_CSRCS += ccm.c
CHIP_CSRCS += cipher_wrap.c
CHIP_CSRCS += cipher.c
CHIP_CSRCS += cmac.c
CHIP_CSRCS += constant_time.c
CHIP_CSRCS += ctr_drbg.c
CHIP_CSRCS += ecp_curves.c
CHIP_CSRCS += ecp.c
CHIP_CSRCS += entropy.c
CHIP_CSRCS += gcm.c
CHIP_CSRCS += md.c
CHIP_CSRCS += pkcs5.c
CHIP_CSRCS += platform_util.c
CHIP_CSRCS += platform.c
CHIP_CSRCS += sha1.c
CHIP_CSRCS += sha3.c
CHIP_CSRCS += sha256.c
CHIP_CSRCS += sha512.c
CHIP_CSRCS += pk.c
CHIP_CSRCS += pk_wrap.c
CHIP_CSRCS += pkparse.c
CHIP_CSRCS += ecdsa.c
CHIP_CSRCS += asn1parse.c
CHIP_CSRCS += asn1write.c
CHIP_CSRCS += rsa.c
CHIP_CSRCS += md5.c
CHIP_CSRCS += oid.c
CHIP_CSRCS += pem.c
CHIP_CSRCS += hmac_drbg.c
CHIP_CSRCS += rsa_alt_helpers.c
CHIP_CSRCS += ecdh.c
CHIP_CSRCS += pk_ecc.c

VPATH += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)mbedtls$(DELIM)port

CHIP_CSRCS += esp_hardware.c
CHIP_CSRCS += esp_mem.c
CHIP_CSRCS += esp_timing.c

VPATH += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)mbedtls$(DELIM)port$(DELIM)md

CHIP_CSRCS += esp_md.c

## WPA Supplicant

WIFI_WPA_SUPPLICANT = chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)wpa_supplicant

CFLAGS += $(DEFINE_PREFIX)__ets__
CFLAGS += $(DEFINE_PREFIX)CONFIG_CRYPTO_MBEDTLS
CFLAGS += $(DEFINE_PREFIX)CONFIG_ECC
CFLAGS += $(DEFINE_PREFIX)CONFIG_IEEE80211W
CFLAGS += $(DEFINE_PREFIX)CONFIG_WPA3_SAE
CFLAGS += $(DEFINE_PREFIX)EAP_PEER_METHOD
CFLAGS += $(DEFINE_PREFIX)ESP_PLATFORM=1
CFLAGS += $(DEFINE_PREFIX)ESP_SUPPLICANT
CFLAGS += $(DEFINE_PREFIX)ESPRESSIF_USE
CFLAGS += $(DEFINE_PREFIX)IEEE8021X_EAPOL
CFLAGS += $(DEFINE_PREFIX)USE_WPA2_TASK
CFLAGS += $(DEFINE_PREFIX)CONFIG_SHA256
CFLAGS += $(DEFINE_PREFIX)CONFIG_SAE
CFLAGS += $(DEFINE_PREFIX)USE_WPS_TASK

ifeq ($(CONFIG_ESPRESSIF_WIFI_ENABLE_SAE_PK),y)
CFLAGS += $(DEFINE_PREFIX)CONFIG_SAE_PK
endif

ifeq ($(CONFIG_ESPRESSIF_WIFI_ENABLE_WPA3_OWE_STA),y)
CFLAGS += $(DEFINE_PREFIX)CONFIG_OWE_STA
endif

ifeq ($(CONFIG_ESPRESSIF_WIFI_GCMP_SUPPORT),y)
CFLAGS += $(DEFINE_PREFIX)CONFIG_GCMP
endif

ifeq ($(CONFIG_ESPRESSIF_WIFI_GMAC_SUPPORT),y)
CFLAGS += $(DEFINE_PREFIX)CONFIG_GMAC
endif

INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)$(WIFI_WPA_SUPPLICANT)$(DELIM)include
INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)$(WIFI_WPA_SUPPLICANT)$(DELIM)src

VPATH += $(WIFI_WPA_SUPPLICANT)$(DELIM)src$(DELIM)ap

INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)$(WIFI_WPA_SUPPLICANT)$(DELIM)src$(DELIM)ap

CHIP_CSRCS += ap_config.c
CHIP_CSRCS += ieee802_11.c
CHIP_CSRCS += comeback_token.c
CHIP_CSRCS += pmksa_cache_auth.c
CHIP_CSRCS += sta_info.c
CHIP_CSRCS += wpa_auth_ie.c
CHIP_CSRCS += wpa_auth.c

VPATH += $(WIFI_WPA_SUPPLICANT)$(DELIM)src$(DELIM)common

INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)$(WIFI_WPA_SUPPLICANT)$(DELIM)src$(DELIM)common

CHIP_CSRCS += dragonfly.c
CHIP_CSRCS += sae.c
CHIP_CSRCS += wpa_common.c
CHIP_CSRCS += sae_pk.c
CHIP_CSRCS += bss.c
CHIP_CSRCS += scan.c
CHIP_CSRCS += ieee802_11_common.c

VPATH += $(WIFI_WPA_SUPPLICANT)$(DELIM)src$(DELIM)crypto

CHIP_CSRCS += aes-ccm.c
CHIP_CSRCS += aes-gcm.c
CHIP_CSRCS += aes-omac1.c
CHIP_CSRCS += aes-unwrap.c
CHIP_CSRCS += aes-wrap.c
CHIP_CSRCS += ccmp.c
CHIP_CSRCS += crypto_ops.c
CHIP_CSRCS += des-internal.c
CHIP_CSRCS += dh_groups.c
CHIP_CSRCS += rc4.c
CHIP_CSRCS += sha1-prf.c
CHIP_CSRCS += sha256-kdf.c
CHIP_CSRCS += sha256-prf.c

VPATH += $(WIFI_WPA_SUPPLICANT)$(DELIM)src$(DELIM)eap_peer

CHIP_CSRCS += chap.c
CHIP_CSRCS += eap_common.c
CHIP_CSRCS += eap_mschapv2.c
CHIP_CSRCS += eap_peap_common.c
CHIP_CSRCS += eap_peap.c
CHIP_CSRCS += eap_tls_common.c
CHIP_CSRCS += eap_tls.c
CHIP_CSRCS += eap_ttls.c
CHIP_CSRCS += eap.c
CHIP_CSRCS += mschapv2.c

VPATH += $(WIFI_WPA_SUPPLICANT)$(DELIM)src$(DELIM)rsn_supp

CHIP_CSRCS += pmksa_cache.c
CHIP_CSRCS += wpa_ie.c
CHIP_CSRCS += wpa.c

VPATH += $(WIFI_WPA_SUPPLICANT)$(DELIM)src$(DELIM)utils

INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)$(WIFI_WPA_SUPPLICANT)$(DELIM)src$(DELIM)utils

CHIP_CSRCS += base64.c
CHIP_CSRCS += bitfield.c
CHIP_CSRCS += common.c
CHIP_CSRCS += ext_password.c
CHIP_CSRCS += json.c
CHIP_CSRCS += uuid.c
CHIP_CSRCS += wpa_debug.c
CHIP_CSRCS += wpabuf.c

VPATH += $(WIFI_WPA_SUPPLICANT)$(DELIM)port

INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)$(WIFI_WPA_SUPPLICANT)$(DELIM)port$(DELIM)include

CHIP_CSRCS += eloop.c
CHIP_CSRCS += os_xtensa.c

## ESP Supplicant (Espressif's WPA supplicant extension)

INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)$(WIFI_WPA_SUPPLICANT)$(DELIM)esp_supplicant$(DELIM)include

VPATH += $(WIFI_WPA_SUPPLICANT)$(DELIM)esp_supplicant$(DELIM)src

INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)$(WIFI_WPA_SUPPLICANT)$(DELIM)esp_supplicant$(DELIM)src

CHIP_CSRCS += esp_common.c
CHIP_CSRCS += esp_hostap.c
CHIP_CSRCS += esp_wpa_main.c
CHIP_CSRCS += esp_wpa3.c
CHIP_CSRCS += esp_wpas_glue.c
CHIP_CSRCS += esp_owe.c
CHIP_CSRCS += esp_scan.c
CHIP_CSRCS += esp_wps.c

VPATH += $(WIFI_WPA_SUPPLICANT)$(DELIM)esp_supplicant$(DELIM)src$(DELIM)crypto

INCLUDES += $(INCDIR_PREFIX)$(ARCH_SRCDIR)$(DELIM)$(WIFI_WPA_SUPPLICANT)$(DELIM)src$(DELIM)crypto

CHIP_CSRCS += crypto_mbedtls-bignum.c
CHIP_CSRCS += crypto_mbedtls-ec.c
CHIP_CSRCS += crypto_mbedtls-rsa.c
CHIP_CSRCS += crypto_mbedtls.c
CHIP_CSRCS += tls_mbedtls.c
CHIP_CSRCS += aes-siv.c

endif
