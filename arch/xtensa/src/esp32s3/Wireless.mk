############################################################################
# arch/xtensa/src/esp32s3/Wireless.mk
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

WIRELESS_DRV_REPO    = esp-wireless-drivers-3rdparty
WIRELESS_DRV_BRANCH  = release/v2.0
WIRELESS_DRV_URL     = https://github.com/espressif/esp-wireless-drivers-3rdparty.git
WIRELESS_DRV_PATH    = $(ARCH_SRCDIR)$(DELIM)chip$(DELIM)$(WIRELESS_DRV_REPO)

chip/$(WIRELESS_DRV_REPO):
	$(Q) echo "Cloning: ESP Wireless Drivers"
	$(Q) git clone --depth=1 --branch $(WIRELESS_DRV_BRANCH) $(WIRELESS_DRV_URL) chip/$(WIRELESS_DRV_REPO)

context:: chip/$(WIRELESS_DRV_REPO)
	$(Q) echo "ESP Wireless Drivers: ${WIRELESS_DRV_BRANCH}"
	$(Q) git -C chip/$(WIRELESS_DRV_REPO)/esp-idf reset --hard
	$(Q) echo "ESP Wireless Drivers: initialize submodule esp-idf"
	$(Q) git -C chip/$(WIRELESS_DRV_REPO) submodule update --init --depth=1 esp-idf
	$(Q) echo "ESP Wireless Drivers: initialize submodule mbedtls, esp_phy and esp_phy from esp-idf"
	$(Q) git -C chip/$(WIRELESS_DRV_REPO)/esp-idf submodule update --init --depth=1 components/mbedtls/mbedtls components/esp_phy/lib components/esp_wifi/lib
	$(Q) git -C chip/$(WIRELESS_DRV_REPO)/esp-idf/components/mbedtls/mbedtls reset --hard
	$(Q) echo "ESP Wireless Drivers: apply patches for NuttX"
	$(Q) cd chip/$(WIRELESS_DRV_REPO)/esp-idf && git apply ../nuttx/patches/esp-idf/*.patch
	$(Q) cd chip/$(WIRELESS_DRV_REPO)/esp-idf/components/mbedtls/mbedtls && git apply ../../../../nuttx/patches/esp-idf/submodules/mbedtls/*.patch

distclean::
	$(call DELDIR, chip/$(WIRELESS_DRV_REPO))

INCLUDES += $(shell $(INCDIR) "$(CC)" $(ARCH_SRCDIR)$(DELIM)chip$(DELIM)esp-wireless-drivers-3rdparty$(DELIM)nuttx$(DELIM)include$(DELIM)esp32s3)
INCLUDES += $(shell $(INCDIR) "$(CC)" $(ARCH_SRCDIR)$(DELIM)chip$(DELIM)esp-wireless-drivers-3rdparty$(DELIM)esp-idf$(DELIM)components$(DELIM)esp_wifi$(DELIM)include)
INCLUDES += $(shell $(INCDIR) "$(CC)" $(ARCH_SRCDIR)$(DELIM)chip$(DELIM)esp-wireless-drivers-3rdparty$(DELIM)esp-idf$(DELIM)components$(DELIM)esp_phy$(DELIM)include)
INCLUDES += $(shell $(INCDIR) "$(CC)" $(ARCH_SRCDIR)$(DELIM)chip$(DELIM)esp-wireless-drivers-3rdparty$(DELIM)esp-idf$(DELIM)components$(DELIM)esp_phy$(DELIM)esp32s3$(DELIM)include)
INCLUDES += $(shell $(INCDIR) "$(CC)" $(ARCH_SRCDIR)$(DELIM)chip$(DELIM)esp-wireless-drivers-3rdparty$(DELIM)esp-idf$(DELIM)components$(DELIM)esp_common$(DELIM)include)
INCLUDES += $(shell $(INCDIR) "$(CC)" $(ARCH_SRCDIR)$(DELIM)chip$(DELIM)esp-wireless-drivers-3rdparty$(DELIM)esp-idf$(DELIM)components$(DELIM)esp_event$(DELIM)include)
INCLUDES += $(shell $(INCDIR) "$(CC)" $(ARCH_SRCDIR)$(DELIM)chip$(DELIM)esp-wireless-drivers-3rdparty$(DELIM)esp-idf$(DELIM)components$(DELIM)nvs_flash$(DELIM)include)
INCLUDES += $(shell $(INCDIR) "$(CC)" $(ARCH_SRCDIR)$(DELIM)chip$(DELIM)esp-wireless-drivers-3rdparty$(DELIM)esp-idf$(DELIM)components$(DELIM)esp_system$(DELIM)include)
INCLUDES += $(shell $(INCDIR) "$(CC)" $(ARCH_SRCDIR)$(DELIM)chip$(DELIM)esp-wireless-drivers-3rdparty$(DELIM)esp-idf$(DELIM)components$(DELIM)esp_hw_support$(DELIM)include)
INCLUDES += $(shell $(INCDIR) "$(CC)" $(ARCH_SRCDIR)$(DELIM)chip$(DELIM)esp-wireless-drivers-3rdparty$(DELIM)esp-idf$(DELIM)components$(DELIM)esp_timer$(DELIM)include)
INCLUDES += $(shell $(INCDIR) "$(CC)" $(ARCH_SRCDIR)$(DELIM)chip$(DELIM)esp-wireless-drivers-3rdparty$(DELIM)esp-idf$(DELIM)components$(DELIM)esp_rom$(DELIM)include)
INCLUDES += $(shell $(INCDIR) "$(CC)" $(ARCH_SRCDIR)$(DELIM)chip$(DELIM)esp-wireless-drivers-3rdparty$(DELIM)esp-idf$(DELIM)components$(DELIM)esp_rom$(DELIM)include$(DELIM)esp32s3)
INCLUDES += $(shell $(INCDIR) "$(CC)" $(ARCH_SRCDIR)$(DELIM)chip$(DELIM)esp-wireless-drivers-3rdparty$(DELIM)esp-idf$(DELIM)components$(DELIM)soc$(DELIM)esp32s3$(DELIM)include)

EXTRA_LIBPATHS += -L $(ARCH_SRCDIR)$(DELIM)chip$(DELIM)esp-wireless-drivers-3rdparty$(DELIM)esp-idf$(DELIM)components$(DELIM)esp_phy$(DELIM)lib$(DELIM)esp32s3
EXTRA_LIBPATHS += -L $(ARCH_SRCDIR)$(DELIM)chip$(DELIM)esp-wireless-drivers-3rdparty$(DELIM)esp-idf$(DELIM)components$(DELIM)esp_wifi$(DELIM)lib$(DELIM)esp32s3

EXTRA_LIBS += -lphy

# Wireless interfaces.

CHIP_CSRCS += esp32s3_wireless.c

ifeq ($(CONFIG_ESP32S3_WIFI),y)
CHIP_CSRCS += esp32s3_wlan.c esp32s3_wifi_utils.c esp32s3_wifi_adapter.c
EXTRA_LIBS += -lcore -lnet80211 -lpp

ifeq ($(CONFIG_WPA_WAPI_PSK),y)
EXTRA_LIBS += -lwapi
endif

## ESP-IDF's mbedTLS

VPATH += chip$(DELIM)esp-wireless-drivers-3rdparty$(DELIM)esp-idf$(DELIM)components$(DELIM)mbedtls$(DELIM)mbedtls$(DELIM)library

INCLUDES += $(shell $(INCDIR) "$(CC)" $(ARCH_SRCDIR)$(DELIM)chip$(DELIM)esp-wireless-drivers-3rdparty$(DELIM)esp-idf$(DELIM)components$(DELIM)mbedtls$(DELIM)mbedtls$(DELIM)include)
INCLUDES += $(shell $(INCDIR) "$(CC)" $(ARCH_SRCDIR)$(DELIM)chip$(DELIM)esp-wireless-drivers-3rdparty$(DELIM)esp-idf$(DELIM)components$(DELIM)mbedtls$(DELIM)mbedtls$(DELIM)library)
INCLUDES += $(shell $(INCDIR) "$(CC)" $(ARCH_SRCDIR)$(DELIM)chip$(DELIM)esp-wireless-drivers-3rdparty$(DELIM)esp-idf$(DELIM)components$(DELIM)mbedtls$(DELIM)port$(DELIM)include)

### Define Espressif's configs for mbedTLS

CFLAGS += ${DEFINE_PREFIX}MBEDTLS_CONFIG_FILE="<mbedtls/esp_config.h>"

CHIP_CSRCS += aes.c
CHIP_CSRCS += bignum.c
CHIP_CSRCS += constant_time.c
CHIP_CSRCS += ctr_drbg.c
CHIP_CSRCS += ecp.c
CHIP_CSRCS += ecp_curves.c
CHIP_CSRCS += entropy.c
CHIP_CSRCS += md.c
CHIP_CSRCS += pkcs5.c
CHIP_CSRCS += platform.c
CHIP_CSRCS += platform_util.c
CHIP_CSRCS += sha1.c
CHIP_CSRCS += sha256.c
CHIP_CSRCS += sha512.c

VPATH += chip$(DELIM)esp-wireless-drivers-3rdparty$(DELIM)esp-idf$(DELIM)components$(DELIM)mbedtls$(DELIM)port

CHIP_CSRCS += esp_hardware.c
CHIP_CSRCS += esp_mem.c
CHIP_CSRCS += esp_timing.c

VPATH += chip$(DELIM)esp-wireless-drivers-3rdparty$(DELIM)esp-idf$(DELIM)components$(DELIM)mbedtls$(DELIM)port$(DELIM)md

CHIP_CSRCS += esp_md.c

## WPA Supplicant

WIFI_WPA_SUPPLICANT = chip$(DELIM)$(WIRELESS_DRV_REPO)$(DELIM)esp-idf$(DELIM)components$(DELIM)wpa_supplicant

CFLAGS += ${DEFINE_PREFIX}CONFIG_CRYPTO_MBEDTLS
CFLAGS += ${DEFINE_PREFIX}CONFIG_ECC
CFLAGS += ${DEFINE_PREFIX}CONFIG_IEEE80211W
CFLAGS += ${DEFINE_PREFIX}CONFIG_WPA3_SAE
CFLAGS += ${DEFINE_PREFIX}EAP_PEER_METHOD
CFLAGS += ${DEFINE_PREFIX}ESPRESSIF_USE
CFLAGS += ${DEFINE_PREFIX}ESP_PLATFORM
CFLAGS += ${DEFINE_PREFIX}ESP_SUPPLICANT
CFLAGS += ${DEFINE_PREFIX}IEEE8021X_EAPOL
CFLAGS += ${DEFINE_PREFIX}USE_WPA2_TASK
CFLAGS += ${DEFINE_PREFIX}__ets__

ifeq ($(CONFIG_ESP_WIFI_GCMP_SUPPORT),y)
CFLAGS += ${DEFINE_PREFIX}CONFIG_GCMP
endif

ifeq ($(CONFIG_ESP_WIFI_GMAC_SUPPORT),y)
CFLAGS += ${DEFINE_PREFIX}CONFIG_GMAC
endif

INCLUDES += $(shell $(INCDIR) "$(CC)" $(ARCH_SRCDIR)$(DELIM)$(WIFI_WPA_SUPPLICANT)$(DELIM)include)
INCLUDES += $(shell $(INCDIR) "$(CC)" $(ARCH_SRCDIR)$(DELIM)$(WIFI_WPA_SUPPLICANT)$(DELIM)src)

VPATH += $(WIFI_WPA_SUPPLICANT)$(DELIM)src$(DELIM)ap

INCLUDES += $(shell $(INCDIR) "$(CC)" $(ARCH_SRCDIR)$(DELIM)$(WIFI_WPA_SUPPLICANT)$(DELIM)src$(DELIM)ap)

CHIP_CSRCS += ap_config.c
CHIP_CSRCS += sta_info.c
CHIP_CSRCS += wpa_auth.c
CHIP_CSRCS += wpa_auth_ie.c

VPATH += $(WIFI_WPA_SUPPLICANT)$(DELIM)src$(DELIM)common

INCLUDES += $(shell $(INCDIR) "$(CC)" $(ARCH_SRCDIR)$(DELIM)$(WIFI_WPA_SUPPLICANT)$(DELIM)src$(DELIM)common)

CHIP_CSRCS += dragonfly.c
CHIP_CSRCS += sae.c
CHIP_CSRCS += wpa_common.c

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
CHIP_CSRCS += eap.c
CHIP_CSRCS += eap_common.c
CHIP_CSRCS += eap_mschapv2.c
CHIP_CSRCS += eap_peap.c
CHIP_CSRCS += eap_peap_common.c
CHIP_CSRCS += eap_tls.c
CHIP_CSRCS += eap_tls_common.c
CHIP_CSRCS += eap_ttls.c
CHIP_CSRCS += mschapv2.c

VPATH += $(WIFI_WPA_SUPPLICANT)$(DELIM)src$(DELIM)rsn_supp

CHIP_CSRCS += pmksa_cache.c
CHIP_CSRCS += wpa.c
CHIP_CSRCS += wpa_ie.c

VPATH += $(WIFI_WPA_SUPPLICANT)$(DELIM)src$(DELIM)utils

INCLUDES += $(shell $(INCDIR) "$(CC)" $(ARCH_SRCDIR)$(DELIM)$(WIFI_WPA_SUPPLICANT)$(DELIM)src$(DELIM)utils)

CHIP_CSRCS += base64.c
CHIP_CSRCS += bitfield.c
CHIP_CSRCS += common.c
CHIP_CSRCS += ext_password.c
CHIP_CSRCS += json.c
CHIP_CSRCS += uuid.c
CHIP_CSRCS += wpa_debug.c
CHIP_CSRCS += wpabuf.c

VPATH += $(WIFI_WPA_SUPPLICANT)$(DELIM)port

INCLUDES += $(shell $(INCDIR) "$(CC)" $(ARCH_SRCDIR)$(DELIM)$(WIFI_WPA_SUPPLICANT)$(DELIM)port$(DELIM)include)

CHIP_CSRCS += eloop.c
CHIP_CSRCS += os_xtensa.c

## ESP Supplicant (Espressif's WPA supplicant extension)

INCLUDES += $(shell $(INCDIR) "$(CC)" $(ARCH_SRCDIR)$(DELIM)$(WIFI_WPA_SUPPLICANT)$(DELIM)esp_supplicant$(DELIM)include)

VPATH += $(WIFI_WPA_SUPPLICANT)$(DELIM)esp_supplicant$(DELIM)src

INCLUDES += $(shell $(INCDIR) "$(CC)" $(ARCH_SRCDIR)$(DELIM)$(WIFI_WPA_SUPPLICANT)$(DELIM)esp_supplicant$(DELIM)src)

CHIP_CSRCS += esp_common.c
CHIP_CSRCS += esp_hostap.c
CHIP_CSRCS += esp_wpa2.c
CHIP_CSRCS += esp_wpa3.c
CHIP_CSRCS += esp_wpa_main.c
CHIP_CSRCS += esp_wpas_glue.c

VPATH += $(WIFI_WPA_SUPPLICANT)$(DELIM)esp_supplicant$(DELIM)src$(DELIM)crypto

INCLUDES += $(shell $(INCDIR) "$(CC)" $(ARCH_SRCDIR)$(DELIM)$(WIFI_WPA_SUPPLICANT)$(DELIM)src$(DELIM)crypto)

CHIP_CSRCS += crypto_mbedtls.c
CHIP_CSRCS += crypto_mbedtls-bignum.c
CHIP_CSRCS += crypto_mbedtls-ec.c
CHIP_CSRCS += crypto_mbedtls-rsa.c
CHIP_CSRCS += tls_mbedtls.c

endif
