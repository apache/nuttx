=========================================
``mbedtls`` Mbed TLS Cryptography Library
=========================================

NuttX can build the Mbed TLS cryptography library from ``nuttx-apps`` (under
``apps/crypto/mbedtls``). Enable ``CONFIG_CRYPTO_MBEDTLS`` in ``menuconfig`` and
select application options as needed.

Mbed TLS is commonly used for TLS clients and servers on NuttX. For MQTT over TLS
with the MQTT-C integration, enable ``CONFIG_CRYPTO_MBEDTLS`` and
``CONFIG_NETUTILS_MQTTC_WITH_MBEDTLS``; see :doc:`../../netutils/mqttc/index`.
