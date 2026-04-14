========================
``mqttc`` MQTT-C library
========================

Overview
========

The `MQTT-C <https://github.com/LiamBindle/MQTT-C>`_ client library is integrated
into NuttX through ``nuttx-apps`` (``apps/netutils/mqttc``). It provides an MQTT
v3.1.1 client with a small platform abstraction layer.

You can use MQTT-C over plain TCP or, when enabled, over TLS using Mbed TLS. The
TLS path defines ``MQTT_USE_MBEDTLS`` for code that includes the library and the
``examples/mqttc`` publisher application.

Prerequisites
=============

- A NuttX tree and a matching ``nuttx-apps`` checkout (see the top-level
  ``README`` in each repository).
- A working network stack and route to your MQTT broker (Ethernet, Wi-Fi, or
  other), unless you only run loopback tests on the host.
- For TLS with certificate verification, ensure the device has a valid clock
  (RTC or NTP) before connecting; otherwise verification of ``notBefore`` /
  ``notAfter`` may fail.

Configuration
=============

Enable the MQTT-C package and optional pieces from ``menuconfig``:

**Library and TLS**

- ``CONFIG_NETUTILS_MQTTC``: Build the MQTT-C static library.
- ``CONFIG_CRYPTO_MBEDTLS``: Build Mbed TLS (required for the TLS integration).
- ``CONFIG_NETUTILS_MQTTC_WITH_MBEDTLS``: Compile MQTT-C and dependent apps with
  ``MQTT_USE_MBEDTLS``. This option depends on ``CRYPTO_MBEDTLS`` and selects
  ``DEV_URANDOM`` for entropy.
- ``CONFIG_NETUTILS_MQTTC_VERSION``: Upstream MQTT-C version string (default is
  ``1.1.5``).

**Example: ``mqttc_pub`` (``apps/examples/mqttc``)**

- ``CONFIG_EXAMPLES_MQTTC``: Build the NSH publisher example. Program name is
  ``CONFIG_EXAMPLES_MQTTC_PROGNAME`` (default ``mqttc_pub``). Requires
  ``NETUTILS_MQTTC``.
- ``CONFIG_EXAMPLES_MQTTC_ALLOW_UNVERIFIED_TLS``: If TLS verification fails,
  continue anyway. Intended for development with self-signed brokers; do not
  rely on this for production.

**Bundled upstream examples (``apps/netutils/mqttc``)**

- ``CONFIG_NETUTILS_MQTTC_EXAMPLE``: Build extra sample programs from the
  MQTT-C tree. With Mbed TLS enabled this produces ``mqttc_mbedtls_pub``;
  otherwise ``mqttc_posix_pub`` and ``mqttc_posix_sub``.
- ``CONFIG_NETUTILS_MQTTC_TEST``: CMocka-based tests. This option is not
  available when ``CONFIG_NETUTILS_MQTTC_WITH_MBEDTLS`` is enabled.

A minimal ``kconfig`` fragment for TLS-enabled ``mqttc_pub`` might look like:

.. code-block:: kconfig

   CONFIG_CRYPTO_MBEDTLS=y
   CONFIG_NETUTILS_MQTTC=y
   CONFIG_NETUTILS_MQTTC_WITH_MBEDTLS=y
   CONFIG_EXAMPLES_MQTTC=y

Using ``mqttc_pub`` with Mbed TLS
=================================

When ``CONFIG_NETUTILS_MQTTC_WITH_MBEDTLS`` is set, ``mqttc_pub`` uses Mbed TLS
for the broker connection. The default broker port is **8883** (TLS). Typical
arguments:

.. code-block:: text

   mqttc_pub -h BROKER [-p PORT] [-c CAFILE] [-t TOPIC] [-m MESSAGE] [-n COUNT] [-q QOS]

- ``-h``: Broker hostname or address (required for non-default use).
- ``-p``: Port (default ``8883`` in TLS mode).
- ``-c``: Path to a PEM file containing the broker CA certificate (or chain).
  If omitted, the example uses an embedded test CA (PolarSSL/Mbed TLS test
  material), which is only appropriate for matching test servers—not for
  arbitrary production brokers.
- ``-t``, ``-m``, ``-n``, ``-q``: Topic, payload, publish repeat count, and QoS.

Example (NSH, after the network is up):

.. code-block:: text

   nsh> mqttc_pub -h mqtt.example.com -p 8883 -c /etc/ssl/certs/broker-ca.pem

If verification fails and you must use a self-signed broker during bring-up,
enable ``CONFIG_EXAMPLES_MQTTC_ALLOW_UNVERIFIED_TLS`` or fix the CA/time on the
device.

Using ``mqttc_mbedtls_pub``
===========================

When ``CONFIG_NETUTILS_MQTTC_EXAMPLE`` and
``CONFIG_NETUTILS_MQTTC_WITH_MBEDTLS`` are set, the ``mqttc_mbedtls_pub``
program is built from the upstream ``examples/mbedtls_publisher.c``. It expects
positional arguments:

.. code-block:: text

   mqttc_mbedtls_pub CAFILE [ADDRESS [PORT [TOPIC]]]

Defaults are similar to the upstream sample (e.g. public test broker and port
``8883`` if not overridden). Use a CA file that matches your broker.

Build systems (Make and CMake)
==============================

Make-based and CMake-based NuttX builds both support these options. For CMake,
ensure Mbed TLS and MQTT-C targets resolve includes and dependencies; recent
``nuttx-apps`` changes wire ``mqttc`` to ``mbedtls`` when both TLS options are
enabled.

See also
========

- :doc:`../../examples/mqttc/index` — Quick test steps for ``mqttc_pub``.
- :doc:`../../crypto/mbedtls/index` — Mbed TLS package overview.
- :doc:`../paho_mqtt/index` — Eclipse Paho MQTT C client (alternative stack).
