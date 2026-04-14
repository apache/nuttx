========================
``mqttc`` MQTT-C Example
========================

This is a simple MQTT publisher example using MQTT-C.

By default it publishes to the "test" topic and exits.  Default behaviour
including, host, port, topic, message and loop count can be changed through
different arguments.

Plain TCP (no Mbed TLS)
=======================

To test:
From the host start an MQTT broker and subscribe to the "test" topic.  Here
mosquitto is used::

  mosquitto&
  mosquitto_sub -t test

Make sure that mosquitto is not configured in local mode only.

From the nsh:

Launch the built-in app ``mqttc_pub`` specifying the host::

  mqttc_pub -h HOST

The target will publish the message "test".

TLS with Mbed TLS
=================

To use TLS, enable Mbed TLS and MQTT-C with Mbed TLS in ``menuconfig`` (see
:doc:`../../netutils/mqttc/index`). The same ``mqttc_pub`` binary is built with
TLS support; the default port becomes **8883**. You can pass ``-c`` with a path
to your broker CA certificate in PEM format.

For full configuration symbols, CLI options, and the separate
``mqttc_mbedtls_pub`` example, see :doc:`../../netutils/mqttc/index`.
