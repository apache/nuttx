==========================================
``paho_mqtt`` Eclipse Paho MQTT C Library
==========================================

The ``paho_mqtt`` package provides integration of the Eclipse Paho MQTT C
library into NuttX. This library enables MQTT client functionality, supporting
MQTT protocol versions 3.1, 3.1.1, and 5.0. The package includes both a library
for programmatic access and command-line utilities for publishing and
subscribing to MQTT topics.

Overview
========

The Eclipse Paho MQTT C library is a client implementation of the MQTT
protocol. It provides both synchronous and asynchronous APIs for connecting to
MQTT brokers, publishing messages, and subscribing to topics.

This NuttX integration includes:

- **MQTT 5.0 Client Library** (``LIB_MQTT5``): A library providing MQTT client
  functionality through C API calls.

- **Command-line Tools** (``UTILS_MQTT5``):
  - ``mqtt_pub``: A utility for publishing messages to MQTT topics
  - ``mqtt_sub``: A utility for subscribing to MQTT topics and receiving messages

The library is automatically downloaded from the Eclipse Paho GitHub repository
during the build process if not already present.

Configuration
=============

Library Configuration
---------------------

Enable the MQTT 5.0 library:

.. code-block:: kconfig

   CONFIG_LIB_MQTT5=y

Utility Configuration
---------------------

Enable the MQTT command-line utilities:

.. code-block:: kconfig

   CONFIG_UTILS_MQTT5=y
   CONFIG_UTILS_MQTT5_PRIORITY=100
   CONFIG_UTILS_MQTT5_STACKSIZE=16384

Configuration Options
---------------------

- ``CONFIG_LIB_MQTT5``: Enable the MQTT 5.0 client library
- ``CONFIG_UTILS_MQTT5``: Enable MQTT command-line utilities (requires
  ``CONFIG_LIB_MQTT5``)
- ``CONFIG_UTILS_MQTT5_PRIORITY``: Task priority for MQTT utilities (default: 100)
- ``CONFIG_UTILS_MQTT5_STACKSIZE``: Stack size for MQTT utilities (default: 16384)

Usage
=====

mqtt_pub - Publish Messages
----------------------------

The ``mqtt_pub`` utility publishes messages to MQTT topics.

mqtt_pub Syntax
~~~~~~~~~~~~~~~

.. code-block:: bash

   mqtt_pub [topicname] [options]

mqtt_pub Options
~~~~~~~~~~~~~~~~

Connection Options:
  - ``-h, --host <host>``: MQTT broker hostname (default: localhost)
  - ``-p, --port <port>``: Network port (default: 1883)
  - ``-c, --connection <url>``: Connection string (overrides host/port)
  - ``-i, --clientid <id>``: Client ID (default: paho-c-pub)
  - ``-u, --username <user>``: Username for authentication
  - ``-P, --password <pass>``: Password for authentication
  - ``-k, --keepalive <seconds>``: Keepalive timeout (default: 10)

Message Options:
  - ``-t, --topic <topic>``: MQTT topic to publish to
  - ``-m, --message <message>``: Message payload to send
  - ``-f, --filename <file>``: Read message from file
  - ``-q, --qos <0|1|2>``: Quality of Service level (default: 0)
  - ``-r, --retained``: Set retained message flag
  - ``-n, --null-message``: Send zero-length message

MQTT Version:
  - ``-V, --MQTTversion <31|311|5>``: MQTT protocol version (default: 311)

mqtt_pub Examples
~~~~~~~~~~~~~~~~~

Publish a simple message:

.. code-block:: bash

   mqtt_pub -h 192.168.1.100 -t "test/topic" -m "Hello MQTT"

Publish with QoS 1 and retained flag:

.. code-block:: bash

   mqtt_pub -h 192.168.1.100 -t "test/topic" -m "Retained message" -q 1 -r

Publish from a file:

.. code-block:: bash

   mqtt_pub -h 192.168.1.100 -t "test/topic" -f message.txt

mqtt_sub - Subscribe to Topics
-------------------------------

The ``mqtt_sub`` utility subscribes to MQTT topics and receives messages.

mqtt_sub Syntax
~~~~~~~~~~~~~~~

.. code-block:: bash

   mqtt_sub [topicname] [options]

mqtt_sub Options
~~~~~~~~~~~~~~~~

Connection Options:
  - ``-h, --host <host>``: MQTT broker hostname (default: localhost)
  - ``-p, --port <port>``: Network port (default: 1883)
  - ``-c, --connection <url>``: Connection string (overrides host/port)
  - ``-i, --clientid <id>``: Client ID (default: paho-c-sub)
  - ``-u, --username <user>``: Username for authentication
  - ``-P, --password <pass>``: Password for authentication
  - ``-k, --keepalive <seconds>``: Keepalive timeout (default: 10)

Subscription Options:
  - ``-t, --topic <topic>``: MQTT topic to subscribe to (supports wildcards)
  - ``-q, --qos <0|1|2>``: Quality of Service level (default: 0)
  - ``-R, --no-retained``: Do not print retained messages
  - ``--no-delimiter``: Do not use delimiter between messages
  - ``--delimiter <string>``: Custom delimiter (default: \\n)

MQTT Version:
  - ``-V, --MQTTversion <31|311|5>``: MQTT protocol version (default: 311)

Topic Wildcards
~~~~~~~~~~~~~~~

- ``+``: Single-level wildcard (matches one topic level)
  - Example: ``sensor/+/temperature`` matches ``sensor/room1/temperature``
- ``#``: Multi-level wildcard (matches multiple levels, must be at end)
  - Example: ``sensor/#`` matches all topics under ``sensor/``

mqtt_sub Examples
~~~~~~~~~~~~~~~~~

Subscribe to a topic:

.. code-block:: bash

   mqtt_sub -h 192.168.1.100 -t "test/topic"

Subscribe with wildcard:

.. code-block:: bash

   mqtt_sub -h 192.168.1.100 -t "sensor/#"

Subscribe with QoS 1:

.. code-block:: bash

   mqtt_sub -h 192.168.1.100 -t "test/topic" -q 1

Library API
===========

The MQTT 5.0 library provides both synchronous and asynchronous APIs. The main
header files are:

- ``MQTTAsync.h``: Asynchronous MQTT client API
- ``MQTTClient.h``: Synchronous MQTT client API

For detailed API documentation, refer to the Eclipse Paho MQTT C library
documentation at https://www.eclipse.org/paho/clients/c/.
