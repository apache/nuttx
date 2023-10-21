``mqttc``
=========

This is a simple MQTT publisher example using MQTT-C

By default it publishes to the "test" topic and exits.  Default behaviour
including, host, port, topic, message and loop count can be changed through
different arguments.

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
