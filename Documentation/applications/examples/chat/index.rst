====================
``chat`` AT over TTY
====================

Demonstrates AT chat functionality over a TTY device. This is useful with AT
modems, for example, to establish a ``pppd`` connection (see the related ``pppd``
example). Moreover, some AT modems – such as ones made by u-blox – have an
internal TCP/IP stack, often with an implementation of TLS/SSL. In such cases
the chat utility can be used to configure the internal TCP/IP stack, establish
socket connections, set up security (e.g., download base64-encoded certificates
to the modem), and perform data exchange through sockets over the TTY device.

Useful configuration parameters:

- ``CONFIG_EXAMPLES_CHAT_PRESET[0..3]`` – preset chat scripts.
- ``CONFIG_EXAMPLES_CHAT_TTY_DEVNODE`` – TTY device node name.
- ``CONFIG_EXAMPLES_CHAT_TIMEOUT_SECONDS`` – default receive timeout.
