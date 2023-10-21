``can`` CAN Device Test
=======================

If the CAN device is configured in loopback mode, then this example can be used
to test the CAN device in loop back mode. It simple sinces a sequence of CAN
messages and verifies that those messages are returned exactly as sent.

This test depends on these specific CAN/NSH configurations settings (your
specific CAN settings might require additional settings).

- ``CONFIG_CAN`` – Enables CAN support.
- ``CONFIG_CAN_LOOPBACK`` – A CAN driver may or may not support a loopback mode
  for testing. The STM32 CAN driver does support loopback mode.
- ``CONFIG_NSH_BUILTIN_APPS`` – Build the CAN test as an NSH built-in function.
  Default: Built as a standalone program.

Specific configuration options for this example include:

- ``CONFIG_EXAMPLES_CAN_DEVPATH`` – The path to the CAN device. Default:
  ``/dev/can0``.
- ``CONFIG_EXAMPLES_CAN_NMSGS`` – This number of CAN message is collected and the
  program terminates. Default: messages are sent and received indefinitely.

The default behavior assumes loopback mode. Messages are sent, then read and
verified. The behavior can be altered for other kinds of testing where the test
only sends or received (but does not verify) can messages.

- ``CONFIG_EXAMPLES_CAN_READONLY`` – Only receive messages.
- ``CONFIG_EXAMPLES_CAN_WRITEONLY`` – Only send messages.
