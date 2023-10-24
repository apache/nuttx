===========================
``nxscope`` NxScope Library
===========================

This library provides real-time data logging functionality for NuttX.

The principle of action is to accumulate data gathered in virtual channels
and periodically send buffered data through a dedicated interface packed
with a custom protocol.

Supported features:

- up to 255 channels possible
- support for standard data types and user-specific data (``enum nxscope_sample_dtype_e``)
- support for vector data or point data
- support for character-based channels (text messages)
- support for channel metadata - can be used to enumerate samples or timestamp
- stream buffer overflow detection (``NXSCOPE_STREAM_FLAGS_OVERFLOW``)
- remote control with commands (``enum nxscope_hdr_id_e``)
- protocol and interface implementation can be different for control commands and stream data
- (optional) support for user-specific commands (``NXSCOPE_HDRID_USER`` and ``struct nxscope_callbacks_s``)
- (optional) support for samples divider (``CONFIG_LOGGING_NXSCOPE_DIVIDER``)
- (optional) support for ACK frames (``CONFIG_LOGGING_NXSCOPE_ACKFRAMES``)
- (optional) support for user-defined types (``CONFIG_LOGGING_NXSCOPE_USERTYPES``)
- (optional) support for non-buffered critical channels (``CONFIG_LOGGING_NXSCOPE_CRICHANNELS``)

A custom interface and a custom protocol can be implemented with
``struct nxscope_intf_s`` and ``struct nxscope_proto_s`` structures.

Supported interfaces:

1. a serial port: ``logging/nxscope/nxscope_iser.c``
2. a dummy interface for debug purposes: ``logging/nxscope/nxscope_idummy.c``

A default serial protocol is implemented in ``apps/logging/nxscope/nxscope_pser.c``
It just packs NxScope data into simple frames with a CRC-16 checksum.

External tools
--------------

- `Nxslib <https://github.com/railab/nxslib>`_ - a Python (3.10+) client library for NxScope devices,
- `Nxscli <https://github.com/railab/nxscli>`_ - a Python (3.10+) command-line interface for NxScope,
  supporting data capture and visualization
