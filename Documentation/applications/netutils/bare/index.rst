===========================================
``bare`` Binary Application Record Encoding
===========================================

BARE is a simple binary representation for structured application data, see
https://baremessages.org/.

BARE implementation ported to NuttX is ``cbare``, see
https://git.sr.ht/~fsx/cbare.

BARE_ is similar to other binary formats like CBOR_, BSON_, or `Protocol
Buffers`_.

.. _BARE: https://baremessages.org/
.. _CBOR: https://cbor.io/
.. _BSON: https://bsonspec.org/
.. _Protocol Buffers: https://protobuf.dev/

BARE at glance, from its web page:

- Messages are encoded in binary and compact in size. Messages do not contain
  schema information — they are not self-describing.

- BARE is optimized for small messages. It is not optimized for encoding large
  amounts of data in a single message, or efficiently reading a message with
  fields of a fixed size. However, all types are aligned to 8 bits, which does
  exchange some space for simplicity.

- BARE's approach to extensibility is conservative: messages encoded today will
  be decodable tomorrow, and vice-versa. But extensibility is still possible;
  implementations can choose to decode user-defined types at a higher level and
  map them onto arbitrary data types.

- The specification is likewise conservative. Simple implementations of message
  decoders and encoders can be written inside of an afternoon.

- An optional DSL is provided to document message schemas and provide a source
  for code generation. However, if you prefer, you may also define your schema
  using the type system already available in your programming language.
