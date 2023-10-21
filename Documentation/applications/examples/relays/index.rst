``relays`` Relays
=================

Requires ``CONFIG_ARCH_RELAYS``. Contributed by Darcy Gong.

**Note**: This test exercises internal relay driver interfaces. As such, it
relies on internal OS interfaces that are not normally available to a user-space
program. As a result, this example cannot be used if a NuttX is built as a
protected, supervisor kernel (``CONFIG_BUILD_PROTECTED`` or
``CONFIG_BUILD_KERNEL``).
