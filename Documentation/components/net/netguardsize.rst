====================
CONFIG_NET_GUARDSIZE
====================

Global Option for All Drivers
=============================

``CONFIG_NET_GUARD_SIZE`` is global option.  It is added to the allocated size of
each driver packet buffer. Currently it is a very small value, defaulting to only
two bytes. So it is not a memory hog and should be added to the packetsize for
all drivers for commonality.  But why?

It should (eventually) be larger and common for all drivers. We need to look at
how it is used today and how it might be used tomorrow. There is a probably a lot
more involved than you might be initially considering.

Packet Receipt
==============

For packet receipt, it is necessary for some hardware, but not for others. Often
the hardware will DMA a 2 byte FCS at the end of the packet or possibly other
hardware-specific info.  But that is only part of the whole story.
``CONFIG_NET_GUARDSIZE`` is not just for hardware packet receipt.

Packet Transmission
===================

There are several issues for packet transmission. These are less well defined
and need further study, but we need to keep all of the driver packet definitions
in place until we understand how we are going to handle these things:

* Memory Overrun Bugs

  There was in the past, a bug that caused write past the end of the buffer by
  a couple of bytes during TX message formatting. I don't know if that bug still
  exists, but the minimum, two-byte ``CONFIG_NET_GUARDSIZE`` was sufficient to
  eliminate the bug. That is why it has the name GUARD: Its primary purpose is
  to protect from overrunning the packet buffer and corrupting the following memory.

  I do no know if we have any such bugs today. Perhaps they still do?
  Perhaps they do not? Having such a guard is a good thing for reliability in
  any case.

* Variable size IP/TCP headers

  There is a limitation in the way IP packets are formatted now. Basically they
  are formatted like this:

    #. When the packet is received a pointer to the location of the payload is
       set (d_appdata). This is an offset into the packet buffer For TCP, that
       accounts for the MAC/Ethernet header, the minimum IPv4/IPv6 header size,
       and the minimum TCP header size.

    #. The TCP payload is written at that location,
    #. The correctly sized IPv4/IPv6 headers and the correctly sized TCP header
       are added below the payload, and finally
    #. The MAC/Ethernet header as added.

  The start offset of the packet in the packet is no longer zero, but some
  variable offset into the packet buffer.  That new start offset would have
  to be passed to driver in order to send the packet.

  The key to making this all work is:

    * Keep ``CONFIG_NET_GUARDSIZE`` in all driver buffers, and
    * Set the ``CONFIG_NET_GUARDSIZE`` to the maximum size of IPv4/IPv6 and TCP options
      (depending on which IP version is enabled and if TCP is enabled)
    * Extend the driver interface to accept data offset into the driver's packet buffer.

* Variable MSS

  Closely related to this is the MSS which is the maximum size of the payload.
  Currently that is a constant because it assumes the minimum header lengths.
  It should be variable, depending on the actual header sizes.
