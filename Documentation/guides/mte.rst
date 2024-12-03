====================================
ATM64 MTE extension
====================================

Introduction
------------

Arm v8.5 introduced the Arm Memory Tagging Extension (MTE),
a hardware implementation of tagged memory.

Basically, MTE tags every memory allocation/deallocation
with additional metadata. It assigns a tag to a memory location,
which can then be associated with a pointer that references
that memory location. At runtime, the CPU checks that the pointer
and metadata tags match with every load and store.

NX OS currently supports deploying MTE on ARM64 QEMU,
which is supported at the EL1 level of NX OS.

Principle
---------

The Arm Memory Tagging Extension implements lock and key access to memory.
Locks can be set on memory and keys provided during memory access. If the key matches
the lock, the access is permitted. If it does not match, an error is reported.

Memory locations are tagged by adding four bits of metadata to each 16 bytes
of physical memory. This is the Tag Granule. Tagging memory implements the lock.
Pointers, and therefore virtual addresses, are modified to contain the key.
In order to implement the key bits without requiring larger pointers MTE uses the Top Byte
Ignore (TBI) feature of the Armv8-A Architecture. When TBI is enabled, the top byte of
a virtual address is ignored when using it as an input for address translation. This allows the
top byte to store metadata. In MTE four bits of the top byte are used to provide the key

Architectural Details
---------------------

MTE adds instructions to the Armv8-A Architecture that are outlined below and grouped
into three different categories [6]:
Instructions for tag manipulation applicable to stack and heap tagging.

IRG
In order for the statistical basis of MTE to be valid, a source of random tags is required.
IRG is defined to provide this in hardware and insert such a tag into a register for use
by other instructions.

GMI
This instruction is for manipulating the excluded set of tags for use with the IRG instruction.
This is intended for cases where software uses specific tag values for special purposes
while retaining random tag behavior for normal allocations.

LDG, STG, and STZG
These instructions allow getting or setting tags in memory. They are intended for changing
tags in memory either without modifying the data or zeroing the data.

ST2G and STZ2G
These are denser alternatives to STG and STZG which operate on two granules of memory
when allocation size allows them to be used.

STGP
This instruction stores both tag and data to memory.
Instructions Intended for pointer arithmetic and stack tagging:

ADDG and SUBG
These are variants of the ADD and SUB instructions, intended for arithmetic on addresses.
They allow both the tag and address to be separately modified by an immediate value.
These instructions are intended for creating the addresses of objects on the stack.

SUBP(S)
This instruction provides a 56-bit subtract with optional flag setting which is required
for pointer arithmetic that ignores the tag in the top byte.

Instructions intended for system use:
LDGM, STGM, and STZGM
These are bulk tag manipulation instructions which are UNDEFINED at EL0. These are
intended for system software to manipulate tags for the purposes of initialization and
serialization. For example, they can be used to implement swapping of tagged memory
to a medium which is not tag-aware. The zeroing form can be used for efficient
initialization of memory.

Currently NX OS supports the execution of the above instructions,
such as irg, ldg, stg instructions.
Their test programs are stored in "apps/system/mte" to test whether the current system supports

Usage
-----

If you want to experience the MTE function of NX OS, you can refer to the following：
To enable ARM64_MTE, configure the kernel with::

    CONFIG_ARM64_MTE=y

Of course you can also run it with the existing configuration：

    boards/arm64/qemu/qemu-armv8a/configs/mte
