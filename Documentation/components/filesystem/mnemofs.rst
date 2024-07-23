=======
MNEMOFS
=======

Mnemofs is a NAND Flash File System built for NuttX.

Usage
=====

If there's a NAND flash available at a location, for example, ``/dev/nand``,
you can mount it with ``mnemofs`` to a location like ``/mydir`` using::

    mount -t mnemofs /dev/nand /mydir

The above command will only work if the device was already formatted using
mnemofs. For a brand new device, or if you want to switch from an existing
file system, this won't work, and would need a format.

Instead try this::

    mount -t mnemofs -o forceformat /dev/nand /mydir

Unsure of whether you need to do a format? This will help::

    mount -t mnemofs -o autoformat /dev/nand /mydir

This will format the device only if it can not detect mnemofs being already
formatted onto it. Do note this includes cases where mnemofs is formatted to
the device, but it's been mutilated to the point of being unrecognizable.

After this, use it like a regular file system. That's the job of a file
system after all...to hide the storage device's pecularities behind an
abstraction. A file system is considered good if you don't have to think
about its existence during regular usage.

Design
======

mnemofs is designed to be a middle ground between flash storage consumption,
memory consumption, wear and speed. It sacrifices a little bit of everything,
and ends up being acceptably good in all of them, instead of sacrificing
multiple aspects, and being good in one.

mnemofs consists of several components, however, a walkthrough of the process
where a change requested by a user ends up being written to the NAND flash
would serve well for an introduction. The details will be explained further
below.

The user requests some changes, say, add ``x`` bytes to ``y`` offset in a file.
This change is copied into the LRU cache of mnemofs. This LRU cache exists
in-memory, and serves as a tool for wear reduction.

This LRU cache is a kernel list of nodes. Each node represents a file or a
directory. When the LRU is full, the last node is popped from this list and
the changes it contains, which is an accumulation of changes requested by
the user for that particular file or directory since the node was added to
the LRU cache, is written to the flash.

Each file or directory is represented by a `CTZ skip list <https://github.com/littlefs-project/littlefs/blob/master/DESIGN.md#ctz-skip-lists>`_,
and the only attributes required to access the list is the index of the last
CTZ skip list block, the page number of that CTZ skip list block, and the
size of the file. In mnemofs, CTZ skip list blocks take up exactly one page
on the flash.

Mnemofs works in a Copy-On-Write manner, similar to littlefs. When a CTZ
skip list is updated, the new location is added to the Journal of mnemofs
as a log. This log contains some information about the location of the new
CTZ list, the path it belongs to, etc. and then the updated location is
added as an update to its parent's CTZ skip list, and it undergoes the same
process. This log is appended with a checksum of the entire log, which
gives an assurance that the saved log was indeed saved completely before a
power loos.

The journal is a modified singly linked list of blocks on the flash that
contains logs of changes in the file system. The last two blocks of the
journal is reserved for master blocks, hence the number of blocks in the
journal will be referred to as ``n + 2`` blocks.

The area on storage other than the journal contains a certain "base" state of
the file system. All changes to the base state since is written to the
journal. The first block of the journal starts with an 8 byte magic sequence
to identify the start of the journal (on mount), followed by the number of
blocks in the journal and then finally an array of all the ``n + 2`` block
numbers that are part of the journal. After this part, the entire area in the
``n`` blocks contain logs and their checksums.

The last two blocks of a journal are called the master blocks, and they store
multiple instances of the master node. They are duplicates of each other, and
each instance of the master node takes one page each, and are written to
these master blocks in a sequential manner. The master node points to the
root.

When the first ``n`` blocks of the journal are full, then they are flushed
and since the root updates here as well, a new master node is written. Once
the new master node is written, the file system's base state is updated and
thus the old obsolete pages can be erased (if possible). The first ``n``
blocks of the journal move more than the master nodes.

The block allocator of mnemofs is havily inspired from littlefs. It starts
from a random block, and starts allocating pages or blocks sequentially in a
circular manner. It skips pages upon block requirement, but since block
requirements are only required by internal structures, they are always
requested in bulk, and minimize wastage. However, unlike in littlefs, mnemofs
keeps a bitmap in memory about the pages that are currently being used, as
well as the count of pages inside each block that want to be erased.

In mnemofs, the bind might take a lot of time in the worst possible
theoretical case, as it's an ``O(n)`` mounting process, however, it's not the
case in real life. Mnemofs only needs to scan the first page of every block
in the device to look for the start of the journal. Leaving the actual
location of the page aside, this will be pretty fast in real life as the
larger the storage capacity is, the larger are the pages and the larger are
the number of pages per block, and thus the number of blocks in the device
do not increase at a rate similar to the increase in storage capacity of the
device. Further, the journal has the journal array, which contains block
numbers of each block in it, very close to the start of the array, and
mnemofs can quickly jump from there to the latest master node, and scan
the file system for used pages.