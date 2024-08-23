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
file system, this won't work, and would need a format::

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

NAND Flashes
============

Programmatically, the NAND flash has some problems. The whole device can be
condensed into three layers: blocks, pages and cells.

Cells represent the smallest unit of storage in NAND flashes, but are often
ignored, as direct access is not allowed. If a cell stores one bit, it's a
Single Level Cell. There are MLC, TLC, etc. for more bits per cell. Often,
the more bits per cell, the lesser is the wear resilience. Thus, higher
bits per cell are easier to wear out and become unreliable.

Pages are the smallest readable or writable unit of the NAND flash. It's
made up of several cells, and can be expected to have a size of the similar
order of 512 B.

Blocks are the smallest erasable unit of NAND flash. They are made up of
several pages. If a page is already written, it needs to be erased before it
can be written again. And since blocks are the smallest erasable unit, the
entire block needs to be erased if the user wants to update the contents of
one page.

The erase operation is what causes a block to wear out. If a block is worn
out too much, it will lose its ability to reliably store data. An unreliable
block can not guarantee that the data read from the pages in it is the same
as what was written to it. This state is called as a bad block.

A manufacturer can also deem a block to be unreliable from their testing,
and can mark them as bad blocks right from manufacture.

A good file system will aim to level out the wear between blocks as much as
it can.

Design
======

There are various layers and components in mnemofs, and they interact with
various layers on abstraction over each other.

Mnemofs works on a Copy-On-Write (CoW) basis, which means, if a page needs to
be updated, it is copied over in memory, and then the change is applied, and
the new data is written to a new location.

R/W Layer
---------

This works with the NAND flash device driver directly. It can write an
entire page, read an entire page, erase an entire block, check if a block is
bad (from it's bad block marker), or set a block as bad. It's the simplest
layer.

Block Allocator
---------------

The block allocator contains two arrays. One is a bit mask is for tracking
the free pages, while the other is an array of numbers, one number for each
block, denoting the number of pages in that block that are ready to be
erased.

The block allocator allocates pages or blocks in a sequential manner to keep
it fair for all pages, thus, ensuring wear levelling. It also starts from a
random offset to prevent bias to the front of the device in case of multiple
power losses and reinitialization in such casses. If a block is required it
skips pages to the start of the next block. Since block allocations happen
only in the journal, they happen in bulk and the number of skipped pages is
very minimal.

Once reaching the end of the device, it cycles back to the front. Thus any
skipped pages get the chance to be allocated in the next cycle.

CTZ Layer
---------

This works with the R/W Layer, and acts as an abstraction layer for other
components in mnemofs. Mnemofs uses
`CTZ lists <https://github.com/littlefs-project/littlefs/blob/master/DESIGN.md#ctz-skip-lists>`_
to represent both files and directories in flash. CTZ lists of files contain
only the data of the file, while CTZ lists of directories contain directory
entries (direntries) for each FS Object (file or directory) grouped into it.

This layer abstracts away the complex division of flash space that's present
in CTZ skip lists, and allows users of this layer to not worry about the
complexities of a CTZ skip list, and infact, to feel that the data is like a
contiguous space.

This layer allows the user to specify a data offset, which refers to the
offset into the actual data stored in the CTZ skip list (ie. excluding the
pointers), and the number of bytes, and perform operations on the CTZ list
almost like if it were a single array.

In mnemofs, each CTZ block takes up the space of exactly 1 page, and each
pointer takes up 4 bytes.

Littlefs design document shows how a CTZ list can be identified using the
index of the last CTZ block, and the page number of that CTZ block.

Journal
-------

The journal in mnemofs is made out of ``n + 2`` blocks. The last two block
concern the master node. These blocks are arranged in a singly linked list.

Due to CoW policy, when a CTZ list is updated, it now has a new location. The
first ``n`` blocks of the journal is responsible for storing logs containing
information about this very update. It will contain the old location of the
CTZ skip list, and the new location.

Thus, when the user requires an updated location of a CTZ list, they will
first find the old location by traversing the FS tree in the flash, and then
will traverse the journal to find the latest location.

So, the FS tree on the flash acts like a "base" state with updates stored in
the journal. Each log in journal is followed by a checksum to verify if all
of it was written properly. This helps in making it power loss resilient.

The journal, when combined with CoW, plays another important role. In pure
CoW, any update to a CTZ file will result in it having a new location. This
new location wil need to be updated in the parent, which itself will have a
new location after the update, and so on till it reaches the root. The
journal stops this propagation immediately. When the journal is full above
a certain limit, it will flush, and apply all of these changes to the FS
tree in one go. This helps in wear reduction.

The journal mainly works with the CTZ layer, and any updates to a CTZ list
using this layer automatically adds a log for it in the journal.

The journal starts with a magic sequence, then the number of blocks in the
journal (excluding master blocks), and then follows an array with the block
numbers of the blocks in the journal (including the master blocks). Following
this, logs are stored in the blocks.

Master Node and Root
--------------------

The root of the file system is treated like any directory as far as its
storage on the flash is concerned. This is because the master node acts as a
parent to the root, and contains information of the root in a way identical
to direntries.

The master node is stored in the master blocks. There are two master blocks,
and both are duplicated of each other for backup. Each master block is a
block, and thus have multiple pages in them. Each page contains one revision
of the master node. The master nodes are written sequentially, and have a
timestamp on them as well.

When a CTZ list is moved to a new location, the obsolete pages of the old
CTZ list are marked for deletion.

LRU Cache
---------

Mnemofs has a Least Recently Used (LRU) Cache component. The main use of this
component is to reduce wear on the flash at the expense of memory.

The LRU is a kernel list of nodes. Each node represents an FS object. Each
node also contains a kernel list of deltas. Each delta contains information
about an update or deletion from the user (which is what all of the VFS write
operations can be condensed to).

There's a pre-configured limit for both deltas per node and nodes in the LRU.

If the delta list in a node is full, and another is to be added, all the
existing deltas in the list are clubbed together and written to the flash
using the CTZ layer. The layer also automatically adds a log for this update.
When a node receives a delta, it is bumped from its current location in the
LRU to be at the front. This way, the last node in the LRU is always the
least used node.

If the node limit is reached in the LRU, and a new node is to be added to the
LRU, then the final node (which is also the least recently used node), is
popped from the LRU to make space for the new node. This popped node is then
written to the flash using the CTZ layer as well.

The LRU helps in clubbing updates to a single FS object and thus helps in
reducing the wear of the flash.

Journal Flush
-------------

The latest master node revision is the most useful out of the revisions. As
in CoW it's prudent to update the FS tree from bottom up, the root is the
last one to get updated in the case of a journal flush.

The logs are just location updates. So, when a journal flush occurs, it will
update the locations of all the children in the parent. This updates the
parent, and then this update goes through the same procedure as any other
update.

This is why it's best to start the flush operation when the journal is filled
up over a certain limit, instead of waiting it to be full. Why? Any log of
a parent makes any log of its children written **before** it useless, as the
updated location of the parent can be read to get the updated location of the
child till that point in the logs.

So, it will be best to move up the FS tree from bottom during update and
update the root last, since the root is the parent of every FS object.

Once the root is updated, all other journal logs become useless, and can be
erased. The root's log is not written in the first ``n`` blocks of the
journal, but written as a new master node entry in the master blocks.

Once the new root is written, the first ``n`` blocks can be erased, and
reallocated (due to the rules of wear levelling). The master blocks however
have some conditions for reallocation. This is called moving of the journal.

Every time the first ``n`` blocks are cleared, a new master node is added.
The only time a master block needs to be erased is when it becomes full.
Thus, if there are ``p`` pages in a block, the master blocks will be
moved along with the rest of the journal for every ``p`` journal flushes.

Before the new master node is written, none of the old pages should be erased
to allow rollback to the previous FS tree state. The moment the new master
node is updated, any block which has all of the pages in it ready for
deletion will be erased to make space.

FS Object Layer
---------------

This layer provides an abstraction for iterating, adding, deleting or
reading direntries.

This works with the LRU and the journal to get the latest data and thus the
user of this layer does not have to worry about these underlying mnemofs
components.

VFS Method Layer
----------------

VFS method layer contains methods exposed to the VFS. This layer works with
the FS Object layer for direntry related tasks, or the LRU for file level
read/write tasks.