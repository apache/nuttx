General SD-card Operation (Bob Feretich)
=========================

SD-cards are not passive memory devices. They contain a microcontroller,
input/output buffers, and flash memory. Data being written to an SD-card
is received by the microcontroller, CRC checked, and then placed in an input
buffer. The size of these input buffers vary, but most modern (SDHC (4GB+) and
newer) cards have at least 16 KB of input buffers and are able to store the
contents of the entire input buffer into the flash memory in one "program"
operation. This "program" operation accounts for most of the delay of the
write operation.

When a Single Block Write (CMD24) is performed, a sector (512 bytes) of data
is transfered to the SD-card, the data is placed in an input buffer, an erased
area of flash memory is obtained/identified, and the data in the input buffer
is programmed into the flash memory.

When a Multiple Block Write (CMD25) is performed, one or more sectors
(512 bytes each) of data are transfered to the SD-card, the data is CRC
checked and placed in the input buffer, when the buffer is full, the contents
are programmed into the flash memory. During the programming operation,
the SD-card will hold-off the transfer of additional blocks from the mmcsd
driver by becoming "busy". Then when it can accept more data, the card drops
its busy indication and the transfer of data blocks continue. When the mmcsd
driver has sent the number of blocks that it was requested to send, the mmcsd
driver issues a Stop Transmission CMD12 to the SD-card. This command causes
the SD-card to program any data remaining in the input buffer into the
flash memory.

During a Multiple Block Write, obtaining erased flash memory areas may become
a bottleneck. (Erasing flash takes much longer than programming it.)
To eliminate this delay, the mmcsd driver issues a SET_WR_BLK_ERASE_COUNT
ACMD23 to notify the SD-card of the needed number of erased blocks in advance.

The mmcsd driver will automatically use Multiple Block Writes when mmcsd_write
is called with nsectors greater than one, unless CONFIG_MMCSD_MULTIBLOCK_DISABLE
is defined.

The SDIO block callback option (Bob Feretich)
==============================
The best SD-card write performance is achieved when Multiple Block Writes are
used to take advantage of the large input buffers in the SD-cards. By calling
mmcsd_write with a 16 KB buffer (32 sectors of data) I was able to consistently
achieve greater than 1.7 MB / second on a Class 10 SDHC SD-card. (stm32f7
microcontroller with the sdio clock at 16 MHz; time is averaged over writing
100 MB of data) Performance using Single Block Writes was less than
100 KB / second. Note that new cards had much higher initial performance, but
the performance degraded to (and seems to have stabilized at) this level after
writing several gigabytes to the card.

But even with the resources of a stm32f7 dealing with 16 KB DMA friendly
buffers is very difficult. (The stm32 DTCM memory region is 64 KB. That is
only four 16K buffers.) The designers of the SD-card architecture knew this
and made the SD-card interface flexible enough so that a microcontroller
with only a single 512-byte buffer can take advantage of Multiple Block Write
speeds.

The feature in the SD-card interface that makes this possible is the ability
for the interface to be paused between blocks of a Multiple Block Write.
(After a block of data is transferred to the SD-card, the buffer can be
refilled and its contents transferred again as needed. Or better yet, a double
buffering architecture can be used, so while one buffer is being transferred
to the SD-card, another buffer could be being filled.)
Unfortunately, the POSIX interfaces of the operating system and the FAT
File System do not facilitate buffers being refilled or swapped in the middle
of a write operation. But there is a way to do this and make both the
operating system and the FAT File System happy.

The POSIX write function is defined as...
ssize_t (*write)(FAR struct file *filep, FAR const char *buffer, size_t buflen);
This configuration option redefines the interpretation of <buffer> from a
pointer directly to the data buffer to a pointer to a structure that contains
the callback linkage to the caller's function that provides the address of the
next block to be transferred. The <buffer> itself is opaque to Nuttx. The most
Nuttx is permitted to do is check that <buffer> specifies a memory address
that is valid for data transfer.

But, why reinterpret write() when ioctl() is provided for these special
circumstances? We choose to provide a reinterpretation of write() because
file systems like fatfs do not deal with ioctl() forms of write, and this
reinterpretation permits Multiple Block Writes to utilize the file system
and therefore can generate media that can be read on other computers.

Nuttx passes a write() upon an opened SD-card file to the fatfs file system...
ssize_t fat_write(FAR struct file *filep, FAR const char *buffer, size_t buflen);
<buflen> is important for directory management and FAT cluster assignment, and
since it contains the correct length of data to be written, these functions
will perform as designed.

If <buflen> is an even multiple of 512, then the Nuttx implementation of fatfs
provides the option of data transfer directly from the user's buffer. When
this option is utilized, fatfs only checks that the address contained in
<buffer> specifies a valid DMA transfer region. So if the callback linkage
structure pointed to by <buffer> is located in DMA capable RAM (not
necessarily DTCM), then fatfs is happy and will pass the write to the
mmcsd driver...
ssize_t mmcsd_write(FAR struct inode *inode, FAR const unsigned char *buffer,
                           size_t startsector, unsigned int nsectors);

When CONFIG_SDIO_BLK_CALLBACK is defined, then the mmcsd driver's
mmcsd_writemultiple function will interpret <buffer> as a pointer to a
struct sdsector_callback_s...
   typedef FAR uint8_t *(*sdsector_callback_t)( size_t );
   struct sdsector_callback_s {
     sdsector_callback_t callback;
   };

Then when the mmcsd driver is ready to transmit a sector of data to the ,
SD-card it will call the provided callback function and the function is
expected to return the address of the next 512 bytes of data to be written.
If NULL is returned, then the mmcsd driver will issue a Stop Transmission
CMD12 to end the write. If the write is ended prematurely, then the blocks
already written will contain the new data, but blocks designated for erasure
may only be partially erased.

The below measurements were made on a stm32f722 (216 MHz, 16 MHz sdio clock)
using a Patriot SDHC 4 GB SD-card. (Several GBs of data were written to the
card before it was reformatted and used for these tests.

Sectors-per-Write Buffer_Size_Used  Average_Data_Rate  Longest_Delay Via_fatfs
        1             512 Bytes       50-75 KB/s*         490 ms        yes
        1             512 Bytes       50-75 KB/s*         490 ms        no
       32             512 Bytes        1750 KB/s          330 ms        yes
       32             512 Bytes        1850 KB/s          300 ms        no
       32             16K Bytes        1875 KB/s          310 ms        yes
       32             16K Bytes        1925 KB/s          300 ms        no
* The range shown depicts run to run variations that exceeded any noticeable
  performance difference between these two rows. The single sector write tests
  wrote only 10MB of data per run. Other tests wrote 100 MB of data per run.
Other measurements showed +/- 10% run to run variations. Note that new SD-card
performance was much faster than these numbers, but the performance drops off
quickly as the card is written.

Usage notes:

* The architecture of the SDIO adapter in the MCU must permit configuring
  DMA after the CMD25 is issued.
* When using Multiple Block Writes through the file system, a Multiple Block
  Write must not span a file system cluster boundary. (Cluster's in a file
  may not be contiguous.)
