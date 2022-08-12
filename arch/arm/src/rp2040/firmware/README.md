CYW43xx WiFi SoC firmware
=========================

This directory contains firmware patch blobs that need to be downloaded on to the
CYW43xx SoC in order for it to function correctly.

The firmware is padded to 512 bytes and then the CLM appended to that, to create the
combined binary file.

For example:

    $ cp 43439A0.bin 43439A0_padded.bin
    $ dd if=/dev/zero of=43439A0_padded.bin bs=1 count=1 seek=$(( ($(stat -c %s 43439A0.bin) / 512) * 512 + 512 - 1))
    $ cat 43439A0_padded.bin 43439A0.clm_blob > 43439A0-7.95.49.00.combined

Note that the number of dots in the filename is significant, due to the way symbols
are renamed by the linker when it is included in a compiled application.

When updating this firmware, check and update the values of `CYW43_FW_LEN` and
`CYW43_CLM_LEN`.  These should be the original, unpadded size of the two binaries
in bytes.  The array size should be the size of the combined file.
