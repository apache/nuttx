===============
Host Side Tools
===============

``bitmap_converter.py`` NxWidgets
---------------------------------

This script converts from any image type supported by Python imaging library to
the RLE-encoded format used by NxWidgets.

RLE (Run Length Encoding) is a very simply encoding that compress quite well
with certain kinds of images: Images that that have many pixels of the same
color adjacent on a row (like simple graphics). It does not work well with
photographic images.

But even simple graphics may not encode compactly if, for example, they have
been resized. Resizing an image can create hundreds of unique colors that may
differ by only a bit or two in the RGB representation. This "color smear" is the
result of pixel interpolation (and might be eliminated if your graphics software
supports resizing via pixel replication instead of interpolation).

When a simple graphics image does not encode well, the symptom is that the
resulting RLE data structures are quite large. The palette structure, in
particular, may have hundreds of colors in it. There is a way to fix the graphic
image in this case. Here is what I do (in fact, I do this on all images prior to
conversion just to be certain):

- Open the original image in GIMP.
- Select the option to select the number of colors in the image.
- Pick the smallest number of colors that will represent the image faithfully.
  For most simple graphic images this might be as few as 6 or 8 colors.
- Save the image as PNG or other lossless format (NOT jpeg).
- Then generate the image.
