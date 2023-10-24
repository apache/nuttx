=========================
``brickmatch`` Brickmatch
=========================

Brickmatch is a kind puzzle game like a mix between tetris and Candy
Crush. It is a 6x6 matrix with blocks (cells) with different colors.

Your goal is to move the blocks of the board to unite three or
more with the same color.

Everytime that three of more blocks with the same color match that block
will blink and it will be removed from the board, leaving more space
for movements.

The game starts with only the border cells filled and you can move the
walls, floor and ceil in direction to the center of the board to make
the cells of same color to match.

Basic Test
----------

The best way to play brickmatch is using an APA102 RGB 16x16 matrix
and Gesture sensor APDS9960. There are some board examples already
done for this integration, all you need to do is connecting the APA102
matrix to the right SPI pins (look your board configuration) and the
APDS9960 to the I2C port (also connect its INT pin).

If you don't have an APA102 matrix you can also play it using an LCD
display and a digital joystick (DJOYSTICK) or the console input. 

Then you can configure and compile the game to play in your board,
i.e. for ESP32-Devkitc there is already an example using the APA102::


    $ ./tools/configure.sh esp32-devkitc:brickmatch
    $ make -j flash ESPTOOL_PORT=/dev/ttyUSB0
    $ minicom
    nsh> brick

That is it! Have fun!
