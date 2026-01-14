=========================
``match4`` Match4
=========================

Match4 is a classic puzzle game inspired by Connect Four and match-based logic games.
The game is played on a matrix (e.g., 6x6, 8x8) where colored blocks fall into columns
and the goal is to align four identical blocks in a row—vertically, horizontally, or diagonally.

The objective is to match four blocks of the same color row—vertically, horizontally, or diagonally.
As the game progresses, blocks continue to fall, and the challenge is to manage the board to avoid it filling up.

The game ends if there are no available moves left (e.g., the board is full and no matches are possible) or one player
achieve to match 4 blocks.

Basic Test
----------

To play game, you need to have a led matrix (e.g. ws2812) for output
and three gpio pins to input.

Alternatively, you can give inputs using serial console by changing settings.

Then you can configure and compile the game to play in your board,
i.e. for ESP32-Devkitc there is already an example::


    $ ./tools/configure.sh esp32-devkitc:match4
    $ make -j flash ESPTOOL_PORT=/dev/ttyUSB0
    $ minicom
    nsh> match

