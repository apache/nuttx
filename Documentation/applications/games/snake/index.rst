=========================
``snake`` Snake
=========================

Snake is a classic game, especially popular on old phones. The game is played on a
matrix (e.g., 6x6, 8x8) with blocks (cells) of different colors to represent
the snake and food objects.

The goal of the game is to eat food as much as possible without running into yourself.
After eating food, the snake's tail increases by one until the size of the snake
equals the size of the game board.

The game starts with a one-length snake, and the player can move through the board's edges.
When the snake reaches the edge, it reappears on the opposite side.

Basic Test
----------

To play game, you need to have a led matrix (e.g. ws2812) for output
and four gpio pins to input.

Alternatively, you can give inputs using serial console by changing settings.

Then you can configure and compile the game to play in your board,
i.e. for ESP32-Devkitc there is already an example::


    $ ./tools/configure.sh esp32-devkitc:snake
    $ make -j flash ESPTOOL_PORT=/dev/ttyUSB0
    $ minicom
    nsh> snake

