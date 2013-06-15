INTRO
=====

FantaLCD is an LCD text terminal application for the AVR. 

![](http://i.imgur.com/RnW6xKM.jpg)

It is a standalone LCD controller that is designed to augment most of the LCD control functions and make them accessible with escape sequences in the text itself. This greatly reduces overhead on your application for controlling the display. 

- You don't have to care about LCD display timing
- You don't have to generate clock signal for the LCD
- You don't have to make distinctions between data and control commands. 

- You just write "\cHello World!" to the serial input line and the display is cleared and your text is displayed on the display. 

PINOUT
======

This is the final pinout of a programmed chip: 

![](http://i.imgur.com/cBBwIoC.jpg)

COMPILING
========

We all know linux is the shit, so get a copy of "fanta-tools" for ubuntu: https://github.com/fantachip/fanta-tools and compile the toolchain for the avr yourself. 
