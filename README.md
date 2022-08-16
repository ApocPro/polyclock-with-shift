# Polyrhythm Clock Generator

Clock Dividers have always been a staple of modular synthesis.  They have added complexity and nuance to many of the patches I have built, allowing for long delayed triggers to keep new things happening.
These Clock Dividers are great, but the approach has always seemed to me to be mathematical rather than musical.  This is my attempt to make a clock divider that is based more on musical principles than mathematical ones (and thereby entailing even more math :). 
The principle is based on Polyrhythms such as 3:4 or 5:3.  This is achieved by taking an input clock, setting what time signature that input clock is in, so that we can find the downbeat, and then subdividing that downbeat to match our new desired time signature.

I was in the process of building and coding, and having latency issues, when Look Mum No Computer published his [video about his pendulum clock](https://www.youtube.com/watch?v=DT5CafrZyDw).  I had already been looking at ginky syntese grain's code. The patch that RohanM. did for LMNC solved a number of the problems that I was having.  90% of this code is based on their work and I simply added a few switches and changed the math to accomplish what I was trying to do.  All of the `class` building and `interrupts` are entirely their work and I am highly appreciative of them sharing their work:

## Clock multiplier / divider / beatshift

The [original code by a773](https://github.com/attejensen/a773_grains) (atte.dk) (released under
the GPL licence) provided alternate firmware for the ginky synthese grains eurorack module.

Extended by [RohanM.](https://github.com/RohanM/clock-with-shift) to provide a general purpose clock multipler / divider / beatshift
code for use as a synthesiser module.


## Hardware

This module runs on an Arduino (tested on an Arduino Nano). It expects four potentiometers as controls
(pins defined by `MULT_POT`, `DIV_POT`, `MODE_POT` and `BEATSHIFT_POT`). It takes a clock as input
on the `CLOCK_IN` pin and has two trigger outputs - `SHIFTED_OUT` and `UNSHIFTED_OUT` (the latter
identical but without the beatshift applied).


## Operation

The module can operate in two modes:

- Simple mode: Multiplies or divides by factors of two (1, 2, 4, 8, 16, ...)
- Complex mode: Multiplies or divides by prime numbers (1, 3, 5, 7, 11, ...)

The mode can be selected by moving the mode knob to the left or right side. A range of output
tempos can be produced by combining multiplication and division, especially by using complex factors.
The beatshift knob can be used to delay the trigger from the beatshifted output by up to one beat.


## Changelog

- 24-10-2021 Reworked by Rohan Mitchell for easier multi/div changes between beats
- 16-10-2021 Further changes to allow for longer gaps between incoming beats and logic to handle multi/div changes between beats for Look Mum No Computer
- 11-9-2021 Adapted by Jesse Stevens of artist duo Cake Industries for Look Mum No Computer offbeat shift needs
