# STM32F0-WS2812-Dev

This repo builds on top of a previous repo:
https://github.com/Cpt-Quantum/STM32F0-Timer-WS2812

Whereas that repo stopped at just getting the drive signal working for WS2812 (addressable) LEDs,
this repository goes further, adding in support for more peripherals. The main addition is USART and
ADC APIs. The pre-existing functions in the old repo have also been improved.

The code in this repo is designed to work specifically on the custom dev board I built in:
https://github.com/Cpt-Quantum/uart-to-ws2812

The main aim for this codebase is to add in code support for all required SoC peripherals on the
STM32F030 part, in order to build a simple WS2812 driver with different lighting effects. The
primary effect being a VU meter, but this will be targeting a much longer LED strip than is present
on this board. The functionality for this effect (as well as a few others) is already in place.

This codebase should work with minor adaptation for any board based around the STM32F030 chips.
