# STM32F0-Timer-WS2812

This repo contains an example and some simple functions for driving WS2812 LEDs using DMA and a general purpose timer on an STM32F0 part. The motivation behind this project was to be able to generate the accurate timings required for these LEDs, without having to stall the processor with NOP instructions or disable interrupts.

With the DMA peripheral automatically copying the required timings into the timer's capture compare register, there should be a fair amount of time for the processor to do other things. In theory setting the size of the DMA buffer larger should reduce time spent jumping into ISRs, and therefore give more time for the CPU to do other stuff, obviously this is a trade off vs the amount of memory used.
