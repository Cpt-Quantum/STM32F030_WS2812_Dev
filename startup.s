/* Chip specific attributes */
.syntax unified
.cpu cortex-m0
.fpu softvfp
.thumb

/* Global memory locations */
.global vtable
.global reset_handler

/* Default handler to handle unexpected interrupts */
.section .text.default_interrupt_handler,"ax",%progbits
default_interrupt_handler:
	default_interrupt_loop:
		B	default_interrupt_loop
.size default_interrupt_handler, .-default_interrupt_handler

/* Reset handler */
.type reset_handler, %function
reset_handler:
	/* Set the stack pointer to the end of the stack */
	LDR		r0,		=_estack
	MOV		sp,		r0

	/* Copy data from flash to RAM data init section */
	MOVS	r0,		#0
	/* Load the start and end of the data region into r1 & r2 respectively */
	LDR		r1,		=_sdata
	LDR		r2,		=_edata
	LDR		r3,		=_sidata
	B		copy_sidata_loop

	copy_sidata:
	/* Offset the data init section by the copy progress */
	LDR		r4,		[r3, r0]
	/* Copy the current word into data and increment */
	STR		r4,		[r1, r0]
	ADDS	r0,		r0,		#4

	copy_sidata_loop:
	/* Unless the whole data section has been copied already, copy the next */
	/* word from sidata into data                                           */
	ADDS	r4,		r0,		r1
	CMP		r4,		r2
	BCC		copy_sidata

	/* Once the data section has been finished being copied to RAM, move on */
	/* initialising the BSS sections with 0s.                               */
	MOVS	r0,		#0
	LDR		r1,		=_sbss
	LDR		r2,		=_ebss
	B		reset_bss_loop

	reset_bss:
	/* Store a 0 and increment by a full word (4 bytes) */
	STR		r0,		[r1]
	ADDS	r1,		r1,		#4

	reset_bss_loop:
	CMP		r1,		r2
	BCC		reset_bss

	/* Now branch to main */
	BL		main

.size reset_handler, .-reset_handler

/* Note that these vector table entries and weak symbol names are copied from */
/* STMicro's own example startup file.                                        */
.type vtable, %object
.section .vector_table,"a",%progbits
vtable:
	.word _estack
	.word reset_handler
	.word NMI_Handler
	.word HardFault_Handler
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word SVC_Handler
	.word 0
	.word 0
	.word PendSV_Handler
	.word SysTick_Handler
	.word WWDG_IRQHandler                   /* Window WatchDog              */
	.word 0                                 /* Reserved                     */
	.word RTC_IRQHandler                    /* RTC through the EXTI line    */
	.word FLASH_IRQHandler                  /* FLASH                        */
	.word RCC_IRQHandler                    /* RCC                          */
	.word EXTI0_1_IRQHandler                /* EXTI Line 0 and 1            */
	.word EXTI2_3_IRQHandler                /* EXTI Line 2 and 3            */
	.word EXTI4_15_IRQHandler               /* EXTI Line 4 to 15            */
	.word 0                                 /* Reserved                     */
	.word DMA1_Channel1_IRQHandler          /* DMA1 Channel 1               */
	.word DMA1_Channel2_3_IRQHandler        /* DMA1 Channel 2 and Channel 3 */
	.word DMA1_Channel4_5_IRQHandler        /* DMA1 Channel 4 and Channel 5 */
	.word ADC1_IRQHandler                   /* ADC1                         */
	.word TIM1_BRK_UP_TRG_COM_IRQHandler    /* TIM1 Break, Update, Trigger and Commutation */
	.word TIM1_CC_IRQHandler                /* TIM1 Capture Compare         */
	.word 0                                 /* Reserved                     */
	.word TIM3_IRQHandler                   /* TIM3                         */
	.word 0                                 /* Reserved                     */
	.word 0                                 /* Reserved                     */
	.word TIM14_IRQHandler                  /* TIM14                        */
	.word 0                                 /* Reserved                     */
	.word TIM16_IRQHandler                  /* TIM16                        */
	.word TIM17_IRQHandler                  /* TIM17                        */
	.word I2C1_IRQHandler                   /* I2C1                         */
	.word 0                                 /* Reserved                     */
	.word SPI1_IRQHandler                   /* SPI1                         */
	.word 0                                 /* Reserved                     */
	.word USART1_IRQHandler                 /* USART1                       */
	.word 0                                 /* Reserved                     */
	.word 0                                 /* Reserved                     */
	.word 0                                 /* Reserved                     */
	.word 0                                 /* Reserved                     */

	/* Define weak symbols to assign each exception handler to the default handler */
	/* These symbols will be overwritten if a custom exception handler function is */
	/* written for a specific interrupt.                                           */
	.weak      NMI_Handler
	.thumb_set NMI_Handler,default_interrupt_handler

	.weak      HardFault_Handler
	.thumb_set HardFault_Handler,default_interrupt_handler

	.weak      SVC_Handler
	.thumb_set SVC_Handler,default_interrupt_handler

	.weak      PendSV_Handler
	.thumb_set PendSV_Handler,default_interrupt_handler

	.weak      SysTick_Handler
	.thumb_set SysTick_Handler,default_interrupt_handler

	.weak      WWDG_IRQHandler
	.thumb_set WWDG_IRQHandler,default_interrupt_handler

	.weak      RTC_IRQHandler
	.thumb_set RTC_IRQHandler,default_interrupt_handler

	.weak      FLASH_IRQHandler
	.thumb_set FLASH_IRQHandler,default_interrupt_handler

	.weak      RCC_IRQHandler
	.thumb_set RCC_IRQHandler,default_interrupt_handler

	.weak      EXTI0_1_IRQHandler
	.thumb_set EXTI0_1_IRQHandler,default_interrupt_handler

	.weak      EXTI2_3_IRQHandler
	.thumb_set EXTI2_3_IRQHandler,default_interrupt_handler

	.weak      EXTI4_15_IRQHandler
	.thumb_set EXTI4_15_IRQHandler,default_interrupt_handler

	.weak      DMA1_Channel1_IRQHandler
	.thumb_set DMA1_Channel1_IRQHandler,default_interrupt_handler

	.weak      DMA1_Channel2_3_IRQHandler
	.thumb_set DMA1_Channel2_3_IRQHandler,default_interrupt_handler

	.weak      DMA1_Channel4_5_IRQHandler
	.thumb_set DMA1_Channel4_5_IRQHandler,default_interrupt_handler

	.weak      ADC1_IRQHandler
	.thumb_set ADC1_IRQHandler,default_interrupt_handler

	.weak      TIM1_BRK_UP_TRG_COM_IRQHandler
	.thumb_set TIM1_BRK_UP_TRG_COM_IRQHandler,default_interrupt_handler

	.weak      TIM1_CC_IRQHandler
	.thumb_set TIM1_CC_IRQHandler,default_interrupt_handler

	.weak      TIM3_IRQHandler
	.thumb_set TIM3_IRQHandler,default_interrupt_handler

	.weak      TIM14_IRQHandler
	.thumb_set TIM14_IRQHandler,default_interrupt_handler

	.weak      TIM16_IRQHandler
	.thumb_set TIM16_IRQHandler,default_interrupt_handler

	.weak      TIM17_IRQHandler
	.thumb_set TIM17_IRQHandler,default_interrupt_handler

	.weak      I2C1_IRQHandler
	.thumb_set I2C1_IRQHandler,default_interrupt_handler

	.weak      SPI1_IRQHandler
	.thumb_set SPI1_IRQHandler,default_interrupt_handler

	.weak      USART1_IRQHandler
	.thumb_set USART1_IRQHandler,default_interrupt_handler
.size vtable, .-vtable
