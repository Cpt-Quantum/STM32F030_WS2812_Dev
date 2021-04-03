/* Define lable for program entry point */
ENTRY(reset_handler)

/* Define the end point of the stack */
_estack = 0x020001000;

/* Define the minimum size for dynamic memory allocation */
_Min_Leftover_RAM = 0x400;

MEMORY
{
	FLASH (rx)	: ORIGIN = 0x08000000,	LENGTH = 16K
	RAM (rxw)	: ORIGIN = 0x20000000,	LENGTH = 4K
}

SECTIONS
{
	/* Vector table region */
	/* This needs to be placed at the start of flash */
	.vector_table :
	{
		. = ALIGN(4);
		KEEP(*(.vector_table))
		. = ALIGN(4);
	} >FLASH

	/* Text region containing the main program code */
	.text :
	{
		. = ALIGN(4);
		*(.text)
		*(.text*)
		. = ALIGN(4);
	} >FLASH

	/* Read only data section */
	.rodata :
	{
		. = ALIGN(4);
		*(.rodata)
		*(.rodata*)
		. = ALIGN(4);
	} >FLASH

	/* Data section for variables et al */
	_sidata = .;
	.data : AT(_sidata)
	{
		. = ALIGN(4);
		_sdata = .;	/* Mark the start of the data section */
		*(.data)
		*(.data*)
		. = ALIGN(4);
		_edata = .;	/* Mark the end of the data section */
	} >RAM
	
	/* Zero initialised data section */
	.bss :
	{
		. = ALIGN(4);
		_sbss = .;	/* Mark the start of the bss section */
		*(.bss)
		*(.bss*)
		*(COMMON)
		. = ALIGN(4);
		_ebss = .;	/* Mark the end of the bss section */
	} >RAM

	/* Section for heap/stack allocation */
	.dynamic_allocations :
	{
		. = ALIGN(4);
		_ssystem_ram = .;	/* Mark the start of malloc region */
		. = . + _Min_Leftover_RAM;
		. = ALIGN(4);
		_esystem_ram = .;	/* Mark the end of malloc region */
	}
}

