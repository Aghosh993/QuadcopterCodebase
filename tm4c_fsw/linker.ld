/* Linker script for TM4C123GH6PM */
/* Taken from https://github.com/spirilis/tiva1202/blob/master/TM4C123GH6PM.ld */

MEMORY
{
    FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 0x00040000
    SRAM (rwx) : ORIGIN = 0x20000000, LENGTH = 0x00008000
}

_stack_size = 10K;

SECTIONS
{
    .text :
    {
        _text = .;
        KEEP(*(.isr_vector))
        *(.text .text* .gnu.linkonce.t.*)
        *(.rodata .rodata* .gnu.linkonce.r.*)

        /* C++ initializers et al */
        . = ALIGN(4);
        KEEP(*(.init))
        . = ALIGN(4);
        __preinit_array_start = .;
        KEEP (*(SORT(.preinit_array*)))
        KEEP (*(.preinit_array))
        __preinit_array_end = .;
        . = ALIGN(4);
        __init_array_start = .;
        KEEP (*(SORT(.init_array*)))
        KEEP (*(.init_array))
        __init_array_end = .;
        . = ALIGN(4);
        KEEP(*(.fini))
        . = ALIGN(4);
        __fini_array_start = .;
        KEEP(*(.fini_array))
        KEEP(*(SORT(.fini_array*)))
        __fini_array_end = .;

        . = ALIGN(4);
        _etext = .;
    } > FLASH

    .data :
    {
        _data = .;
        *(vtable)
        *(.data .data* .gnu.linkonce.d.*)
        *(.fastcode)
        _edata = .;
    } > SRAM AT > FLASH

    .ARM.extab   : { *(.ARM.extab* .gnu.linkonce.armextab.*) } > FLASH
    __exidx_start = .;
    .ARM.exidx   : { *(.ARM.exidx* .gnu.linkonce.armexidx.*) } > FLASH
    __exidx_end = .;

    .bss (NOLOAD):
    {
        _bss = .;
        *(.bss*)
        *(COMMON)
        _ebss = .;
    } > SRAM

    _heap_bottom = ALIGN(8);
    _heap_top = ORIGIN(SRAM) + LENGTH(SRAM) - _stack_size;
    _stack_bottom = ALIGN(8);
    _stack_top = ORIGIN(SRAM) + LENGTH(SRAM);
}