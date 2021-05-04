MEMORY
{
    FLASH      (rx)  : ORIGIN = 0x08000000, LENGTH = 128K
    STACK      (rwx) : ORIGIN = 0x20000000, LENGTH = 8K
    SRAM       (rwx) : ORIGIN = 0x20002000, LENGTH = 8K
}

SECTIONS
{
    .text :
    {
        LONG(ORIGIN(STACK) + LENGTH(STACK));
        KEEP(*(.handlers))
        *(.text*)
        *(.rodata*)
        _etext = .;
    } > FLASH

    .bss (NOLOAD) :
    {
        _sbss = .;
        *(.bss*)
        *(COMMON)
        _ebss = .;
    } > SRAM

    .data :
    {
        _sdata = .;
        *(.data*);
        _edata = .;
    } > SRAM AT >FLASH
}
