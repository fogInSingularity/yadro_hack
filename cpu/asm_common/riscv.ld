OUTPUT_ARCH("riscv")

ENTRY(_start)

SECTIONS
{
    . = 0x10000;

    .text :
    {
        loader.o(.text)
        *(.text .text.*)
        *(.text.startup .text.startup.*)
    }
}
