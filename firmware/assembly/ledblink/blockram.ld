OUTPUT_ARCH( "riscv" )
ENTRY(start)

MEMORY
{
   BLOCKRAM (RWX) : ORIGIN = 0x0000, LENGTH = 0x1800  /* 6kB RAM */
}
SECTIONS
{
    all :
    {
	. = ALIGN(4);
	ledblink.o (.text)
        *(.*)
    } >BLOCKRAM
}
