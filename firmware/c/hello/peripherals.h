#include <stdint.h>

#ifndef PERIPHERALS_H
#define PERIPHERALS_H

#define IO_BASE_ADDR   0x400000
#define IO_LEDS        4
#define IO_UART_DAT    8
#define IO_UART_CTRL   16
#define IO_7SEG1_DATA  32
#define IO_7SEG2_DATA  64

#define IO_IN(port)       *(volatile uint32_t*)(IO_BASE_ADDR + port)
#define IO_OUT(port,val)  *(volatile uint32_t*)(IO_BASE_ADDR + port)=(val)


#endif /* PERIPHERALS_H */
