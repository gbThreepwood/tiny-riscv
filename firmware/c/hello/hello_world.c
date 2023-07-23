#include "peripherals.h"
#include "printf.h"

extern putstring(char *str);

int main(void) {

    putstring("RISC-V demo\r\n");
    putstring("putstring funksjonen frå assembly kan også kallast opp frå C.\r\n");

    char c[2] = {0};
    for(uint8_t i = 0; i < 10; i++){
        putstring("Talet er:");

        c[0] = i + 48;
        putstring(c);

        putstring("\r\n");
    }

    putstring("Det samme gjeld delay\r\n");


    //for(uint8_t i = 0; i < 10; i++){
    //    printf("Hei, verden %i", i);
    //}


    return 0;
}
