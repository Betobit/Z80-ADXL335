#define PPI_BASE_ADDR  0x60
#define UART_BASE_ADDR 0x70

#include "smz80.h"

ISR_NMI() {
}

ISR_INT_38() {
}

__sfr __at 0x60 PORTA;
__sfr __at 0x61 PORTB;
__sfr __at 0x62 PORTC;

/**
    P r o t o t y p e s
**/
void configPPI();
void system_init();

int main() {
    unsigned char c = 0;
    system_init();

    while(TRUE) {
        PORTC = 0x00;
        URTHR = PORTA;
        delay_ms(25);
        // leer PUERTO
        //URTHR = c > 254 ? c = 0 : c++;
        PORTC = 0x01;
        URTHR = PORTA;
        delay_ms(25);
        //uart_write(c > 254 ? c = 0 : c++);
    }
}

/**
    Init I/O ports and system
**/
void system_init() {
    uart_cfg_t uartConfig;
    uartConfig.baudrate = UART_BAUDRATE_9600;
    uartConfig.stop_bits = UART_STOP_BITS_1;
    uartConfig.parity = UART_PARITY_NONE;
    uartConfig.word_length = UART_WORD_LENGTH_8;
    uartConfig.interrupt = UART_INTERRUPT_NONE;
    uart_init(&uartConfig);
    configPPI();
}

/**
    Config PPI.
    PORT A - Input
    PORT B - Input
    PORT C HIGH - Don't care
    PORT C LOW  - Output
**/
void configPPI() {
    // CONTROL WORD = 1001 0010
    PPI_CTRL = 0x92;
}
