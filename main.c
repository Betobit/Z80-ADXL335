#define PPI_BASE_ADDR  0x60
#define UART_BASE_ADDR 0x70

#include "smz80.h"

ISR_NMI() {
    /* Código de servicio de la interrupción NMI.*/
}

ISR_INT_38() {
    /* Código de servicio de la interrupción INT.*/
}

/**
    P r o t o t y p e s
**/
void configPPI();

void system_init() {
    /*Código para inicializar los dispositivos E/S del Z80 y las variables del sistema*/
    configPPI();
}

int main() {
    // Inicialización del sistema.
    system_init();
    // Ciclo infinito del programa.
    while(TRUE){
    	PPI_PORTA = 0xff;
    	delay_ms(2000);
        /* CÓDIGO AQUI*/


    	PPI_PORTA = 0x00;
    	delay_ms(2000);
    }
}

/**
    Config PPI.
    PORT A - Output
    PORT B - Output
    PORT C HIGH - Don't care
    PORT C LOW  - Don't care
**/
void configPPI() {
    // CONTROL WORD = 1001 0010
    PPI_CTRL = 0x92;
}
