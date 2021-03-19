#ifndef _BSP_H_
#define _BSP_H_

// Initializations for protocols
void BSP_init(void);
void ADC_init(void);
void GPIO_init(void);
void UART_init(void);
void DMA_init(void);
void WTimer_init(void);

// DMA Control Setup
void CTLTable_init(void);
void DMA_start(void);

// Delay loop
void Delay(unsigned long counter);

// UART0
char UART0_Rx(void);
void UART0_printChar(unsigned char data);
void UART0_printString(char * string);

#endif // _BSP_H_
