
#include <avr/io.h>

#ifndef UART_H_
#define UART_H_


	#define UART_BAUD 115200		// tu definiujemy interesuj�c� nas pr�dko��
	#define __UBRR ((F_CPU+UART_BAUD*8UL) / (16UL*UART_BAUD)-1)  // obliczamy UBRR dla U2X=0
/*
	// definicje na potrzeby RS485
	#define UART_DE_PORT PORTD
	#define UART_DE_DIR DDRD
	#define UART_DE_BIT (1<<PD2)

	#define UART_DE_ODBIERANIE  UART_DE_PORT |= UART_DE_BIT
	#define UART_DE_NADAWANIE  UART_DE_PORT &= ~UART_DE_BIT
*/

	#define UART_RX_BUF_SIZE 128 // definiujemy bufor o rozmiarze 32 bajt�w
	// definiujemy mask� dla naszego bufora
	#define UART_RX_BUF_MASK ( UART_RX_BUF_SIZE - 1)

	#define UART_TX_BUF_SIZE 256 // definiujemy bufor o rozmiarze 16 bajt�w
	// definiujemy mask� dla naszego bufora
	#define UART_TX_BUF_MASK ( UART_TX_BUF_SIZE - 1)




	// deklaracje funkcji publicznych

	void USART_Init( uint16_t baud );

	char uart_getc(void);
	void uart_putc( char data );
	void uart_puts(char *s);
	void uart_putint(int value, int radix);

#endif /* MKUART_H_ */
