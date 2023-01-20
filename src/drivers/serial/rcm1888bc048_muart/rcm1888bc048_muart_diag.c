//--------------------------------------------------------------
// @author  V. Syrtsov
//--------------------------------------------------------------

#include <compiler.h>
#include <drivers/diag.h>
#include <framework/mod/options.h>

#include <1888bc048_uart.h>

#define RCM1888BC048_UARTx_BASE 		OPTION_GET(NUMBER, uart_base)
#define RCM1888BC048_UARTx 				((UART_TypeDef *)RCM1888BC048_UARTx_BASE)
#define RCM1888BC048_BAUD_RATE 			OPTION_GET(NUMBER, baud_rate)

void muart_diag_putc(const struct diag *diag, char ch) {
	while ( RCM1888BC048_UARTx->FIFO_STATE.TxNum );
	UART_Write(RCM1888BC048_UARTx, ch);
}

static int muart_diag_kbhit(const struct diag *diag) {
	return UART_GetRxNum(RCM1888BC048_UARTx);
}

static char muart_diag_getc(const struct diag *diag) {
	return (char)UART_ReadInputData(RCM1888BC048_UARTx);
}

static int muart_diag_init(const struct diag *diag) {
		
	UART_InitTypeDef UART_InitStruct;
	 
	UART_StructInit(&UART_InitStruct);
	UART_InitStruct.Div.BAUD_DIV = 50000000UL/RCM1888BC048_BAUD_RATE;
	UART_Init(RCM1888BC048_UARTx, &UART_InitStruct); 

	return 0;
}

DIAG_OPS_DEF(
		.init = muart_diag_init,
		.putc = muart_diag_putc,
		.getc = muart_diag_getc,
		.kbhit = muart_diag_kbhit,
);
