//--------------------------------------------------------------
// @author  V. Syrtsov
//--------------------------------------------------------------

#include <compiler.h>
#include <drivers/diag.h>
#include <framework/mod/options.h>

#include "muart.h"

#define RCM1888BC048_UARTx_BASE 		OPTION_GET(NUMBER, uart_base)
#define RCM1888BC048_UARTx 				((UART_TypeDef *)RCM1888BC048_UARTx_BASE)
#define RCM1888BC048_BAUD_RATE 			OPTION_GET(NUMBER, baud_rate)





/********************/
 /**
  * @brief  Reads the specified input port pin.
  * @param  UARTx: where x can be (0 or 1) to select the UART peripheral.
  * @retval The input port pin value.
  */
static uint32_t UART_ReadInputData(UART_TypeDef* UARTx){
return *((uint32_t*)&(UARTx->RxD));
}

/**
  * @brief  Fills each UART_InitStruct member with its default value.
  * @param  UART_InitStruct : pointer to a UART_InitTypeDef structure which will
  *         be initialized.
  * @retval None
  */
static void UART_StructInit(UART_InitTypeDef* UART_InitStruct){
/* Reset UART init structure parameters values */
  UART_InitStruct->Div.N_DIV = 0;
  UART_InitStruct->Div.BAUD_DIV  = (uint32_t)(50e6)/115200;
  UART_InitStruct->Ctrl = (UART_CTRL_TypeDef){0};

  UART_InitStruct->Ctrl.Enable = 1;
  UART_InitStruct->Ctrl.APB_FIFO = 1;
  UART_InitStruct->Ctrl.POL = 1;
  UART_InitStruct->Ctrl.StopBits = UART_StopBits_1;
  UART_InitStruct->Ctrl.Length = UART_Length_8;
}

/**
  * @brief  Initializes the UARTx peripheral according to the specified
  *         parameters in the UART_InitStruct.
  * @param  UARTx: where x can be (0..1) to select the UART peripheral.
  * @param  UART_InitStruct: pointer to a UART_InitTypeDef structure that
  *         contains the configuration information for the specified UART peripheral.
  * @retval None
  */
static void UART_Init(UART_TypeDef* UARTx, UART_InitTypeDef* UART_InitStruct){
  UARTx->BDIV = UART_InitStruct->Div;
  UARTx->CTRL = UART_InitStruct->Ctrl;
}

  /**
  * @brief  Writes data to the specified UART data port.
  * @param  UARTx: where x can be (0..1) to select the UART peripheral.
  * @param  PortVal: specifies the value to be written to the port output data register.
  * @retval None
  */
static void UART_Write(UART_TypeDef* UARTx, uint8_t PortVal){
	UARTx->TxD = PortVal;
}
#if 0
/**
 * @brief  Writes data to the specified UART data port with FIFO.
 * @param  UARTx: where x can be (0..1) to select the UART peripheral.
 * @param  data: specifies the data vector to be written to the port output data register.
 * @param  num: specifies the number of bytes to be written.
 * @retval None
 */
static void UART_SendData(UART_TypeDef *port, uint8_t *data, uint16_t num) {
	while (num--) {
		port->TxD = *data++;
	}
}
#endif
/**
  * @brief     .
  * @param  UARTx: where x can be (0..1) to select the UART peripheral.
  * @retval   .
  */
static uint32_t UART_GetRxNum(UART_TypeDef* UARTx){return UARTx->FIFO_STATE.RxNum;}
#if 0
/**
  * @brief       FIFO  .
  * @param  UARTx: where x can be (0..1) to select the UART peripheral.
  * @param  data:
  * @retval   .
  */
static uint32_t UART_ReadData(UART_TypeDef *UARTx, uint8_t *data,
		uint16_t limit) {
	uint32_t out = UARTx->FIFO_STATE.RxNum, j;
	for (j = 0; (j < out) && (j < limit); j++)
		data[j] = UARTx->RxD & 0xFF;
	return out;
}
#endif
 /*******************/
static void muart_diag_putc(const struct diag *diag, char ch) {
	while ( RCM1888BC048_UARTx->FIFO_STATE.TxNum );
	UART_Write(RCM1888BC048_UARTx, ch);
}

static int muart_diag_kbhit(const struct diag *diag) {
	return UART_GetRxNum(RCM1888BC048_UARTx);
}

static char muart_diag_getc(const struct diag *diag) {
	return (char)(UART_ReadInputData(RCM1888BC048_UARTx) & 0xFF);
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
