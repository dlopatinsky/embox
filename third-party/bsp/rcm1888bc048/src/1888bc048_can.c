/**
  ******************************************************************************
  * @file    1888bc048_can.c
  * @author  A. Maksimov, V. Syrtsov 
  * @version V1.0.1
  * @date    05.12.2019
  * @brief   This file provides firmware functions to manage the following 
  *          functionalities of the CAN peripheral:
  *           + Initialization and Configuration functions
  *           + CAN Read and Write functions
  *
  *  @verbatim
  *
    ===========================================================================
                         ##### How to use this driver #####
    ===========================================================================
      [..]
    @endverbatim
  ******************************************************************************
  * @attention
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "1888bc048_can.h"

/**
  * @brief      .
  * @param  CANx: where x can be (0 or 1) to select the CAN peripheral.
  * @retval TRT value.
  */
uint32_t *CAN_GetTRT(CAN_TypeDef *CANx){
return (uint32_t*)&(CANx->TRT);  
}

/**
  * @brief  Fills TRT with trt value.
  * @param  CAN_TRT_TypeDef: pointer to a CAN_TRT_TypeDef structure which will
  *         be initialized.
  * @retval None
  */

void CAN_SetTRT(CAN_TypeDef *CANx, CAN_TRT_TypeDef trt){
uint32_t temp;
  //  ...
  CANx->IMCR = IMCR_InitIn;
  while(CANx->IMCR==IMCR_InitIn){}
  //    ...
  CANx->Mode.TRCE = 1;
  temp = *(uint32_t*)&trt;
  CANx->TRT = trt;
  while(temp!=*CAN_GetTRT(CANx)){}
  //    ...  
  CANx->Mode.TRCE = 0;
  //       
  CANx->IMCR = IMCR_InitOut;
  while(CANx->IMCR==IMCR_InitOut){}
  while(!(CANx->Status.BusIdle)){}
}

/**
  * @brief  Writes data to the specified CAN data buffer.
  * @param  CANx:   .
  * @param  num:   .
  * @param  buf:    .
  * @retval  
  */
uint32_t CAN_WriteTxBuf(CAN_TypeDef *CANx, uint32_t num, uint32_t *buf){
int j;
  if((num<CAN_TQ_BUF_NUM) && (CANx->TQ[num].EFCnt)){
    for(j=0;j<CAN_DATA_BUF_LEN; j++) {
      CANx->DataTQ[num][0] = *buf++;
      dsb();
      isb();
    }  
    return CAN_STATUS_OK;
  }
  return CAN_STATUS_ERR;
}
/**
  * @brief      .
  * @param  CANx:   .
  * @retval     .
  */
uint32_t *CAN_ReadRxBuf(CAN_TypeDef *CANx){
static uint32_t out[CAN_DATA_BUF_LEN];
uint32_t j;
  for(j=0;j<CAN_DATA_BUF_LEN;j++){
    out[j] = CANx->DataEQ[0];
  }
return out;  
}

/**
  * @}
  */
