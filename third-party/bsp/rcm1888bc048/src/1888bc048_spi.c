/**
  ******************************************************************************
  * @file    1888bc048_spi.c
  * @author  A. Maksimov, V. Syrtsov
  * @version V1.0.1
  * @date    
  * @brief   This file provides firmware functions to manage the following 
  *          functionalities of the SPI peripheral:
  *           .......
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
#include "1888bc048_spi.h"

/**
  * @brief  Resets the SPIx peripheral registers to their default reset values.
  * @param  SPIx: Select the SPI peripheral.
  *         This parameter can be one of the following values: SPI0, SPI1.
  * @retval None
  */
void SPI_DeInit(SPI_TypeDef* SPIx)
{
  /* Check the parameters */
  assert_param(IS_SPI_ALL_PERIPH(SPIx));

  SPIx->SOFTRST = 1;
  while ( SPIx->SOFTRST != 0 );
}

/**
  * @brief  Initializes the SPIx peripheral according to the specified
  *         parameters in the SPI_InitStruct.
  * @param  SPIx: Select the SPI peripheral.
  *         This parameter can be one of the following values:
  *         SPI0, SPI1.
  * @param  SPI_InitStruct: pointer to a SPI_InitTypeDef structure
  *         that contains the configuration information for the specified SPI peripheral.
  * @retval None
  */
void SPI_Init(SPI_TypeDef* SPIx, const SPI_InitTypeDef* SPI_InitStruct)
{
  /* Check the parameters */
  assert_param(IS_SPI_ALL_PERIPH(SPIx));
  assert_param(IS_SPI_SPEED_FACTOR(SPI_InitStruct->SPI_SCR));
  assert_param(IS_SPI_SPEED_DIVIDER(SPI_InitStruct->SPI_CPSDVSR));


  SPIx->CR0.DSS = SPI_InitStruct->SPI_WordLength;
  SPIx->CR0.FRF = SPI_InitStruct->SPI_FRF;
  SPIx->CR0.SckPol = SPI_InitStruct->SPI_SPO;
  SPIx->CR0.SckPhase = SPI_InitStruct->SPI_SPH;
  SPIx->CR0.SCR = SPI_InitStruct->SPI_SCR;
  SPIx->CPSR.DIV = SPI_InitStruct->SPI_CPSDVSR;
}

/**
  * @brief  Fills each SPI_InitStruct member with its default value.
  * @param  SPI_InitStruct: pointer to a SPI_InitTypeDef structure
  *         that is to be initialized.
  * @retval None
  */
void SPI_StructInit(SPI_InitTypeDef* SPI_InitStruct)
{
  SPI_InitStruct->SPI_WordLength = DSS_8_Bits;
  SPI_InitStruct->SPI_SPO = SPO_Low;
  SPI_InitStruct->SPI_SPH = SPH_1Edge;
  SPI_InitStruct->SPI_FRF = FRF_SPI_Motorola;
  SPI_InitStruct->SPI_CPSDVSR = 2;
  SPI_InitStruct->SPI_SCR = 0;
}

/**
  * @brief  Enables or disables the specified SPI peripheral.
  * @param  SPIx: Select the SPI peripheral.
  *         This parameter can be one of the following values:
  *         SPI0, SPI1.
  * @param  NewState: new state of the SPIx peripheral.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void SPI_Cmd(SPI_TypeDef* SPIx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_SPI_ALL_PERIPH(SPIx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if ( NewState != DISABLE ) {
    SPIx->CR1.Enable = 1;
  } else {
    SPIx->CR1.Enable = 0;
  }
}

/**
  * @brief  Transmits single data through the SPIx peripheral.
  * @param  SPIx: Select the SPI peripheral.
  *         This parameter can be one of the following values:
  *         SPI0, SPI1.
  * @param  Data: the data to transmit.
  * @retval None
  */
void SPI_SendData(SPI_TypeDef* SPIx, uint16_t Data)
{
  /* Check the parameters */
  assert_param(IS_SSP_ALL_PERIPH(SSPx));

  /* Transmit Data */
  SPIx->DR = Data;

}

/**
  * @brief  Returns the most recent received data by the SPIx peripheral.
  * @param  SPIx: Select the SPI peripheral.
  *         This parameter can be one of the following values:
  *         SPI0, SPI1.
  * @retval The received data (7:0) and error flags (15:8).
  */
uint16_t SSP_ReceiveData(SPI_TypeDef* SPIx)
{
  /* Check the parameters */
  assert_param(IS_SPI_ALL_PERIPH(SPIx));

  /* Receive Data */
  return ((uint16_t)(SPIx->DR));
}

/**
  * @brief  Checks whether the specified SPI flag is set or not.
  * @param  SPIx: Select the SPI or the SPI peripheral.
  *         This parameter can be one of the following values:
  *         SPI0, SPI1.
  * @param  SPI_FLAG: specifies the flag to check.
  *         This parameter can be one of the following values:
  *           @arg SPI_FLAG_BSY
  *           @arg SPI_FLAG_RFF
  *           @arg SPI_FLAG_RNE
  *           @arg SPI_FLAG_TNF
  * @retval The new state of SPI_FLAG (SET or RESET).
  */
FlagStatus SPI_GetFlagStatus(SPI_TypeDef* SPIx, uint32_t SPI_FLAG)
{
  FlagStatus bitstatus;

  /* Check the parameters */
  assert_param(IS_SPI_ALL_PERIPH(SPIx));
  assert_param(IS_SPI_FLAG(SPI_FLAG));

  if ( SPIx->SR & SPI_FLAG ) {
    bitstatus = SET;
  } else {
    bitstatus = RESET;
  }

  return (bitstatus);
}

/**
  * @brief  Control SS pin the SPIx peripheral.
  * @param  SPIx: Select the SPI peripheral.
  *         This parameter can be one of the following values:
  *         SPI0, SPI1.
  * @param  LevelValue: the data to SS pin.
  * @retval None
  */
void SPI_SoftSS(SPI_TypeDef* SPIx, BitStatus LevelValue)
{
  /* Check the parameters */
  assert_param(IS_SPI_ALL_PERIPH(SPIx));

  SPIx->SOFTSS = (1 << SPI_SOFTSS_EN_Pos) | ((LevelValue << SPI_SOFTSS_LEVEL_Pos) & SPI_SOFTSS_LEVEL_Msk);
}

/**
  * @}
  */
