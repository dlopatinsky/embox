/**
  ******************************************************************************
  * @file    1888bc048_arinc.c
  * @author  V. Syrtsov
  * @version V1.0.1
  * @date    
  * @brief   This file provides firmware functions to manage the following 
  *          functionalities of the ARINC429:
  *           .......
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
#include "1888bc048_arinc.h"


/**
  * @brief  
  * @param  
  * @retval None
  */
void ARINC429TX_ChannelDeInit(uint32_t ARINC429_CHANNELx)
{
	/* Check the parameters */
	assert_param(IS_ARINC429_CHANNEL(ARINC429_CHANNELx));	
	
	ARINC429->CHANNEL_RST = (1 << ARINC429_CHANNELx) << ARINC429_CHANNEL_RST_TX_Pos;
}

/**
  * @brief  
  * @param  
  * @retval None
  */
void ARINC429TX_ChannelInit(uint32_t ARINC429_CHANNELx, ARINC429_InitChannelTypeDef * ARINC429_InitChanelStruct)
{
        __IO uint32_t tmpFREQ_TX;
	/* Check the parameters */
	assert_param(IS_ARINC429_CHANNEL(ARINC429_CHANNELx));
	assert_param(IS_ARINC429_CLK(ARINC429_InitChanelStruct->ARINC429_CLK));
	assert_param(IS_FUNCTIONAL_STATE(ARINC429_InitChanelStruct->ARINC429_EN_PAR));
	assert_param(IS_ARINC429_TX_GAP(ARINC429_InitChanelStruct->ARINC429_GAP));
	assert_param(IS_ARINC429_ODD(ARINC429_InitChanelStruct->ARINC429_ODD));
	
	ARINC429->PARITY_ODD_bit.PARITY_ODD_TX &= ~(1 << ARINC429_CHANNELx);
	ARINC429->PARITY_ODD_bit.PARITY_ODD_TX |= (ARINC429_InitChanelStruct->ARINC429_ODD << ARINC429_CHANNELx);	
	
	if ( ARINC429_InitChanelStruct->ARINC429_EN_PAR != DISABLE ) {
		ARINC429->PARITY_BIT_bit.PARITY_BIT_TX |= (1 << ARINC429_CHANNELx);
                tmpFREQ_TX =  ARINC429_FREQ_TX_WBS_AND_APBS_UNLOCK_Msk        |
                              (32 << ARINC429_FREQ_TX_WORD_BITS_SIZE_Pos)     | 
                              (1 << ARINC429_FREQ_TX_AXI_PARITY_BIT_DIS_Pos);
	} else {
		ARINC429->PARITY_BIT_bit.PARITY_BIT_TX &= ~(1 << ARINC429_CHANNELx);
	}
	
	ARINC429->FREQ_TX[ARINC429_CHANNELx] = 	tmpFREQ_TX                          |
                                                ARINC429_FREQ_TX_FREQ_UNLOCK_Msk    |                                                
						ARINC429_FREQ_TX_GAP_UNLOCK_Msk     |                                                
						((ARINC429_InitChanelStruct->ARINC429_GAP << ARINC429_FREQ_TX_GAP_Pos) & ARINC429_FREQ_TX_GAP_Msk) |	
						((ARINC429_InitChanelStruct->ARINC429_CLK << ARINC429_FREQ_TX_FREQ_Pos) & ARINC429_FREQ_TX_FREQ_Msk);
}

/**
  * @brief  
  * @param  
  * @retval None
  */
void ARINC429TX_ChannelCmd(uint32_t ARINC429_CHANNELx, FunctionalState NewState)
{
	/* Check the parameters */
	assert_param(IS_ARINC429_CHANNEL(ARINC429_CHANNELx));
	assert_param(IS_FUNCTIONAL_STATE(NewState));
	
	if ( NewState != DISABLE ) {
		ARINC429->CHANNEL_EN = (1 << ARINC429_CHANNELx) << ARINC429_CHANNEL_EN_TX_Pos;
	} else {
		ARINC429->CHANNEL_DIS = (1 << ARINC429_CHANNELx) << ARINC429_CHANNEL_DIS_TX_Pos;
	}
}

/**
  * @brief  
  * @param  
  * @retval None
  */
void ARINC429TX_FIFOForceSwitchDMA(uint32_t ARINC429_FIFOx)
{
	/* Check the parameters */
	assert_param(IS_ARINC429_TX_FIFO(ARINC429_FIFOx));
	
	ARINC429->SW_SIG_TX = 1 << ARINC429_FIFOx;
}

/**
  * @brief  
  * @param  
  * @retval None
  */
void ARINC429TX_DMAInit(uint32_t ARINC429_FIFOx, ARINC429_FIFO_DMA_ConfigData_TypeDef* DMADscr)
{
	/* Check the parameters */
	assert_param(IS_ARINC429_TX_FIFO(ARINC429_FIFOx));
	
	ARINC429->TX_DMA_PRM_SRC[ARINC429_FIFOx] = (uint32_t)DMADscr->PRM_DATA.SRC;
	ARINC429->TX_DMA_PRM_SRC_SIZE[ARINC429_FIFOx] = DMADscr->PRM_DATA.SRC_WORD_SIZE;
	
	ARINC429->TX_DMA_ALT_SRC[ARINC429_FIFOx] = (uint32_t)DMADscr->ALT_DATA.SRC;
	ARINC429->TX_DMA_ALT_SRC_SIZE[ARINC429_FIFOx] = DMADscr->ALT_DATA.SRC_WORD_SIZE;
}

/**
  * @brief  
  * @param  
  * @retval None
  */
void ARINC429TX_DMASwitchModeConfig(uint32_t ARINC429_FIFOx, uint32_t Mode)
{
	/* Check the parameters */
	assert_param(IS_ARINC429_TX_FIFO(ARINC429_FIFOx));		
	assert_param(IS_ARINC429_DMA_SWITCH(Mode));
	
	switch ( Mode ) {
		case ARINC429_DMA_Switch_Immediately:
			ARINC429->WAIT_TMR_TX_bit.WAIT_TX &= ~(1 << ARINC429_FIFOx);
			ARINC429->WAIT_TMR_TX_bit.TIMER_TX &= ~(1 << ARINC429_FIFOx);
			break;
		case ARINC429_DMA_Switch_On_SIG_Event:
			ARINC429->WAIT_TMR_TX_bit.TIMER_TX &= ~(1 << ARINC429_FIFOx);
			ARINC429->WAIT_TMR_TX_bit.WAIT_TX |= (1 << ARINC429_FIFOx);
			break;
		case ARINC429_DMA_Switch_On_TMR_Event:
			ARINC429->WAIT_TMR_TX_bit.TIMER_TX |= (1 << ARINC429_FIFOx);
			ARINC429->WAIT_TMR_TX_bit.WAIT_TX |= (1 << ARINC429_FIFOx);
			break;			
		default:
			break;
	}
}

/**
  * @brief  
  * @param  
  * @retval The new state of DMA_FLAG (SET or RESET).
  */
FlagStatus ARINC429TX_DMA_GetFlagStatus(uint32_t ARINC429_FIFOx, uint32_t DMA_Flag)
{
	/* Check the parameters */
	assert_param(IS_ARINC429_TX_FIFO(ARINC429_FIFOx));	
	assert_param(IS_ARINC429_DMA_FLAG(DMA_Flag));
	
	/* Read Flag Status */
	switch ( DMA_Flag ) {
		case ARINC429_DMA_FLAG_PRM_DONE:
			return (FlagStatus)(ARINC429->TX_DMA_DONE_bit.TX_DMA_PRM_DONE & (1 << ARINC429_FIFOx));
			break;
		case ARINC429_DMA_FLAG_ALT_DONE:
			return (FlagStatus)(ARINC429->TX_DMA_DONE_bit.TX_DMA_ALT_DONE & (1 << ARINC429_FIFOx));
			break;	
		default:
			return (FlagStatus)0;
	}
}

/**
  * @brief  
  * @param  
  * @retval Pointer on DMA Status register.
  */
ARINC429_FIFO_DMA_StatusStruct_TypeDef * ARINC429TX_DMA_GetStatus(uint32_t ARINC429_FIFOx, uint32_t DMA)
{
	/* Check the parameters */
	assert_param(IS_ARINC429_TX_FIFO(ARINC429_FIFOx));	
	assert_param(IS_ARINC429_DMA(DMA));	
	
	switch ( DMA ) {
		case ARINC429_DMA_PRM:
			return (ARINC429_FIFO_DMA_StatusStruct_TypeDef *)&ARINC429->TX_DMA_PRM_STATUS[ARINC429_FIFOx];
			break;
		case ARINC429_DMA_ALT:
			return (ARINC429_FIFO_DMA_StatusStruct_TypeDef *)&ARINC429->TX_DMA_ALT_STATUS[ARINC429_FIFOx];
			break;	
		default:
			return (ARINC429_FIFO_DMA_StatusStruct_TypeDef *)0;
	}	
}

/**
  * @brief  
  * @param  
  * @retval None
  */
void ARINC429TX_ITConfig(uint32_t ARINC429_FIFOx, FunctionalState NewState)
{
	/* Check the parameters */
	assert_param(IS_ARINC429_TX_FIFO(ARINC429_FIFOx));
	assert_param(IS_FUNCTIONAL_STATE(NewState));
	
	if ( NewState != DISABLE ) {
		ARINC429->IENR_TX_DMA_DONE |= (1 << ARINC429_FIFOx);
	} else {
		ARINC429->IENR_TX_DMA_DONE &= ~(1 << ARINC429_FIFOx);
	}
}

/**
  * @brief  
  * @param  
  * @retval None
  */
void ARINC429RX_ChannelDeInit(uint32_t ARINC429_CHANNELx)
{
	/* Check the parameters */
	assert_param(IS_ARINC429_CHANNEL(ARINC429_CHANNELx));	
	
	ARINC429->CHANNEL_RST = (1 << ARINC429_CHANNELx) << ARINC429_CHANNEL_RST_RX_Pos;
}

/**
  * @brief  
  * @param  
  * @retval None
  */
void ARINC429RX_ChannelInit(uint32_t ARINC429_CHANNELx, ARINC429_InitChannelTypeDef * ARINC429_InitChanelStruct)
{
	/* Check the parameters */
	assert_param(IS_ARINC429_CHANNEL(ARINC429_CHANNELx));
	assert_param(IS_ARINC429_CLK(ARINC429_InitChanelStruct->ARINC429_CLK));
	assert_param(IS_FUNCTIONAL_STATE(ARINC429_InitChanelStruct->ARINC429_EN_PAR));
	assert_param(IS_ARINC429_RX_GAP(ARINC429_InitChanelStruct->ARINC429_GAP));
	assert_param(IS_ARINC429_ODD(ARINC429_InitChanelStruct->ARINC429_ODD));
	
	ARINC429->PARITY_ODD_bit.PARITY_ODD_RX &= ~(1 << ARINC429_CHANNELx);
	ARINC429->PARITY_ODD_bit.PARITY_ODD_RX |= (ARINC429_InitChanelStruct->ARINC429_ODD << ARINC429_CHANNELx);	
	
	if ( ARINC429_InitChanelStruct->ARINC429_EN_PAR != DISABLE ) {
		ARINC429->PARITY_BIT_bit.PARITY_BIT_RX |= (1 << ARINC429_CHANNELx);
	} else {
		ARINC429->PARITY_BIT_bit.PARITY_BIT_RX &= ~(1 << ARINC429_CHANNELx);
	}
	
	ARINC429->FREQ_RX[ARINC429_CHANNELx] = 	ARINC429_FREQ_RX_FREQ_UNLOCK_Msk		|
											ARINC429_FREQ_RX_PTIMER_UNLOCK_Msk		|
											((ARINC429_InitChanelStruct->ARINC429_GAP << ARINC429_FREQ_RX_PTIMER_Pos) & ARINC429_FREQ_RX_PTIMER_Msk) |	
											((ARINC429_InitChanelStruct->ARINC429_CLK << ARINC429_FREQ_RX_FREQ_Pos) & ARINC429_FREQ_RX_FREQ_Msk);
}

/**
  * @brief  
  * @param  
  * @retval None
  */
void ARINC429RX_ChannelCmd(uint32_t ARINC429_CHANNELx, FunctionalState NewState)
{
	/* Check the parameters */
	assert_param(IS_ARINC429_CHANNEL(ARINC429_CHANNELx));
	assert_param(IS_FUNCTIONAL_STATE(NewState));
	
	if ( NewState != DISABLE ) {
		ARINC429->CHANNEL_EN = (1 << ARINC429_CHANNELx) << ARINC429_CHANNEL_EN_RX_Pos;
	} else {
		ARINC429->CHANNEL_DIS = (1 << ARINC429_CHANNELx) << ARINC429_CHANNEL_DIS_RX_Pos;
	}
}

/**
  * @brief  
  * @param  
  * @retval None
  */
void ARINC429RX_FIFOAssignChannel(uint32_t ARINC429_FIFOx, uint32_t ARINC429_CHANNELx)
{
	/* Check the parameters */
	assert_param(IS_ARINC429_RX_FIFO(ARINC429_FIFOx));
	assert_param(IS_ARINC429_CHANNEL(ARINC429_CHANNELx));	
	
	ARINC429->RX_CHANNEL_ASSIGN_FIFO_bit[ARINC429_FIFOx].NUM_RX_CH = ARINC429_CHANNELx;
}

/**
  * @brief  
  * @param  
  * @retval None
  */
void ARINC429RX_FIFOLabelCmd(uint32_t ARINC429_FIFOx, FunctionalState NewState)
{
	/* Check the parameters */
	assert_param(IS_ARINC429_RX_FIFO(ARINC429_FIFOx));	
	assert_param(IS_FUNCTIONAL_STATE(NewState));	
	
	if ( NewState != DISABLE ) {
		ARINC429->LABEL_EN_RX |= (1 << ARINC429_FIFOx);
	} else {
		ARINC429->LABEL_EN_RX &= ~(1 << ARINC429_FIFOx);
	}
}

/**
  * @brief  
  * @param  
  * @retval None
  */
void ARINC429RX_FIFOLoadLabel(uint32_t ARINC429_FIFOx, uint8_t *Label, uint32_t Size)
{
    __IO uint32_t iFIFO;
    /* Check the parameters */
    assert_param(IS_ARINC429_RX_FIFO(ARINC429_FIFOx));

   for ( iFIFO = 0; iFIFO < ((Size <= 32) ? MEM_ALIGN_SIZE(Size, 4) : 8) / 4; iFIFO++ ) {
        ARINC429->RX_FIFO_LABEL[ARINC429_FIFOx] = (((uint32_t)Label[(iFIFO * 4 + 0) % Size] << ARINC429_RX_FIFO_LABEL_0_Pos) & ARINC429_RX_FIFO_LABEL_0_Msk) |
                                                  (((uint32_t)Label[(iFIFO * 4 + 1) % Size] << ARINC429_RX_FIFO_LABEL_1_Pos) & ARINC429_RX_FIFO_LABEL_1_Msk) |
                                                  (((uint32_t)Label[(iFIFO * 4 + 2) % Size] << ARINC429_RX_FIFO_LABEL_2_Pos) & ARINC429_RX_FIFO_LABEL_2_Msk) |
                                                  (((uint32_t)Label[(iFIFO * 4 + 3) % Size] << ARINC429_RX_FIFO_LABEL_3_Pos) & ARINC429_RX_FIFO_LABEL_3_Msk);
        dsb();
        isb();
    }
}

/**
  * @brief  
  * @param  
  * @retval None
  */
void ARINC429RX_FIFOCmd(uint32_t ARINC429_FIFOx, FunctionalState NewState)
{
	/* Check the parameters */
	assert_param(IS_ARINC429_RX_FIFO(ARINC429_FIFOx));	
	assert_param(IS_FUNCTIONAL_STATE(NewState));
	
	if ( NewState != DISABLE ) {
		ARINC429RX_ChannelCmd((uint32_t)ARINC429->RX_CHANNEL_ASSIGN_FIFO_bit[ARINC429_FIFOx].NUM_RX_CH, NewState);
	} else {
		ARINC429->FIFO_DIS_RX = 1 << ARINC429_FIFOx;
	}
}

/**
  * @brief  
  * @param  
  * @retval None
  */
void ARINC429RX_FIFOForceSwitchDMA(uint32_t ARINC429_FIFOx)
{
	/* Check the parameters */
	assert_param(IS_ARINC429_RX_FIFO(ARINC429_FIFOx));
	
	ARINC429->SW_SIG_RX = 1 << ARINC429_FIFOx;
}

/**
  * @brief  
  * @param  
  * @retval None
  */
void ARINC429RX_DMAInit(uint32_t ARINC429_FIFOx, ARINC429_FIFO_DMA_ConfigData_TypeDef* DMADdst)
{
	/* Check the parameters */
	assert_param(IS_ARINC429_RX_FIFO(ARINC429_FIFOx));
	
	ARINC429->RX_DMA_PRM_SRC[ARINC429_FIFOx] = (uint32_t)DMADdst->PRM_DATA.SRC;
	ARINC429->RX_DMA_PRM_SRC_SIZE[ARINC429_FIFOx] = DMADdst->PRM_DATA.SRC_WORD_SIZE;
	
	ARINC429->RX_DMA_ALT_SRC[ARINC429_FIFOx] = (uint32_t)DMADdst->ALT_DATA.SRC;
	ARINC429->RX_DMA_ALT_SRC_SIZE[ARINC429_FIFOx] = DMADdst->ALT_DATA.SRC_WORD_SIZE;	
}

/**
  * @brief  
  * @param  
  * @retval None
  */
void ARINC429RX_DMASwitchModeConfig(uint32_t ARINC429_FIFOx, uint32_t Mode)
{
	/* Check the parameters */
	assert_param(IS_ARINC429_RX_FIFO(ARINC429_FIFOx));		
	assert_param(IS_ARINC429_DMA_SWITCH(Mode));
	
	switch ( Mode ) {
		case ARINC429_DMA_Switch_Immediately:
			ARINC429->WAIT_SIG_RX &= ~(1 << ARINC429_FIFOx);
			ARINC429->TMR_MASK_RX &= ~(1 << ARINC429_FIFOx);
			break;
		case ARINC429_DMA_Switch_On_SIG_Event:
			ARINC429->TMR_MASK_RX &= ~(1 << ARINC429_FIFOx);
			ARINC429->WAIT_SIG_RX |= (1 << ARINC429_FIFOx);
			break;
		case ARINC429_DMA_Switch_On_TMR_Event:
			ARINC429->TMR_MASK_RX |= (1 << ARINC429_FIFOx);
			ARINC429->WAIT_SIG_RX |= (1 << ARINC429_FIFOx);
			break;			
		default:
			break;
	}	
}

/**
  * @brief  
  * @param  
  * @retval The new state of DMA_FLAG (SET or RESET).
  */
FlagStatus ARINC429RX_DMA_GetFlagStatus(uint32_t ARINC429_FIFOx, uint32_t DMA_Flag)
{
	/* Check the parameters */
	assert_param(IS_ARINC429_RX_FIFO(ARINC429_FIFOx));	
	assert_param(IS_ARINC429_DMA_FLAG(DMA_Flag));
	
	/* Read Flag Status */
	switch ( DMA_Flag ) {
		case ARINC429_DMA_FLAG_PRM_DONE:
			return (FlagStatus)(ARINC429->RX_DMA_PRM_DONE & (1 << ARINC429_FIFOx));
			break;
		case ARINC429_DMA_FLAG_ALT_DONE:
			return (FlagStatus)(ARINC429->RX_DMA_ALT_DONE & (1 << ARINC429_FIFOx));
			break;	
		default:
			return (FlagStatus)0;
	}	
}

/**
  * @brief  
  * @param  
  * @retval Pointer on DMA Status register.
  */
ARINC429_FIFO_DMA_StatusStruct_TypeDef * ARINC429RX_DMA_GetStatus(uint32_t ARINC429_FIFOx, uint32_t DMA)
{
	/* Check the parameters */
	assert_param(IS_ARINC429_RX_FIFO(ARINC429_FIFOx));	
	assert_param(IS_ARINC429_DMA(DMA));	
	
	switch ( DMA ) {
		case ARINC429_DMA_PRM:
			return (ARINC429_FIFO_DMA_StatusStruct_TypeDef *)&ARINC429->RX_DMA_PRM_STATUS[ARINC429_FIFOx];
			break;
		case ARINC429_DMA_ALT:
			return (ARINC429_FIFO_DMA_StatusStruct_TypeDef *)&ARINC429->RX_DMA_ALT_STATUS[ARINC429_FIFOx];
			break;	
		default:
			return (ARINC429_FIFO_DMA_StatusStruct_TypeDef *)0;
	}	
}

/**
  * @brief  
  * @param  
  * @retval None
  */
void ARINC429RX_ITConfig(uint32_t ARINC429_FIFOx, FunctionalState NewState)
{
	/* Check the parameters */
	assert_param(IS_ARINC429_RX_FIFO(ARINC429_FIFOx));
	assert_param(IS_FUNCTIONAL_STATE(NewState));
	
	if ( NewState != DISABLE ) {
		ARINC429->IENR_RX_DMA_DONE |= (1 << ARINC429_FIFOx);
	} else {
		ARINC429->IENR_RX_DMA_DONE &= ~(1 << ARINC429_FIFOx);
	}
}

/**
  * @brief  
  * @param  
  * @retval None
  */
void ARINC429_ITPendClear(void)
{
	ARINC429->IR_CLR_bit.CLR = ARINC429_IR_CLR_Msk;
}

/**
  * @brief  
  * @param  
  * @retval None
  */
void ARINC429_SetLoopback(uint32_t ARINC429TX_CHANNELx, uint32_t ARINC429RX_CHANNELx)
{
	/* Check the parameters */
	assert_param(IS_ARINC429_CHANNEL(ARINC429TX_CHANNELx));
	assert_param(IS_ARINC429_CHANNEL(ARINC429RX_CHANNELx));
	
	ARINC429->SELF_RX_bit[ARINC429RX_CHANNELx].NUM_TX_CH = ARINC429TX_CHANNELx;
	ARINC429->SELF_RX_bit[ARINC429RX_CHANNELx].SELF_TEST_EN = 1;
	
}

/**
  * @brief  
  * @param  
  * @retval None
  */
void ARINC429_ResetLoopback(uint32_t ARINC429TX_CHANNELx, uint32_t ARINC429RX_CHANNELx)
{
	/* Check the parameters */
	assert_param(IS_ARINC429_CHANNEL(ARINC429TX_CHANNELx));
	assert_param(IS_ARINC429_CHANNEL(ARINC429RX_CHANNELx));
	
	ARINC429->SELF_RX_bit[ARINC429RX_CHANNELx].SELF_TEST_EN = 0;
}

/**
  * @}
  */
