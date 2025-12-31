/*
 * stm32f103xx_spi_drivers.c
 *
 *  Created on: Sep 14, 2025
 *      Author: stellarbeing22
 *
 *******************************************************************************
 *                              FUNCTION INDEX
 * ----------------------------------------------------------------------------
 *  1.  SPI_PeriClockControl()             - Enable or disable SPI peripheral clock
 *
 *  2.  SPI_init()                         - Initialize SPI peripheral
 *  3.  SPI_Deinit()                       - Reset SPI peripheral registers
 *
 *  4.  SPI_SendData()                     - Send data (blocking mode)
 *  5.  SPI_RecieveData()                  - Receive data (blocking mode)
 *  6.  SPI_SendData_IT()                  - Send data using interrupt (non-blocking)
 *  7.  SPI_RecieveData_IT()               - Receive data using interrupt (non-blocking)
 *
 *  8.  SPI_IRQInterruptConfig()           - Configure NVIC interrupt enable/disable for SPI
 *  9.  SPI_IRQPriorityConfig()            - Configure NVIC interrupt priority for SPI
 * 10.  SPI_IRQHandling()                  - Handle SPI interrupt and delegate to ISR helpers
 *
 * 11.  SPI_GetFlagStatus()                - Read status of a specific SPI flag
 * 12.  SPI_PeripheralControl()            - Enable or disable SPI peripheral
 * 13.  SPI_SSIConfig()                    - Configure internal slave select (SSI)
 * 14.  SPI_SSOEConfig()                   - Configure SS output enable (SSOE)
 * 15.  SPI_ClearOVRFlag()                 - Clear overrun (OVR) flag
 * 16.  SPI_AbortTransmission()            - Abort interrupt-driven SPI transmission
 * 17.  SPI_AbortReception()               - Abort interrupt-driven SPI reception
 *
 * 18.  SPI_ApplicationEventCallback()     - Application callback for SPI events
 *
 * --------------------------- INTERNAL (STATIC) ------------------------------
 * 19.  spi_txe_interrupt_handle()         - Handle TXE interrupt (transmit buffer empty)
 * 20.  spi_rxne_interrupt_handle()        - Handle RXNE interrupt (receive buffer not empty)
 * 21.  spi_ovr_err_interrupt_handle()     - Handle OVR error interrupt
 ******************************************************************************/



#include "stm32f103xx_spi_drivers.h"

static void spi_txe_interrupt_handle(SPI_Handler_t *pHandle);
static void spi_rxne_interrupt_handle(SPI_Handler_t *pHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handler_t *pHandle);


/* GPIO peripheral clock */

/**
 * @fn                        - SPI_PeriClockControl
 *
 * @brief                     - Enables or disables the peripheral clock for the specified SPI peripheral.
 *
 * @param[in]                 - pSPIx  Base address of the SPI peripheral (e.g., SPI1, SPI2).
 * @param[in]                 - En_or_Di ENABLE to turn on the clock, DISABLE to turn it off (uses macros).
 *
 * @return                    - None
 *
 * @note                      - Must be called before initializing the SPI peripheral.
 */


void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t En_or_Di)
{

	//only 2 SPI periphral in stm32f103xx board
	if(En_or_Di == ENABLE)        //enabling clock
	{
		if((pSPIx == SPI1))
		{
			SPI1_PCLK_EN;
		}else if((pSPIx == SPI2))
		{
			SPI2_PCLK_EN;
		}
	}else                        //disabling clock
	{
		if((pSPIx = SPI1))
		{
			SPI1_PCLK_DI;
		}else if((pSPIx == SPI2))
		{
			SPI1_PCLK_DI;
		}
	}
}


/**
 * @fn                        - SPI_init
 *
 * @brief                     - Initializes the SPI peripheral according to the specified configuration in the SPI handle.
 *
 * @param[in]                 - pSPIHandle Pointer to the SPI handle containing SPI configuration settings.
 *
 * @return                    - None
 *
 * @note                      - Ensure peripheral clock is enabled before calling this function.
 */

void SPI_init(SPI_Handler_t *pSPIHandle)
{
	//Configuring CR1 register first
	uint32_t tempReg = 0;

	//Enable peripheral clock
	SPI_PeriClockControl(pSPIHandle->pSPIx,ENABLE);

	//1. configure DeviceMode
		tempReg = pSPIHandle->SPI_config.SPI_DeviceMode << SPI_CR1_MSTR;

	//2. configure BusConfig
		if(pSPIHandle->SPI_config.SPI_BusConfig == SPI_BUS_CONFIG_FDPX)
		{
			//BIDIMIDE = 0
			tempReg &= ~(1 << SPI_CR1_BIDIMODE);
		}else if(pSPIHandle->SPI_config.SPI_BusConfig == SPI_BUS_CONFIG_HDPX)
		{
			//BIDIMODE = 1
			tempReg |= (1 << SPI_CR1_BIDIMODE);
		}else if(pSPIHandle->SPI_config.SPI_BusConfig == SPI_BUS_CONFIG_SPX_RXONLY)
		{
			//BIDIMODE = 0
			tempReg &= ~(1 << SPI_CR1_BIDIMODE);
			//RXONLY = 1
			tempReg |=  (1 << SPI_CR1_RXONLY);
		}

	//3. configure Speed
		tempReg |= (pSPIHandle->SPI_config.SPI_Speed << SPI_CR1_BD);
	//4. configure DFF
		tempReg |= (pSPIHandle->SPI_config.SPI_DFF << SPI_CR1_DFF);
	//5. configure CPHA
		tempReg |= (pSPIHandle->SPI_config.SPI_CPHA << SPI_CR1_CPHA);
	//6. configure CPOL
		tempReg |= (pSPIHandle->SPI_config.SPI_CPOL << SPI_CR1_CPOL);
	//7. configure SSM
		tempReg |= (pSPIHandle->SPI_config.SPI_SSM << SPI_CR1_SSM);

		pSPIHandle->pSPIx->CR1 = tempReg;
}


/**
 * @fn                        - SPI_Deinit
 *
 * @brief                     - Resets the specified SPI peripheral, clearing all registers to their default reset state.
 *
 * @param[in]                 - pSPIx Base address of the SPI peripheral (e.g., SPI1, SPI2).
 *
 * @return                    - None
 */

void SPI_Deinit(SPI_RegDef_t *pSPIx)
{
	if((pSPIx == SPI1))
	{
		SPI1_REG_RESET;
	}else if((pSPIx == SPI2))
	{
		SPI2_REG_RESET;
	}
}



/* Data send and receive */

/*
 * Also called the blocking API because it waits for the whole bit to
 * trasnfer,and no other task can be done meanwhile.
 *
 *                     START
 *                       |
 *                       |
 *  -------------->  |len = 0?| ------------> |exit from the function|
 * 	|					 |	                              |
 * 	|					 |           		              |
 * 	|	 |wait until the Tx buffer is empty|		    |END|
 * 	|	 			     |
 * 	|	 			     |
 * 	|	 			   |DFF?| 1---------16 bit---------------
 * 	|	 			   0 | 8                                |
 * 	|	 			     | bit                              |
 * 	|	  |Load DR with 1 byte of data    |  |Load DR with 1 byte of data    |
 * 	|	  |And incriment the buffer adress|	 |And incriment the buffer adress|
 * 	|	                 |                                  |
 * 	|                    |                                  |
 * 	------------------|Len--| <--------------------------|len--|
 *
 * 	1. you can't put data inside the Tx buffer before its empty, otherwise
 * 	   corrupion can occur. Thus check the SPI_SR->TXE bit.
 * 	2. when the buffer is empty its automatically set to 1.
 * 	3. Similarly to read from Rx buffer, check the SPI_SR->RXNE bit.
 * 	4. if its 1, the Rx buffer is full then data can be read from it.
 * */




/**
 * @fn                        - SPI_SendData
 *
 * @brief                     - Sends data over SPI in blocking mode (polling).
 *
 * @param[in]                 - pSPIx Base address of the SPI peripheral.
 * @param[in]                 - TxBuffer Pointer to the data buffer to be transmitted.
 * @param[in]                 - len Number of bytes to transmit.
 *
 * @return                    - None
 *
 * @note                      - Function blocks until all data is transmitted.
 */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len)
{

	while(len > 0)
	{
		//1Wait for the Tx flag
		while(SPI_GetFlagStatus(pSPIx, SPI_FLAG_TXE) == FLAG_RESET);

		//2. Check DFF, 0-> 8 bit, 1->16bit
		if(pSPIx->CR1 & (1<<SPI_CR1_DFF))
		{
			//16 bit format
			//1. Load the data into the data register
			pSPIx->DR = *((uint16_t *)pTxBuffer);
			len-=2; //twice because sent 2 bytes
			pTxBuffer+=2; //incriment the Tx buffer
		}else
		{
			//8 bit format
			//1. Load the data into the data register
			pSPIx->DR = *pTxBuffer;
			len--;//once because sent 1 byte
			pTxBuffer++;//incriment the Tx buffer
		}
	}
}


/*
 *
 *                     START
 *                       |
 *                       |
 *  -------------->  |len = 0?| ------------> |exit from the function|
 * 	|					 |	                              |
 * 	|					 |           		              |
 * 	| |wait until the Rx buffer is non empty|		    |END|
 * 	|	 			     |
 * 	|	 			     |
 * 	|	 			   |DFF?| 1---------16 bit---------------
 * 	|	 			   0 | 8                                |
 * 	|	 			     | bit                              |
 * 	|	  |Read DR for 1 byte of data     |  |Read DR for 2 bytes of data    |
 * 	|	  |And incriment the buffer adress|	 |And incriment the buffer adress|
 * 	|	                 |                                  |
 * 	|                    |                                  |
 * 	------------------|Len--| <--------------------------|len--|
 *
 * */



/**
 * @fn                        - SPI_RecieveData
 *
 * @brief                     - Receives data from SPI in blocking mode (polling).
 *
 * @param[in]                 - pSPIx Base address of the SPI peripheral.
 * @param[in]                 - RxBuffer Pointer to the buffer where received data will be stored.
 * @param[in]                 - len Number of bytes to receive.
 *
 * @return                    - None
 *
 * @note                      - Function blocks until all data is received.
 */
void SPI_RecieveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len)
{

	while(len > 0)
	{
		//1 Wait for the RXNE flag
		while(SPI_GetFlagStatus(pSPIx, SPI_FLAG_RXNE) == FLAG_RESET);

		//2. Check DFF, 0-> 8 bit, 1->16bit
		if(pSPIx->CR1 & (1<<SPI_CR1_DFF))
		{
			//16 bit format
			//1. Load the data from DR to address buffer
			*((uint16_t *)pRxBuffer) = (uint16_t)pSPIx->DR;
			len-=2; //twice because recived 2 bytes
			pRxBuffer+=2; //incriment the Rx buffer
		}else
		{
			//8 bit format
			//1. Read the data from DR to address buffer
			*pRxBuffer = pSPIx->DR;
			len--;//once because recived 1 byte
			pRxBuffer++;//incriment the Rx buffer
		}
	}
}




/**
 * @fn                        - SPI_SendData_IT
 *
 * @brief                     - Sends data over SPI in non-blocking interrupt mode.
 *
 * @param[in]                 - pSPIHandle Pointer to the SPI handle containing configuration and state.
 * @param[in]                 - TxBuffer Pointer to the data buffer to be transmitted.
 * @param[in]                 - len Number of bytes to transmit.
 *
 * @return                    - uint8_t Returns state of the SPI transmission (READY or BUSY).
 *
 * @note                      - Enables SPI interrupts to handle data transmission.
 */
uint8_t SPI_SendData_IT(SPI_Handler_t *pSPIHandle, uint8_t *TxBuffer, uint32_t len)
{
	uint8_t state = pSPIHandle->TxState;
	if(state != SPI_BUSY_IN_TX)
	{
		//1. Save the Tx buffer adn len information in some gobal variable
		pSPIHandle->pTxBuffer = TxBuffer;
		pSPIHandle->TxLen     = len;

		//2. Mark the SPI state as bust in transmission so that
		//   no other code can take over same SPI peripheral until the transmission is over.
		pSPIHandle->TxState   = SPI_BUSY_IN_TX;

		//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);

		//4. Data transmossion will be handled by the ISR code (Will impliment late)
	}
	return state;
}



/**
 * @fn                        - SPI_RecieveData_IT
 *
 * @brief                     - Receives data over SPI in non-blocking interrupt mode.
 *
 * @param[in]                 - pSPIHandle Pointer to the SPI handle containing configuration and state.
 * @param[in]                 - RxBuffer Pointer to the buffer where received data will be stored.
 * @param[in]                 - len Number of bytes to receive.
 *
 * @return                    - uint8_t Returns state of the SPI reception (READY or BUSY).
 *
 * @note                      - Enables SPI interrupts to handle data reception.
 */
uint8_t SPI_RecieveData_IT(SPI_Handler_t *pSPIHandle, uint8_t *RxBuffer, uint32_t len)
{
	uint8_t state = pSPIHandle->RxState;
	if(state != SPI_BUSY_IN_RX)
	{
		//1. Save the Tx buffer adn len information in some gobal variable
		pSPIHandle->pRxBuffer = RxBuffer;
		pSPIHandle->RxLen     = len;

		//2. Mark the SPI state as bust in transmission so that
		//   no other code can take over same SPI peripheral until the transmission is over.
		pSPIHandle->RxState   = SPI_BUSY_IN_RX;

		//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);

		//4. Data transmossion will be handled by the ISR code (Will impliment late)
	}
	return state;
}


/*
 * Druing SPI communication, interrupts can be generated by the fillowing events
 * 1. Transmit Tx bugger readt to be loaded
 * 2. Data recived in Rx buffer
 * 3. Master mode faut (must avoid in single master mode)
 * 4. overrun error
 *
 * Interrupts can be enables and disabled seperately
 *
 *    =========================SPI Interrupt request======================
 *          Interrupt event      |   Event flag    |  Enable Control bit  |
 *    ====================================================================
 * 1. Transmit Tx buffer ready   |                 |                      |
 *    to be loaded               |_______TXE_______|________TXEIE_________|
 * 2. Data recived in Rx buffer  |_______RXNE______|________RXNEIE________|
 * 3. Master Mode failt event    |       MODF______|                      |
 * 4. Overrun error              |_______OVR_______|        ERRIE         |
 * 5. CRC error                  |_______CRCERR____|                      |
 * 6. TI frame format error      |_______FRE_______|______________________|
 *  ======================================================================
 *
 * */




/**
 * @fn                        - SPI_IRQInterruptConfig
 *
 * @brief                     - Configures the NVIC to enable or disable the specified IRQ number for SPI interrupts.
 *
 * @param[in]                 - IRQNumber SPI IRQ number to configure.
 * @param[in]                 - En_or_Di ENABLE to activate, DISABLE to deactivate.
 *
 * @return                    - None
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t En_or_Di)
{
	if(En_or_Di == ENABLE)
	{
		if(IRQNumber <=31)
		{
			//Configure ISER0 reg
			*NVIC_ISER0 |= (1 << IRQNumber);

		}else if(IRQNumber>=32 && IRQNumber < 64)
		{
			//Configure ISER1 reg
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));

		}else if(IRQNumber>=64 && IRQNumber < 96)
		{
			//Configure ISER2 reg
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}else
	{
		if(IRQNumber <=31)
		{
			//Configure ISER0 reg
			*NVIC_ICER0 |= (1 << IRQNumber);
		}else if(IRQNumber>=32 && IRQNumber < 64)
		{
			//Configure ISER1 reg
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}else if(IRQNumber>=64 && IRQNumber < 96)
		{
			//Configure ISER2 reg
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}
}


/**
 * @fn                        - SPI_IRQPriorityConfig
 *
 * @brief                     - Sets the priority for the specified SPI interrupt in the NVIC.
 *
 * @param[in]                 - IRQNumber SPI IRQ number to configure.
 * @param[in]                 - IRQPriority Priority level (0 to 15, lower is higher priority).
 *
 * @return                    - None
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	//First check the IPR register
		uint8_t IPRx = IRQNumber/4;
		uint8_t IPRx_section = IRQNumber%4;
		uint8_t Shift_ammount = (8*IPRx_section) + (8-PRIORITY_BITS);
	//	*(NVIC_IPR_BASEADDR + (IPRx)) &= ~(0xF<<Shift_ammount);
		*(NVIC_IPR_BASEADDR + (IPRx)) |= (IRQPriority<<Shift_ammount);

		// NVIC_IPR_BASEADDR is a uint32_t* (each step = 4 bytes).
		// Adding IPRx already advances by 4*IPRx bytes to reach the correct IPR register.
		// If you also multiply IPRx by 4, you overshoot (4*4 = 16 bytes per step), causing wrong offsets and hard-to-debug errors.

}


/*			                       =============
 *  			                   | Enter ISR |
 *              			       =============
 *                          			 |
 *			                 | Understand which event|
 *			 ----------------| caused the interrupt  |------------------
 * 			 |               |  trigger (Check SR)   |                 |
 *           |                           |                             |
 *           |                 			 |                             |
 * |Interrupt is due to |    | Interrupt due to the  |   |Interrupt due to setting |
 * |SETTIGN OF RXNE flag|    |setting of the TXE flag|   |       of ERROR flag     |
 *           |                           |                             |
 *           |                           |                             |
 * |  Handle RXNE event |    |    Handle TXE flag    |   |        Handle error     |
 *
 * */




/**
 * @fn                        - SPI_IRQHandling
 *
 * @brief                     - Handles the SPI interrupt by checking and clearing relevant interrupt flags.
 *
 * @param[in]                 - pHandle Pointer to the SPI handle containing configuration and state.
 *
 * @return                    - None
 *
 * @note                      - Should be called inside the global IRQ handler for SPI.
 */
void SPI_IRQHandling(SPI_Handler_t *pHandle)
{
	uint8_t temp1, temp2;
	//first check the TXE
	temp1 = pHandle->pSPIx->SR & (1<<SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & (1<<SPI_CR2_TXEIE);

	if(temp1&&temp2)
	{
		//TXE interrupt handling
		spi_txe_interrupt_handle(pHandle);
	}

	//first check the RXXE
	temp1 = pHandle->pSPIx->SR & (1<<SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & (1<<SPI_CR2_RXNEIE);
	if(temp1&&temp2)
	{
		//RXNE interrupt handling
		spi_rxne_interrupt_handle(pHandle);
	}

	//first check the ovr Error flag
	temp1 = pHandle->pSPIx->SR & (1<<SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & (1<<SPI_CR2_ERRIE);
	if(temp1&&temp2)
	{
		//OVR interrupt handling
		spi_ovr_err_interrupt_handle(pHandle);
	}
}


/* Other Peripheral Control APIs */


/**
 * @fn                        - SPI_GetFlagStatus
 *
 * @brief                     - Returns the status of the specified SPI flag.
 *
 * @param[in]                 - pSPIx Base address of the SPI peripheral.
 * @param[in]                 - FlagName Name of the SPI flag to check (TXE, RXNE, OVR, etc.).
 *
 * @return                    - uint8_t FLAG_SET or FLAG_RESET.
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}


/**
 * @fn                        - SPI_PeripheralControl
 *
 * @brief                     - Enables or disables the SPI peripheral.
 *
 * @param[in]                 - pSPIx Base address of the SPI peripheral.
 * @param[in]                 - En_Or_Di ENABLE to turn on, DISABLE to turn off.
 *
 * @return                    - None
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t En_Or_Di)
{
	if(En_Or_Di == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}


/**
 * @fn                        - SPI_SSIConfig
 *
 * @brief                     - Configures the internal slave select (SSI) for the SPI peripheral.
 *
 * @param[in]                 - pSPIx Base address of the SPI peripheral.
 * @param[in]                 - En_Or_Di ENABLE to set SSI high, DISABLE to clear.
 *
 * @return                    - None
 *
 * @note                      - Necessary when the SPI is configured as a master to prevent MODF error.
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t En_Or_Di)
{
	if(En_Or_Di == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}



/**
 * @fn                        - SPI_SSOEConfig
 *
 * @brief                     - Configures the SS output enable (SSOE) for master SPI mode.
 *
 * @param[in]                 - pSPIx Base address of the SPI peripheral.
 * @param[in]                 - En_Or_Di ENABLE to enable automatic SS management, DISABLE to disable.
 *
 * @return                    - None
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t En_Or_Di)
{
	if(En_Or_Di == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}




/**
 *
 *                        Handle TXE interrupt
 *                                |
 *                                |
 *       -------------------|8 bit/16bit|------------------
 * 	     |					 	                          |
 * 	     |					            		          |
 *  |Write 1 byte of data|                       |Write 2 byte of data|
 *  |to SPI Data Register|                       |to SPI Data Register|
 *       |					 	                          |
 * 	     |					            		          |
 * 	  |Len--| <-----------------------------------> |Len--, Len--|
 * 	                             |
 * 	                         |Len = 0?|---NO--------------
 * 	                         YES |                       |
 * 	                       |Close the|         |Wait till another TXE|
 * 	                       |SPI TX   |         |interrupt            |
 *
 */



/**
 * @fn                        - spi_txe_interrupt_handle
 *
 * @brief                     - Handles the SPI TXE (transmit buffer empty) interrupt for non-blocking transmission.
 *
 * @param[in]                 - pHandle Pointer to the SPI handle containing configuration and transmission state.
 *
 * @return                    - None
 *
 * @note                      - Internal driver function. Not to be called directly by application code.
 */
void spi_txe_interrupt_handle(SPI_Handler_t *pSPIHandle)
{
	//2. Check DFF, 0-> 8 bit, 1->16bit
	if(pSPIHandle->pSPIx->CR1 & (1<<SPI_CR1_DFF))
	{
		//16 bit format
		//1. Load the data into the data register
		pSPIHandle->pSPIx->DR = *((uint16_t *)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen-=2;//twice because sent 2 bytes
		pSPIHandle->pTxBuffer+=2; //incriment the Tx buffer
	}else
	{
		//8 bit format
		//1. Load the data into the data register
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;//once because sent 1 byte
		pSPIHandle->pTxBuffer++;//incriment the Tx buffer
	}

	if(!pSPIHandle->TxLen)
	{
		//TxLen is zero, so close the SPI comm. and inform the
		//application that TX is over

		SPI_AbortTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}


/** TODO: update this fow chart
 *
 *                        Handle RXNE interrupt
 *                                |
 *                                |
 *       -------------------|8 bit/16bit|------------------
 * 	     |					 	                          |
 * 	     |					            		          |
 *  |Write 1 byte of data|                       |Write 2 byte of data|
 *  |to SPI Data Register|                       |to SPI Data Register|
 *       |					 	                          |
 * 	     |					            		          |
 * 	  |Len--| <-----------------------------------> |Len--, Len--|
 * 	                             |
 * 	                         |Len = 0?|---NO--------------
 * 	                         YES |                       |
 * 	                       |Close the|         |Wait till another TXE|
 * 	                       |SPI TX   |         |interrupt            |
 *
 */


/**
 * @fn                        - spi_rxne_interrupt_handle
 *
 * @brief                     - Handles the SPI RXNE (receive buffer not empty) interrupt for non-blocking reception.
 *
 * @param[in]                 - pHandle Pointer to the SPI handle containing configuration and reception state.
 *
 * @return                    - None
 *
 * @note                      - Internal driver function. Not to be called directly by application code.
 */
void spi_rxne_interrupt_handle(SPI_Handler_t *pSPIHandle)
{
	//2. Check DFF, 0-> 8 bit, 1->16bit
	if(pSPIHandle->pSPIx->CR1 & (1<<SPI_CR1_DFF))
	{
		//16 bit format
		//1. Load the data from DR to address buffer
		*((uint16_t *)pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen-=2; //twice because recived 2 bytes
		pSPIHandle->pRxBuffer+=2; //incriment the Rx buffer
	}else
	{
		//8 bit format
		//1. Read the data from DR to address buffer
		*pSPIHandle->pRxBuffer = pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;//once because recived 1 byte
		pSPIHandle->pRxBuffer++;//incriment the Rx buffer
	}

	if(!pSPIHandle->TxLen)
	{
		//TxLen is zero, so close the SPI comm. and inform the
		//application that TX is over

		//clear out the TXEIE bit, and prevent interrupts from setting up the TXE flag
		SPI_AbortReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}


/**
 * @fn                        - spi_ovr_err_interrupt_handle
 *
 * @brief                     - Handles the SPI overrun (OVR) error interrupt, clearing flags and notifying application if needed.
 *
 * @param[in]                 - pHandle Pointer to the SPI handle containing configuration and error state.
 *
 * @return                    - None
 *
 * @note                      - Internal driver function. Not to be called directly by application code.
 */
void spi_ovr_err_interrupt_handle(SPI_Handler_t *pHandle)
{
	uint8_t temp;
	//1. Clear the OVR flag
	if(pHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pHandle->pSPIx->DR;
		temp = pHandle->pSPIx->SR;
	}
	(void)temp;
	//i2. nform the application
	SPI_ApplicationEventCallback(pHandle, SPI_EVENT_OVR_ERR);

}



/**
 * @fn                        - SPI_ClearOVRFlag
 *
 * @brief                     - Clears the overrun (OVR) flag in the SPI status register.
 *
 * @param[in]                 - pSPIx Base address of the SPI peripheral.
 *
 * @return                    - None
 *
 * @note                      - Must be called after reading received data to prevent SPI from locking.
 */
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}


/**
 * @fn                        - SPI_AbortTransmission
 *
 * @brief                     - Aborts an ongoing SPI transmission in interrupt mode.
 *
 * @param[in]                 - pSPIHandle Pointer to the SPI handle containing configuration and state.
 *
 * @return                    - None
 */
void SPI_AbortTransmission(SPI_Handler_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1<<SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}


/**
 * @fn                        - SPI_AbortReception
 *
 * @brief                     - Aborts an ongoing SPI reception in interrupt mode.
 *
 * @param[in]                 - pSPIHandle Pointer to the SPI handle containing configuration and state.
 *
 * @return                    - None
 */

void SPI_AbortReception(SPI_Handler_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1<<SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}


/**
 * @fn                        - SPI_ApplicationEventCallback
 *
 * @brief                     - Callback function invoked by the SPI driver on specific application events (e.g., transmission complete, reception complete, OVR error).
 *
 * @param[in]                 - pSPIHandle Pointer to the SPI handle generating the event.
 * @param[in]                 - AppEve Type of event that occurred (transmission complete, reception complete, etc.).
 *
 * @return                    - None
 *
 * @note                      - This is an weak implimentation, the application may overwrite it
 */
__weak void SPI_ApplicationEventCallback(SPI_Handler_t *pSPIHandle,uint8_t AppEve)
{
	//This is an weak implimentation, the application may overwrite it
}


