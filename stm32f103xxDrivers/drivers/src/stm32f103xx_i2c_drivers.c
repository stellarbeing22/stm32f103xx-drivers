/*
 * stm32f103xx_i2c_drivers.c
 *
 *  Created on: Oct 19, 2025
 *      Author: stellarbeing22
 */

#include "stm32f103xx_i2c_drivers.h"

//the array name says it all, the prescalers for AHB bus and APB1 bus
uint16_t AHB_Prescaler[8] = {2,4,8,16,32,64,128,256};
uint8_t APB1_Prescaler[4] = {2,4,8,16};

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2CHandle, uint8_t SlaveAddr);
static void I2C_ClearAddrFlag(I2C_RegDef_t *pI2Cx);




/* GPIO peripheral clock */

/**
 * @fn                        - I2C_PeriClockControl
 *
 * @brief                     - Enables or disables the peripheral clock for the specified I2C peripheral.
 *
 * @param[in]                 - pI2Cx  Base address of the I2C peripheral (e.g., I2C1, I2C2).
 * @param[in]                 - En_or_Di ENABLE to turn on the clock, DISABLE to turn it off (uses macros).
 *
 * @return                    - None
 *
 * @note                      - Must be called before initializing the I2C peripheral.
 */

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t En_or_Di)
{
	if(En_or_Di == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN;
		}else
		{
			I2C2_PCLK_EN;
		}
	}else
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN;
		}else
		{
			I2C2_PCLK_EN;
		}
	}
}

/* Init and Deinit */



/* Steps to create clock (Sm)
 *
 * 1. Configure the mode in CCR, Sm/Fm (select the duty cycle using CCR 14 in Fm)
 * 2. Program the FREQ field in CR2 with value of Pclk1 (8MHz default, HSi,F1)
 * 3. Calculate the value of CCR using the subtable formula and fill the bitfield
 *    inside the CCR of CCR Reg
 *
 *    for Sm:
 *    	Thigh = CCR * T PCLK1
 *    	Tlow = CCR * TPCLK1
 *
 *    for Fm:
 *    	If DUTY = 0:
 *    		Thigh = CCR * T PCLK1
 *    		Tlow = 2 * CCR * TPCLK1
 *    	If DUTY = 1:
 *    		Thigh = 9 * CCR * TPCLK1
 *    		Tlow = 16 * CCR * TPCLK1
 *
 *   for example, lets say PCLK1 = 8MHz (TPCLK = 125nS) and we need 100KHz (Sm) on SCL
 *   CCR[15] <- Sm
 *   FREQ[5:0] <- 8
 *   now calculate CCR
 *
 *   for 100KHz, Sm, Thigh = Tlow = 5 uS
 *
 *   thus 5 uS = CCR * 125 nS
 *   => CCR= 40
 *
 *  # For Sm: Tlow ~ Thigh
 *  # For Fm: Tlow = 2Thigh or 1.8Thigh
 *
 *
 *  Clock streaching
 *  The holding of the clock to the ground (0) level
 *
 *  The moment the clock is held at low, the whole I2C
 *  interface pauses until clock is given its optimal operation level
 * */



/**
 * @fn                        - I2C_init
 *
 * @brief                     - Initializes the I2C peripheral according to the specified configuration in the I2C handle.
 *
 * @param[in]                 - pI2CHandle Pointer to the I2C handle containing I2C configuration settings.
 *
 * @return                    - None
 *
 * @note                      - Ensure peripheral clock is enabled before calling this function.
 */
void I2C_init(I2C_Handler_t *pI2CHandle)
{
	uint32_t tempReg = 0;

	//1. Enable peripheral Clock
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	//2. Enable ACK control
	tempReg |= pI2CHandle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK;
	pI2CHandle->pI2Cx->CR1 = tempReg;

	//3. set FREQ[5:0]
	tempReg = 0;
	tempReg |= RCC_getPCLK1value()/1000000U;
	pI2CHandle->pI2Cx->CR2 = tempReg & 0x3F;

	//4. setup the device address (slave mode)
	tempReg = 0;
	tempReg |= pI2CHandle->I2C_Config.I2C_DeviceAddress>>1;
	tempReg |= (1<<14);
	pI2CHandle->pI2Cx->OAR1 = tempReg;

	//5. configure CCR value

	uint16_t CCR_Value = 0;
	tempReg = 0;
	//Standard mode at 100KHz
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{	//CCR = fpclk1/2fscl
		CCR_Value = (RCC_getPCLK1value()/(2 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		tempReg |= CCR_Value & 0xFFF;
	}else
	{	//Fast mode at 200 and 400KHz
		tempReg |= (1<<15);                                                             //Enable Fast mode
		tempReg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle<<14);                        //Select Duty cycle, 1:2 or 9:16
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_DUTY_2)                        //1:2
		{
			CCR_Value = (RCC_getPCLK1value()/(3 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}else                                                                           //9:16
		{
			CCR_Value = (RCC_getPCLK1value()/(25 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		tempReg |= CCR_Value & 0xFFF;
	}
	pI2CHandle->pI2Cx->CCR = tempReg;

	//6. Configure Trise
	//TODO



}


/**
 * @fn                        - I2C_Deinit
 *
 * @brief                     - Resets the specified I2C peripheral, clearing all registers to their default reset state.
 *
 * @param[in]                 - pI2Cx Base address of the I2C peripheral (e.g., I2C1, I2C2).
 *
 * @return                    - None
 */
void I2C_Deinit(I2C_RegDef_t *pI2Cx)
{
	if((pI2Cx == I2C1))
	{
		I2C1_REG_RESET;
	}else if((pI2Cx == I2C2))
	{
		I2C2_REG_RESET;
	}
}

/**
 * @fn                        - I2CmasterSendData
 *
 * @brief                     - Transmits a block of data from the I2C master to a specified slave
 *                               using polling (blocking) mode.
 *
 * @param[in]                 - pI2CHandle  Pointer to the I2C handle structure containing
 *                              peripheral base address and configuration.

 * @param[in]                 - pTxBuffer  Pointer to the transmit buffer holding data to be sent.
 *
 * @param[in]                 - Len Number of bytes to transmit.
 *
 * @param[in]                 - SlaveAddr  7-bit I2C slave address.
 *
 * @return  void
 */


/*
 * This function generates a START condition, sends the 7-bit slave
 * address with write intent, transmits the data bytes sequentially,
 * and finally generates a STOP condition. The function blocks until
 * the entire transmission is complete. It assumes the I2C peripheral
 * is already initialized and enabled.*/
void I2CmasterSendData(I2C_Handler_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t SlaveAddr)
{
	//1. generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. Confirm that the start generation is competed by checking the SB flag in the SR1 reg
	//note: until SB is cleared SCL will be streached (pulled to LOW)
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	//3. send the address of the slave with e/wr bit set to w(0) (total 8 bits)
	I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, SlaveAddr);

	//4. confirm the address phase is completed by checking the ADDR flag in SR1
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	//5. clear the ADDR flag according to its software sequence
	//note: until ADDR us cleared SCL will be streached (pulled to LOW)
	I2C_ClearAddrFlag(pI2CHandle->pI2Cx);

	//6. send the data until len != 0
	while(len != 0)
	{
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE)); //Wait till Txe set
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		len--;
	}

	//7. when Len == 0 wait for TXE = 1 and BTF  = 1 before genrating the STOP conditon
	//note: TXE = 1 AND BTF = 1 means that both SR and DR are empty and next transmission should beigin
	//when BTF = 1 SCL will be streached (pulled to LOW)
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE)); //Wait till Txe set
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF)); //Wait till BTF set

	//8.Generate the STOP conditon and master need not to wait for th completion of the stop condition
	//note: generating STOP automatically clears the BTF
	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
}




/* IQR config and ISR handling */

/**
 * @fn                        - I2C_IRQInterruptConfig
 *
 * @brief                     - Configures the NVIC to enable or disable the specified IRQ number for I2C interrupts.
 *
 * @param[in]                 - IRQNumber I2C IRQ number to configure.
 * @param[in]                 - En_or_Di ENABLE to activate, DISABLE to deactivate.
 *
 * @return                    - None
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t En_or_Di)
{}

/**
 * @fn                        - I2C_IRQPriorityConfig
 *
 * @brief                     - Sets the priority for the specified I2C interrupt in the NVIC.
 *
 * @param[in]                 - IRQNumber I2C IRQ number to configure.
 * @param[in]                 - IRQPriority Priority level (0 to 15, lower is higher priority).
 *
 * @return                    - None
 */
void I2C_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{}

/* Other Peripheral Control APIs */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	if(pI2Cx->SR1 & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t En_Or_Di)
{
	if(En_Or_Di == ENABLE)
	{
		pI2Cx->CR1 |= (1<<I2C_CR1_PE);
	}else
	{
		pI2Cx->CR1 &= ~(1<<I2C_CR1_PE);
	}
}

/*Application event*/


/**
 * @fn                        - I2C_ApplicationEventCallback
 *
 * @brief                     - Callback function invoked by the I2C driver on specific application events.
 *
 * @param[in]                 - pI2C Handle Pointer to the I2C handle generating the event.
 * @param[in]                 - AppEve Type of event that occurred (transmission complete, reception complete, etc.).
 *
 * @return                    - None
 *
 * @note                      - This is an weak implimentation, the application may overwrite it
 */
 __weak void I2C_ApplicationEventCallback(I2C_Handler_t *pI2CHandle,uint8_t AppEve)
 {}


 /**
  * @fn                        - RCC_getPCLK1value
  *
  * @brief                     - Return the PCLK1 frequency value.
  *
  * @param[in]                 - None.
  *   *
  * @return                    - uint32_t returns the clock frequency value.
  *
  * @note                      - As of now This fn does not support PLL clock frequency.
  */
 uint32_t RCC_getPCLK1value()
 {
	 //pclk1 -> the clock of the APB1 bus (max 32MHz), sysclk is the system clock;
	 uint32_t pclk1,sysclk;

	 //clksrc is the sourse of the clock =(HSI, HSE, PLL), AHB_Prescaler,
	 //APB1_Prescaler are the prescaler values, HPRE_Value is the clock div(AHB)
	 //and PPRE1_Value is the clock div(APB1)
	 uint8_t clksrc, AHB_Prescaler_Value,APB1_Prescaler_Value, HPRE_Value, PPRE1_Value;


	 /*Get source clock speed, check if = 0/1/2
	  * 0 => HSI
	  * 1 => HSE
	  * 2 => PLL
	  *  HSI = HSE = 8MHz
	  */
	 clksrc = ((RCC->CFGR >> 2)&0x3);
	 if(clksrc == 0)
	 {
		 sysclk = 8000000;
	 }else if(clksrc == 1)
	 {
		 sysclk = 8000000;
	 }else if(clksrc == 2)
	 {
		 sysclk = RCC_getPLLOutputClk();
	 }

	 /*Get HPRE value and check
	  * < 8 => 1
	  *   8 => 2
	  *   9 => 4
	  *   10=> 8
	  *   11=> 16
	  *   12=> 32
	  *   13=> 64
	  *   14=> 128
	  *   15=> 256 */
	 HPRE_Value = ((RCC->CFGR >> 4)&0xF);
	 if(HPRE_Value < 8)
	 {
		 AHB_Prescaler_Value = 1;
	 }else
	 {
		 AHB_Prescaler_Value = AHB_Prescaler[HPRE_Value-8];
	 }

	 /*Get HPRE value and check
	  * < 4 => 1
	  *   4 => 2
	  *   5 => 4
	  *   6 => 8
	  *   7 => 16
	  */
	 PPRE1_Value = ((RCC->CFGR >> 8)&0x7);
	 if(PPRE1_Value < 4)
	 {
		 APB1_Prescaler_Value = 1;
	 }else
	 {
		 APB1_Prescaler_Value = APB1_Prescaler[PPRE1_Value-4];
	 }
	 pclk1 = (sysclk/AHB_Prescaler_Value)/APB1_Prescaler_Value;
	 return pclk1;
 }


 /**
  * @fn                        - RCC_getPLLOutputClk
  *
  * @brief                     - Return the PLL frequency value.
  *
  * @param[in]                 - None.
  *   *
  * @return                    - uint32_t returns the clock frequency value.
  *
  * @note                      - As of now This fn is not implimented.

  * uint32_t RCC_getPLLOutputClk()
  * {}
  * */

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1<<I2C_CR1_START);
}

static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1<<I2C_CR1_STOP);
}

static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1; //make space for r/w bit
	SlaveAddr &= ~(1); //Slave address + r/w bit
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ClearAddrFlag(I2C_RegDef_t *pI2Cx)
{
	uint32_t DummyRead = pI2Cx->SR1;
	DummyRead = pI2Cx->SR2;
	(void)DummyRead;
}
