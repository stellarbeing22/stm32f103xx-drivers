/*
 * stm32f103xx_spi_drivers.h
 *
 *  Created on: Sep 14, 2025
 *      Author: stellarbeing22
 *
========================================================================================================================
                                             FILE INDEX : stm32f103xx_spi_drivers.h
========================================================================================================================

1.  Includes and Header Guards
    1.1  Include dependency: stm32f103xx.h
    1.2  Header guards: INC_STM32F103XX_SPI_DRIVERS_H_

2.  Data Structures
    2.1  SPI_Config_t
         - Configuration parameters: DeviceMode, BusConfig, Speed, DFF, CPHA, CPOL, SSM
    2.2  SPI_Handler_t
         - Pointers and state tracking: pSPIx, pTxBuffer, pRxBuffer, TxLen, RxLen, TxState, RxState

3.  Macros
    3.1  SPI application states
         - SPI_READY, SPI_BUSY_IN_RX, SPI_BUSY_IN_TX
    3.2  SPI application events
         - SPI_EVENT_TX_CMPLT, SPI_EVENT_RX_CMPLT, SPI_EVENT_OVR_ERR
    3.3  SPI configuration macros
         - SPI_DeviceMode: MASTER / SLAVE
         - SPI_BusConfig: Full, Half, Simplex Rx-only
         - SPI_Speed: Clock division factors (DIV_2 to DIV_256)
         - SPI_DFF: 8-bit / 16-bit
         - SPI_CPOL, SPI_CPHA
         - SPI_SSM: Enable / Disable
    3.4  SPI status flag macros
         - SPI_RXNE_FLAG, SPI_TXE_FLAG, SPI_OVR_FLAG, SPI_BSY_FLAG, etc.

4.  Driver API Prototypes
    4.1  Peripheral clock control
         - void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t En_or_Di);
    4.2  Initialization and deinitialization
         - void SPI_init(SPI_Handler_t *pSPIHandle);
         - void SPI_Deinit(SPI_RegDef_t *pSPIx);
    4.3  Data transmission and reception
         - void SPI_SendData(...);
         - void SPI_ReceiveData(...);
         - uint8_t SPI_SendData_IT(...);
         - uint8_t SPI_ReceiveData_IT(...);
    4.4  Interrupt configuration and handling
         - void SPI_IRQInterruptConfig(...);
         - void SPI_IRQPriorityConfig(...);
         - void SPI_IRQHandling(...);
    4.5  Peripheral control functions
         - uint8_t SPI_GetFlagStatus(...);
         - void SPI_PeripheralControl(...);
         - void SPI_SSIConfig(...);
         - void SPI_SSOEConfig(...);
         - void SPI_ClearOVRFlag(...);
         - void SPI_AbortTransmission(...);
         - void SPI_AbortReception(...);
    4.6  Application callback
         - void SPI_ApplicationEventCallback(SPI_Handler_t *pSPIHandle, uint8_t AppEve);

5.  Notes
    5.1  Section on bidirectional mode explanation (BIDIMODE, BIDIOE)
    5.2  Placeholder for future error additions (TODO)

========================================================================================================================
*/



#ifndef INC_STM32F103XX_SPI_DRIVERS_H_
#define INC_STM32F103XX_SPI_DRIVERS_H_

/* =========================================================================
 * 1.Includes & Basic Macros
 * =========================================================================*/

#include"stm32f103xx.h"

typedef struct
{
	uint8_t SPI_DeviceMode;      /*Master or Slave configuration                */
 	uint8_t SPI_BusConfig;       /*Full-duplex / Half-duplex / Simplex (Rx/Tx)  */
	uint8_t SPI_Speed;           /*Baud rate control                            */
	uint8_t SPI_DFF;             /*Data frame format (8-bit / 16-bit)           */
	uint8_t SPI_CPHA;            /*Clock Phase                                  */
	uint8_t SPI_CPOL;            /*Clock Polarity                               */
	uint8_t SPI_SSM;             /*Software slave management                    */
}SPI_Config_t;

typedef struct
{
	SPI_RegDef_t *pSPIx;         /*This holds the base address of the SPIx (1/2) peripheral*/
	SPI_Config_t SPI_config;     /*This holds SPI peripheral configuration settings        */
	uint8_t      *pTxBuffer;     /*This holds the address of the Tx Buffer                 */
	uint8_t      *pRxBuffer;     /*This holds the address of the Rx Buffer                 */
	uint32_t      TxLen;         /*This holds the size of the Tx Buffer                    */
	uint32_t      RxLen;         /*This holds the size of the Rx Buffer                    */
	uint8_t       TxState;       /*This holds the state of Tx                              */
	uint8_t       RxState;       /*This holds the state of Rx                              */
}SPI_Handler_t;

/*
 * Macros for SPI config
 */

/*
 * @SPI Application states
 * */

#define SPI_READY                      0
#define SPI_BUSY_IN_RX                 1
#define SPI_BUSY_IN_TX                 2


/*
 * @Possible SPI Application events
 * */

#define SPI_EVENT_TX_CMPLT             1
#define SPI_EVENT_RX_CMPLT             2
#define SPI_EVENT_OVR_ERR              3
//TODO: Add other errors in future


/*
 * @SPI_DeviceMode
 */

#define SPI_DEVICE_MODE_SLAVE          0
#define SPI_DEVICE_MODE_MASTER         1


/*
 * @SPI_BusConfig
 */

#define SPI_BUS_CONFIG_FDPX            1
#define SPI_BUS_CONFIG_HDPX            2
//#define SPI_BUS_CONFIG_SPX_TXONLY      3
#define SPI_BUS_CONFIG_SPX_RXONLY      3


/*
 * @SPI_Speed
 */
#define SPI_CLOCK_DIV_2                0
#define SPI_CLOCK_DIV_4 			   1
#define SPI_CLOCK_DIV_8 			   2
#define SPI_CLOCK_DIV_16 			   3
#define SPI_CLOCK_DIV_32 			   4
#define SPI_CLOCK_DIV_64 			   5
#define SPI_CLOCK_DIV_128 			   6
#define SPI_CLOCK_DIV_256 			   7


/*
 * @SPI_DFF
 */

#define SPI_DFF_8BITS                  0
#define SPI_DFF_16BITS 				   1



/*
 *@SPI_CPOL
 */

#define SPI_CPOL_0_IDLE                   0
#define SPI_CPOL_1_IDLE                   1
/*
 * @SPI_CPHA
 */
#define SPI_CPHA_1ST_TRANSIT              0
#define SPI_CPHA_2ND_TRANSIT              1


/*
 * @SPI_SSM
 */

#define SPI_SSM_DI                       0
#define SPI_SSM_EN                       1

/*
 * SPI related status flag macros
 */

#define SPI_FLAG_RXNE                (1<<SPI_SR_RXNE  )
#define SPI_FLAG_TXE                 (1<<SPI_SR_TXE   )
#define SPI_FLAG_CHSIDE              (1<<SPI_SR_CHSIDE)
#define SPI_FLAG_UDR                 (1<<SPI_SR_UDR   )
#define SPI_FLAG_CRCERR              (1<<SPI_SR_CRCERR)
#define SPI_FLAG_MODF                (1<<SPI_SR_MODF  )
#define SPI_FLAG_OVR                 (1<<SPI_SR_OVR   )
#define SPI_FLAG_BSY                 (1<<SPI_SR_BSY   )





/*
 * Bit 15 BIDIMODE: Bidirectional data mode enable
0: 2-line unidirectional data mode selected       Full Duplex
1: 1-line bidirectional data mode selected        Half Duplex

Bit 14 BIDIOE: Output enable in bidirectional mode
0: Output disabled (receive-only mode)
1: Output enabled (transmit-only mode)
*/

/***********************************************************************************************************************
 *                                           API SUPPORTED BY THE DRIVER
 *                          FOR MORE INFORMATION ABOUT THE APIs CHECK THE FUNCTION DEFINITION
 ***********************************************************************************************************************/


/* GPIO peripheral clock */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t En_or_Di);

/* Init and Deinit */
void SPI_init(SPI_Handler_t *pSPIHandle);
void SPI_Deinit(SPI_RegDef_t *pSPIx);

/* Data send and receive */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *TxBuffer, uint32_t len);
void SPI_RecieveData(SPI_RegDef_t *pSPIx, uint8_t *RxBuffer, uint32_t len);
uint8_t SPI_SendData_IT(SPI_Handler_t *pSPIHandle, uint8_t *TxBuffer, uint32_t len);
uint8_t SPI_RecieveData_IT(SPI_Handler_t *pSPIHandle, uint8_t *RxBuffer, uint32_t len);


/* IQR config and ISR handling */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t En_or_Di);
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handler_t *pHandle);

/* Other Peripheral Control APIs */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t En_Or_Di);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t En_Or_Di);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t En_Or_Di);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_AbortTransmission(SPI_Handler_t *pSPIHandle);
void SPI_AbortReception(SPI_Handler_t *pSPIHandle);

/*Application event*/
 void SPI_ApplicationEventCallback(SPI_Handler_t *pSPIHandle,uint8_t AppEve);

#endif /* INC_STM32F103XX_SPI_DRIVERS_H_ */
