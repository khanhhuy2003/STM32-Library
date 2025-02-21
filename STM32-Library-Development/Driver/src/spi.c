/*
 * spi.c
 *
 *  Created on: Feb 10, 2025
 *      Author: ASUS
 */
#include "stm32f401xx.h"
#include "spi.h"
#include "stddef.h"
/*
 * Configuration structure for SPIx peripheral
 */
static void SPI_TXE_InterruptHandle(SPI_Handle_t*pHandle);
static void SPI_RXE_InterruptHandle(SPI_Handle_t*pHandle);
static void SPI_OVR_ErrInterruptHandle();
static void SPI_OVR_ErrInterruptHandle(SPI_Handle_t *pSPIHandle);
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
    if(EnorDi == ENABLE)
    {
        if(pSPIx == SPI1)
        {
            SPI1_PCLK_EN();
        }
        else if(pSPIx == SPI2)
        {
            SPI2_PCLK_EN();
        }
        else if(pSPIx == SPI3)
        {
            SPI3_PCLK_EN();
        }
        else if(pSPIx == SPI4)
        {
            SPI4_PCLK_EN();
        }
    }
    else
    {
        if(pSPIx == SPI1)
        {
            SPI1_PCLK_DI();
        }
        else if(pSPIx == SPI2)
        {
            SPI2_PCLK_DI();
        }
        else if(pSPIx == SPI3)
        {
            SPI3_PCLK_DI();
        }
        else if(pSPIx == SPI4)
        {
            SPI4_PCLK_DI();
        }
    }
}



/*
* Init and De-Init
* Some important register in SPI is CR1 (Control register),CR2 (For interupt mode), SR(status register), DR(Data register)
*
*/

void SPI_Init(SPI_Handle_t *pSPIHandle){
	SPI_PeriClockControl(pSPIHandle -> pSPIx, ENABLE);
	uint32_t tempreg = 0;
	//1. Config master and slave mode in bit 2 MSTR. Every device is set up as slave in default
	if(pSPIHandle->SPIConfig.SPI_DeviceMode == SPI_DEVICE_MODE_MASTER){
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;
	}
	//2. Config the bus Full, half or simplex
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD){
		//if full duplex, set 0 (clear) for bit 15 BIDIMODE
		tempreg &= ~(1 << 15);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD){
		//if half duplex, set 1 for bit 15 BIDIMODE
		tempreg |= (1 << 15);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RX_ONLY){
		//if simplex, we need to set bit 10 RXONLY is 1 to force the master produce clock.
		//Because in the mode RXONLY, master will receive data from slave.
		tempreg &= ~(1 << 15);
        if (pSPIHandle->SPIConfig.SPI_DeviceMode == SPI_DEVICE_MODE_MASTER) {
            tempreg |= (1 << 10);  // RXONLY
        }
	}
	//3. Clock speed
	tempreg |= (pSPIHandle->SPIConfig.SPI_SclkSpeed << 3);

	//4. DFF (8 or 16 bit)
	tempreg |= (pSPIHandle->SPIConfig.SPI_DFF << 11);

	//5. CPOL
	tempreg |= (pSPIHandle->SPIConfig.SPI_CPOL << 1);

	//6. CPHA
	tempreg |= (pSPIHandle->SPIConfig.SPI_CPHA << 0);

	pSPIHandle ->pSPIx->CR1 = tempreg;

	pSPIHandle->pSPIx->CR1 |= (1 << 6);  // Config bit 6 SPE

}
void SPI_DeInit(SPI_RegDef_t *pSPIx){

}


/*
* Data send and receive
*/
uint8_t checkFlagBufferTx(SPI_RegDef_t * pSPIx){
	if(pSPIx->SR & (1 << 1)){ //because Tx is bit 1, so need to shift right 1 bit
		return 1; // empty
	}
	return 0; //not empty
}
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Length){
	//Before transmit data, check the Tx buffer is empty or not
	//if empty, exit the send and finish and can get data outside to continue transmit data
	//if not, that means the buffer still exists data and need to send. in that time, no data should be put in that Tx buffer
	//to avoid corrupting data with the existing data.
	//These information can check at SR (status register).
	while(Length > 0){
		while(checkFlagBufferTx(pSPIx) == (uint8_t)RESET); //stuck at this loop if it is 0. that mean not empty
		if( pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			/* Load data into data register */
			/* 16 bit */
		    pSPIx->DR = *((uint16_t*)pTxBuffer);
		    pTxBuffer += 2;
		    Length -= 2;
		}
		else
		{
			/* 8 bit */
			pSPIx->DR = *pTxBuffer;
	        pTxBuffer++;
	        Length--;
		}
	}

}
uint8_t checkFlagBufferRx(SPI_RegDef_t * pSPIx){
	if(pSPIx->SR & 0x01){ //bit 0, so do not need to shift right
		return 1; //not empty
	}
	return 0; //empty
}
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Length){
	while(Length > 0){
		while(checkFlagBufferRx(pSPIx) == (uint8_t)RESET); // empty, waiting for data
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF)){
			//16 bits
			if(Length >= 2){
				*((uint16_t*)pRxBuffer) = pSPIx->DR;
				Length -= 2;
				pRxBuffer += 2;
			}
			else{
				*pRxBuffer =(uint8_t)pSPIx->DR;
				Length--;
				pRxBuffer++;

			}
		}
		else{
			*pRxBuffer = (uint8_t)pSPIx->DR;
			Length--;
			pRxBuffer++;
		}
	}
}


/*
* Data send and receive in Interrupt mode
*/
uint8_t SPI_SendDataInterruptMode(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Length){
    uint8_t state = pSPIHandle->TxState;

    if(state != SPI_BUSY_IN_TX)
    {
        /* Save Tx buffer address and length information in global variables */
        pSPIHandle->pTxBuffer = pTxBuffer;
        pSPIHandle->TxLen = Length;

        /* Mark SPI state as busy so that no other code can take over SPI peripheral until transmission is over */
        pSPIHandle->TxState = SPI_BUSY_IN_TX;
        /* Enable TXEIE control bit to get interrupt whenever TXE flag is set in SR */
        pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
    }
    return state;
}
uint8_t SPI_ReceiveDataInterruptMode(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Length)
{
    uint8_t state = pSPIHandle->RxState;

    if(state != SPI_BUSY_IN_RX)
    {
        /* Save Rx buffer address and length information in global variables */
        pSPIHandle->pRxBuffer = pRxBuffer;
        pSPIHandle->RxLen = Length;

        /* Mark SPI state as busy so that no other code can take over SPI peripheral until transmission is over */
        pSPIHandle->RxState = SPI_BUSY_IN_RX;

        /* Enable RXNEIE control bit to get interrupt whenever RXE flag is set in SR */
        pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
    }

    return state;
}


/*
* IRQ Configuration and ISR handling
*/
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
    if(EnorDi == ENABLE)
    {
        if(IRQNumber <= 31)
        {
            /* Program ISER0 register */
            *NVIC_ISER0 |= (1 << IRQNumber);
        }
        else if(IRQNumber > 31 && IRQNumber < 64)
        {
            /* Program ISER1 register (32 to 63) */
            *NVIC_ISER1 |= (1 << (IRQNumber % 32));
        }
        else if(IRQNumber >= 64 && IRQNumber < 96)
        {
            /* Program ISER2 register (64 to 95) */
            *NVIC_ISER2 |= (1 << (IRQNumber % 64));
        }
    }
    else
    {
        if(IRQNumber <= 31)
        {
            /* Program ICER0 register */
            *NVIC_ICER0 |= (1 << IRQNumber);
        }
        else if(IRQNumber > 31 && IRQNumber < 64)
        {
            /* Program ICER1 register (32 to 63) */
            *NVIC_ICER1 |= (1 << (IRQNumber % 32));
        }
        else if(IRQNumber >= 64 && IRQNumber < 96)
        {
            /* Program ICER2 register (64 to 95) */
            *NVIC_ICER2 |= (1 << (IRQNumber % 64));
        }
    }
}
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle){
	//When enter the ISR, we need to check which event trigger the interupt: Tx, Rx, Error by check the SR reg
	//When an SPI event occurs (e.g., transmit buffer empty (TXE) or receive buffer not empty (RXNE)),
	//the MCU automatically triggers an interrupt.

	//An overrun error (OVR error) occurs in SPI when new incoming data overwrites unread data in the receive buffer (SPI_DR) before the MCU reads it.
	//This means the MCU is not reading data fast enough, causing lost data. => overwrite data

	//An underrun error happens when the SPI Transmit (TX) buffer is empty but SPI is still expected to send data.
	//This issue occurs mostly in SPI Slave mode, where the slave must always have data ready when the master requests it.
	// => sending but the buffer is empty

	//For sending data
	uint8_t temp1;
	uint8_t temp2;

	temp1 = pHandle -> pSPIx->SR & (1 << 1);
	temp2 = pHandle -> pSPIx->CR2 & (1 << 7);

	if( temp1 && temp2 ){
		SPI_TXE_InterruptHandle(pHandle);
	}
	temp1 = pHandle -> pSPIx->SR & (1 << 0);
	temp2 = pHandle -> pSPIx->CR2 & (1 << 6);

	if( temp1 && temp2 ){
		SPI_RXNE_InterruptHandle(pHandle);
	}

	temp1 = pHandle -> pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pHandle -> pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

	if( temp1 && temp2 ){
		 SPI_OVR_ErrInterruptHandle(pHandle	);
	}

}

/*
* Other peripheral APIs
*/
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
    if(pSPIx->SR & FlagName)
    {
        return FLAG_SET;
    }
    return FLAG_RESET;
}
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
    if(EnorDi == ENABLE)
    {
        pSPIx->CR1 |= (1 << SPI_CR1_SPE);
    }
    else
    {
        pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
    }
}
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
    if(EnorDi == ENABLE)
    {
        pSPIx->CR1 |= (1 << SPI_CR1_SSI);
    }
    else
    {
        pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
    }
}

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
    if(EnorDi == ENABLE)
    {
        pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
    }
    else
    {
        pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
    }
}
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
    uint8_t temp;
    temp = pSPIx->DR;
    temp = pSPIx->SR;
    (void)temp;
}
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
    pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
    pSPIHandle->pTxBuffer = NULL;
    pSPIHandle->TxLen = 0;
    pSPIHandle->TxState = SPI_READY;
}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
    pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
    pSPIHandle->pRxBuffer = NULL;
    pSPIHandle->RxLen = 0;
    pSPIHandle->RxState = SPI_READY;
}
void SPI_TXE_InterruptHandle(SPI_Handle_t *pSPIHandle)
{
    if( pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF) )
    {
        /* Load data into data register */
        /* 16 bit */
        pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
        pSPIHandle->pTxBuffer = (uint8_t*)((uint16_t*)pSPIHandle->pTxBuffer + 1);
        pSPIHandle->TxLen -= 2;

    }
    else
    {
        /* 8 bit */
        pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
        pSPIHandle->pTxBuffer++;
        pSPIHandle->TxLen--;

    }

    if(!pSPIHandle->TxLen)
    {
        /* Tx is zero. Close SPI communication and inform application about it.
         * Prevents interrupts from setting up of TXE flag. */
        SPI_CloseTransmission(pSPIHandle);
        SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
    }
}
void SPI_RXNE_InterruptHandle(SPI_Handle_t *pSPIHandle)
{
    if( pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF) )
    {
        /* Load data from data register into buffer */
        /* 16 bit */
        *((uint16_t*)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
        pSPIHandle->pRxBuffer = (uint8_t*)((uint16_t*)pSPIHandle->pRxBuffer + 1);
        pSPIHandle->RxLen -= 2;
    }
    else
    {
        /* 8 bit */
        *pSPIHandle->pRxBuffer = (uint8_t)pSPIHandle->pSPIx->DR;
        pSPIHandle->pRxBuffer++;
        pSPIHandle->RxLen--;
    }

    if(!pSPIHandle->RxLen)
    {
        /* Rx is zero. Close SPI communication and inform application about it.
         * Prevents interrupts from setting up of RXNE flag. */
        SPI_CloseReception(pSPIHandle);
        SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
    }
}

void SPI_OVR_ErrInterruptHandle(SPI_Handle_t *pSPIHandle)
{
    uint8_t temp;

    /* Clear OVR flag */
    if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
    {
        temp = pSPIHandle->pSPIx->DR;
        temp = pSPIHandle->pSPIx->SR;
    }
    (void)temp;

    /* Inform application */
    SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}
/*
* Application callback
*/
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEvent){

}
