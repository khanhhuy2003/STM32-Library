/*
 * i2c.c
 *
 *  Created on: Feb 20, 2025
 *      Author: ASUS
 */

#include "i2c.h"


/*
 * Peripheral Clock setup
 */
uint8_t AHB_PreScaler[8] = {2,4,8,16,32,64,128,256,512};
uint8_t APB1_PreScaler[4] = {2,4,8,16};
uint8_t APB2_PreScaler[4] = {2,4,8,16};
uint32_t RCC_GetPCLK1Value(){
	uint32_t pclk1, Sysclk, ahb_prescaler, apb1_prescaler, apb2_prescaler, temp;

	uint8_t clksrc = ((RCC -> CFGR) >> 2) & 0x3; //check the clock src belong to HSI, HSE or PLL

	if(clksrc == 0){ // HSI
		Sysclk = 16000000;

	}else if(clksrc == 1){ //HSE
		Sysclk = 8000000;
	}

	temp |= (((RCC->CFGR) >> 4) & 0xF);
	if(temp < 8){
		ahb_prescaler /= 1;
	}
	else if(temp > 8){
		ahb_prescaler = AHB_PreScaler[temp - 8];
	}

	temp |= (((RCC->CFGR) >> 10) & 0x3);
	if(temp < 4){
		ahb_prescaler /= 1;
	}
	else if(temp > 4){
		ahb_prescaler = AHB_PreScaler[temp - 4];
	}

	pclk1 = (Sysclk / ahb_prescaler) / apb1_prescaler;
	return pclk1;


}
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(pI2Cx == I2C1){
			I2C1_PCLK_EN();
		}
		else if(pI2Cx == I2C2){
			I2C2_PCLK_EN();
		}
		else if(pI2Cx == I2C3){
			I2C3_PCLK_EN();
		}
	}
	else {
		if(pI2Cx == I2C1){
			I2C1_PCLK_DI();
		}
		else if(pI2Cx == I2C2){
			I2C2_PCLK_DI();
		}
		else if(pI2Cx == I2C3){
			I2C3_PCLK_DI();
		}
	}

}
/*
 * Init and De-Init
 */
void I2C_Init(I2C_Handle_t *pI2CHandle){

	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);
	//config ack CR1
	uint32_t temp = 0;
	uint32_t ccr_value;
	temp |= (pI2CHandle -> I2C_Config.I2C_ACKControl << 10);
	pI2CHandle ->pI2Cx->CR1 |= temp ;

	//config the FREQ of CR2
	temp = 0;
	uint32_t temp_freq = RCC_GetPCLK1Value() / 1000000;
	pI2CHandle ->pI2Cx->CR2 = temp & 0x3F; //first 7 bits

	//config device address OAR1
	temp = 0;
	temp = pI2CHandle -> I2C_Config.I2C_DeviceAddress << 1; //because the first bit is don't care
	temp = (1 << 14); //the reference manual force to do this
	pI2CHandle ->pI2Cx->OAR1 |= temp;

	temp = 0;
	//formula: Tsclk = coef * CCR * Tpclk1 => fpclk1 = coef * CCR *fsclk
	if(pI2CHandle ->I2C_Config.I2C_SCLSpeed == I2C_SCL_SPEED_SM){
		//coef = 2 because duty cycle is 50 50
		temp |= (0 << 15);
		ccr_value = RCC_GetPCLK1Value() / (2 * (pI2CHandle -> I2C_Config.I2C_SCLSpeed));
		temp |= ccr_value & 0xFFF;
	}
	else{
		temp |= (1 << 15);
		temp |= pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14;
		if(pI2CHandle ->I2C_Config.I2C_SCLSpeed == I2C_SCL_SPEED_FM2K){
			//coef = 3 because duty cycle 2 * thigh = tlow
			ccr_value = RCC_GetPCLK1Value() / (3 * (pI2CHandle -> I2C_Config.I2C_SCLSpeed));
		}
		else if(pI2CHandle ->I2C_Config.I2C_SCLSpeed == I2C_SCL_SPEED_FM4K){
			//coef = 3 because duty cycle 16 * thigh = 9 * tlow
			ccr_value = RCC_GetPCLK1Value() / (25 * (pI2CHandle -> I2C_Config.I2C_SCLSpeed));
		}
		temp |= ccr_value & 0xFFF;
 }

	pI2CHandle->pI2Cx->CCR = temp;
    temp = 0;
    if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
    {
        /* Standard mode */
        temp = (RCC_GetPCLK1Value() / 1000000U) + 1;
    }
    else
    {
        /* Fast mode */
        temp = ( (RCC_GetPCLK1Value() * 300) / 1000000U ) + 1;
    }

    pI2CHandle->pI2Cx->TRISE = (temp & 0x3F);

}
void I2C_DeInit(I2C_RegDef_t *pI2Cx);


/*
 * Data send and receive
 */
void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx){
	pI2Cx->CR1 |= (1 << 8);

}
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx){
	pI2Cx->CR1 |= (1 << 9);
}
void I2C_ExecuteAddressPhase(I2C_RegDef_t* pI2Cx, uint8_t addr){
	addr = addr << 1;
	addr &= ~(1); // make space for write/read bit
	pI2Cx->DR = addr; // in i2c, slave address only use 7 high bits, the lowest bit is read/write bit
}
static void I2C_ClearAddrFlag(I2C_Handle_t *pI2CHandle)
{
    uint32_t dummy_read;

    /* Checking for device mode */
    if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
    {
        /* Device is in master mode */
        if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
        {
            if(pI2CHandle->RxSize == 1)
            {
                /* Disabling ACK */
                I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

                /* Clearing ADDR flag, reading SR1 and SR2 */
                dummy_read = pI2CHandle->pI2Cx->SR1;
                dummy_read = pI2CHandle->pI2Cx->SR2;
                (void)dummy_read;
            }
        }
        else
        {
            /* Clearing ADDR flag, reading SR1 and SR2 */
            dummy_read = pI2CHandle->pI2Cx->SR1;
            dummy_read = pI2CHandle->pI2Cx->SR2;
            (void)dummy_read;
        }
    }
    else
    {
        /* Device is in slave mode */
        /* Clearing ADDR flag, reading SR1 and SR2 */
        dummy_read = pI2CHandle->pI2Cx->SR1;
        dummy_read = pI2CHandle->pI2Cx->SR2;
        (void)dummy_read;
    }
}
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Length, uint8_t SlaveAddr, uint8_t Sr){
	//1. Start condition
	I2C_GenerateStartCondition(pI2CHandle ->pI2Cx);

	//2. confirm start condition by check the SB flag
	//if the SB == 0, it mean the start condition is not confirmed on the bus, the SCL is streched
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	//3. Send address with 1 bit write or read (total is 8 bits)
	I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, SlaveAddr);

	//4. Confirm the address phase is completed by check the ADD flag in SR1
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR)); //O is failed, 1 is successful

    /* Clearing the address flag according to its software sequence */
    I2C_ClearAddrFlag(pI2CHandle);

	//5. Send data
	while(Length > 0){
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
		pI2CHandle -> pI2Cx->DR = *pTxBuffer;
		Length--;
		pTxBuffer++;
	}

    //6. Waiting for TXE=1 and BTF=1 before generating STOP condition
	//7. if the TxE and BFT == 1, that mean both DR and SR are empty and ready for new data, now SCL is stretched
    while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE) );
    while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF) );

    //8. Stop condition

    I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

}
//When master receive data, there are 2 case
//1. One byte: if one byte, master send nack to slave and after that send stop condition, finally read data in data register
//2. More than 1 byte: need to wait for the last 2 byte. when read the second last byte, send nack, after read the last byte, send stop condition
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Length, uint8_t SlaveAddr, uint8_t Sr){
	I2C_GenerateStartCondition(pI2CHandle ->pI2Cx);

	//2. confirm start condition by check the SB flag
	//if the SB == 0, it mean the start condition is not confirmed on the bus, the SCL is streched
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	//3. Send address with 1 bit write or read (total is 8 bits)
	I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, SlaveAddr);

	//4. Confirm the address phase is completed by check the ADD flag in SR1
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR)); //O is failed, 1 is successful

    /* Clearing the address flag according to its software sequence */
    I2C_ClearAddrFlag(pI2CHandle);

	if(Length == 1){
		I2C_ManageAcking(pI2CHandle ->pI2Cx, I2C_ACK_DISABLE);

		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

		I2C_GenerateStopCondition(pI2CHandle ->pI2Cx);

        if(Sr == I2C_DISABLE_SR)
        {
            I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
        }

        // Read the data into buffer
        *pRxBuffer = pI2CHandle->pI2Cx->DR;

	}
	else if(Length > 1){
		for(uint32_t i = Length; i > 0; i--){
			while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

			if(i == 2){
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
                if(Sr == I2C_DISABLE_SR)
                {
                    I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
                }
			}

            // Read the data from data register in to the buffer
            *pRxBuffer = pI2CHandle->pI2Cx->DR;
            pRxBuffer++;
		}

	}
    // Re-enable ACK
    if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
    {
        I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
    }
}
void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data)
{
    pI2Cx->DR = data;
}
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx)
{
    return (uint8_t)pI2Cx->DR;
}


/*
 * Data send and receive in Interrupt mode
 */
uint8_t I2C_MasterSendDataInterruptMode(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Length, uint8_t SlaveAddr, uint8_t Sr)
{
    uint8_t busystate = pI2CHandle->TxRxState;

    if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
    {
        pI2CHandle->pTxBuffer = pTxBuffer;
        pI2CHandle->TxLen = Length;
        pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
        pI2CHandle->DevAddr = SlaveAddr;
        pI2CHandle->Sr = Sr;

        /* Generating start condition */
        I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

        /* Enable ITBUFEN Control Bit */
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

        /* Enable ITEVTEN Control Bit */
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

        /* Enable ITERREN Control Bit */
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
    }

    return busystate;
}

uint8_t I2C_MasterReceiveDataInterruptMode(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Length, uint8_t SlaveAddr, uint8_t Sr)
{

    uint8_t busystate = pI2CHandle->TxRxState;

    if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
    {
        pI2CHandle->pRxBuffer = pRxBuffer;
        pI2CHandle->RxLen = Length;
        pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
        /* RxSize is used in the ISR code to manage the data reception */
        pI2CHandle->RxSize = Length;
        pI2CHandle->DevAddr = SlaveAddr;
        pI2CHandle->Sr = Sr;

        /* Generating start condition */
        I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

        /* Enable ITBUFEN Control Bit */
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

        /* Enable ITEVTEN Control Bit */
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

        /* Enable ITERREN Control Bit */
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
    }

    return busystate;
}


void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
    /* Disabling ITBUFEN Control Bit */
    pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

    /* Disabling ITEVFEN Control Bit */
    pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);


    pI2CHandle->TxRxState = I2C_READY;
    pI2CHandle->pTxBuffer = NULL;
    pI2CHandle->TxLen = 0;
}
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
    /* Disabling ITBUFEN Control Bit */
    pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

    /* Disabling ITEVFEN Control Bit */
    pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

    pI2CHandle->TxRxState = I2C_READY;
    pI2CHandle->pRxBuffer = NULL;
    pI2CHandle->RxLen = 0;
    pI2CHandle->RxSize = 0;

    if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
    {
        I2C_ManageAcking(pI2CHandle->pI2Cx,ENABLE);
    }

}


/*
 * IRQ Configuration and ISR handling
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
            *NVIC_ISER0 |= (1 << IRQNumber);
        }
        else if(IRQNumber > 31 && IRQNumber < 64)
        {
            /* Program ICER1 register (32 to 63) */
            *NVIC_ISER1 |= (1 << (IRQNumber % 32));
        }
        else if(IRQNumber >= 64 && IRQNumber < 96)
        {
            /* Program ICER2 register (64 to 95) */
            *NVIC_ISER2 |= (1 << (IRQNumber % 64));
        }
    }
}
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
    uint8_t iprx = IRQNumber / 4;
    uint8_t iprx_section = IRQNumber % 4;

    uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
    *(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}
void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle)
{
    if(pI2CHandle->RxSize == 1)
    {
        *pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
        pI2CHandle->RxLen--;
    }

    if(pI2CHandle->RxSize > 1)
    {
        if(pI2CHandle->RxLen == 2)
        {
            /* Clearing ACK bit */
            I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
        }

        /* Reading DR */
        *pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
        pI2CHandle->pRxBuffer++;
        pI2CHandle->RxLen--;
    }

    if(pI2CHandle->RxLen == 0)
    {
        /* Closing I2C data reception and notifying application */

        /* Generating stop condition */
        if(pI2CHandle->Sr == I2C_DISABLE_SR)
        {
            I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
        }

        /* Closing I2C RX */
        I2C_CloseReceiveData(pI2CHandle);

        /* Notify application about transmission complete */
        I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
    }
}
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle)
{
    /* TXE flag is set */
    if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
    {
        if(pI2CHandle->TxLen > 0)
        {
            /* Loading data into DR */
            pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);

            /* Decrement Tx Length */
            pI2CHandle->TxLen--;

            /* Increment Tx buffer address */
            pI2CHandle->pTxBuffer++;
        }
    }
    /* DBG->Check comments */
}
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
    /* Interrupt handling for both master and slave mode of a device */
    uint32_t temp1, temp2, temp3;

    temp1 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);
    temp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);
    temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB);

    /* Handling for interrupt generated by SB event */
    if(temp1 && temp3)
    {
        if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
        {
            I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
        }
        else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
        {
            I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
        }
    }

    temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);

    /* Handling for interrupt generated by ADDR event */
    if(temp1 && temp3)
    {
        I2C_ClearAddrFlag(pI2CHandle);
    }

    temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);

    /* Handling for interrupt generated by BTF(Byte Transfer Finished) event */
    if(temp1 && temp3)
    {
        /* BTF flag is set */
        if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
        {
            /* TXE flag is also set */
            if(pI2CHandle->pI2Cx->SR1 && (1 << I2C_SR1_TXE))
            {
                if(pI2CHandle->TxLen == 0)
                {
                    /* Generate STOP condition */
                    if(pI2CHandle->Sr == I2C_DISABLE_SR)
                    {
                        I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
                    }

                    /* Reset all member elements of the handle structure */
                    I2C_CloseSendData(pI2CHandle);

                    /* Notify application about transmission complete */
                    I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
                }
            }
        }
        else if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
        {
            ;
        }
    }

    temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF);

    /* Handling for interrupt generated by STOPF event */
    if(temp1 && temp3)
    {
        /* Clear STOPF flag */
        pI2CHandle->pI2Cx->CR1 |= 0x0000;

        /* STOPF generated by master */
        I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
    }

    temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE);

    /* Handling for interrupt generated by TXE event */
    if(temp1 && temp2 && temp3)
    {
        /* Checking device mode */
        if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
        {
            if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
            {
                I2C_MasterHandleTXEInterrupt(pI2CHandle);
            }
        }
        else
        {
            /* Checking if slave is really in Tx mode */
            if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA))
            {
                I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_DATA_REQ);
            }
        }
    }

    temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE);

    /* Handling for interrupt generated by RXNE event */
    if(temp1 && temp2 && temp3)
    {
        /* Checking device mode */
        if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
        {
            /* RXNE flag is set */
            if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
            {
                I2C_MasterHandleRXNEInterrupt(pI2CHandle);
            }
        }
        else
        {
            /* Checking if slave is really in Rx mode */
            if(!(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA)))
            {
                I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_DATA_RCV);
            }
        }
    }
}
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{
    uint32_t temp1, temp2;

    /* Status of ITERREN control bit in CR2 */
    temp2 = (pI2CHandle->pI2Cx->CR2) & (1 << I2C_CR2_ITERREN);

    /* Checking for bus error */
    temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_BERR);
    if(temp1 && temp2)
    {
        /* Clear the bus error flag */
        pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_BERR);

        /* Notify application about error */
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_BERR);
    }

    /* Checking for arbitration lost error */
    temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_ARLO);
    if(temp1 && temp2)
    {
        /* Clear the arbitration lost error flag */
        pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_ARLO);

        /* Notify application about error */
        I2C_ApplicationEventCallback(pI2CHandle, I2C_SR1_ARLO);
    }

    /* Checking for ACK failure error */
    temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_AF);
    if(temp1 && temp2)
    {
        /* Clear the ACK failure error flag */
        pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_AF);

        /* Notify application about error */
        I2C_ApplicationEventCallback(pI2CHandle, I2C_SR1_AF);
    }

    /* Checking for Overrun/Underrun error */
    temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_OVR);
    if(temp1 && temp2)
    {
        /* Clear the Overrun/Underrun error flag */
        pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_OVR);

        /* Notify application about error */
        I2C_ApplicationEventCallback(pI2CHandle, I2C_SR1_OVR);
    }

    /* Checking for Time out error */
    temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_TIMEOUT);
    if(temp1 && temp2)
    {
        /* Clear the Time out error flag */
        pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_TIMEOUT);

        /* Notify application about error */
        I2C_ApplicationEventCallback(pI2CHandle, I2C_SR1_TIMEOUT);
    }
}


/*
 * Other peripheral APIs
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName){
	if(pI2Cx->SR1 & FlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
    if(EnorDi == ENABLE)
    {
        pI2Cx->CR1 |= (1 << I2C_CR1_PE);
    }
    else
    {
        pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
    }
}
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
    if(EnorDi == I2C_ACK_ENABLE)
    {
        /* Enable ACK */
        pI2Cx->CR1 |= ( 1 << I2C_CR1_ACK);
    }
    else
    {
        /* Disable ACK */
        pI2Cx->CR1 &= ~( 1 << I2C_CR1_ACK);
    }
}
/*
 * Application callback
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEvent);

