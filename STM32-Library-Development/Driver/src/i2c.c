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

}
void I2C_DeInit(I2C_RegDef_t *pI2Cx);


/*
 * Data send and receive
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Length, uint8_t SlaveAddr, uint8_t Sr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Length, uint8_t SlaveAddr, uint8_t Sr);
void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx);


/*
 * Data send and receive in Interrupt mode
 */
uint8_t I2C_MasterSendDataInterruptMode(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Length, uint8_t SlaveAddr, uint8_t Sr);
uint8_t I2C_MasterReceiveDataInterruptMode(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Length, uint8_t SlaveAddr, uint8_t Sr);

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);


/*
 * IRQ Configuration and ISR handling
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);


/*
 * Other peripheral APIs
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);


/*
 * Application callback
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEvent);

