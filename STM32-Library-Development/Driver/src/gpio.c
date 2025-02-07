/*
 * gpio.c
 *
 *  Created on: Feb 5, 2025
 *      Author: ASUS
 */

#ifndef SRC_GPIO_C_
#define SRC_GPIO_C_

#include "gpio.h"

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
    if(EnorDi == ENABLE)
    {
        if(pGPIOx == GPIOA)
        {
            GPIOA_PCLK_EN();
        }
        else if(pGPIOx == GPIOB)
        {
            GPIOB_PCLK_EN();
        }
        else if(pGPIOx == GPIOC)
        {
            GPIOC_PCLK_EN();
        }
        else if(pGPIOx == GPIOD)
        {
            GPIOD_PCLK_EN();
        }
        else if(pGPIOx == GPIOE)
        {
            GPIOE_PCLK_EN();
        }
    }
    else
    {
        if(pGPIOx == GPIOA)
        {
            GPIOA_PCLK_DI();
        }
        else if(pGPIOx == GPIOB)
        {
            GPIOB_PCLK_DI();
        }
        else if(pGPIOx == GPIOC)
        {
            GPIOC_PCLK_DI();
        }
        else if(pGPIOx == GPIOD)
        {
            GPIOD_PCLK_DI();
        }
        else if(pGPIOx == GPIOE)
        {
            GPIOE_PCLK_DI();
        }
    }
}
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){
	uint32_t temp = 0;
	//1. config mode of gpio pin

	GPIO_PeriClockControl(pGPIOHandle ->pGPIOx, ENABLE);
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){ //not interupt mode
		temp |= pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER |= temp;
		temp = 0;
	}
	else{ //interupt mode, code later
	}

	//2. config the speed
	temp |= pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;

	//3. config pudp setting
	temp |= pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;

	//4. config outputtype
	temp |= pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->ODR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->ODR |= temp;
	temp = 0;

	//5. config alternate function
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber < 8){
			pGPIOHandle->pGPIOx->AFR[0] &= ~(0xF << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber * 4));
			pGPIOHandle->pGPIOx->AFR[0] |= pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else{
			pGPIOHandle->pGPIOx->AFR[1]	&= ~(0xF << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber - 8) * 4);
			pGPIOHandle->pGPIOx->AFR[1] |= pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber - 8));

		}
	}

}
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
    if(pGPIOx == GPIOA)
    {
        GPIOA_REG_RESET();
    }
    else if(pGPIOx == GPIOB)
    {
        GPIOB_REG_RESET();
    }
    else if(pGPIOx == GPIOC)
    {
        GPIOC_REG_RESET();
    }
    else if(pGPIOx == GPIOD)
    {
        GPIOD_REG_RESET();
    }
    else if(pGPIOx == GPIOE)
    {
        GPIOE_REG_RESET();
    }
}

/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	uint8_t value = 0;
	value =(uint8_t)((pGPIOx->IDR >> PinNumber) & 0x1);
	return value;
}
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){
    uint16_t value;

    value = (uint16_t )pGPIOx->IDR;

    return value;
}
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value){
	if(Value == GPIO_PIN_SET){
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else{
		pGPIOx->ODR &= ~( 1 << PinNumber );
	}
}
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value){
	pGPIOx->ODR = Value;
}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	pGPIOx->ODR ^= (1 << PinNumber);
}




#endif /* SRC_GPIO_C_ */
