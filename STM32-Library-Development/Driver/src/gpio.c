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
uint8_t ConvertPortToCodeEXTI(GPIO_RegDef_t *pGPIOx){
	// return portcode, for example, GPIOA = 0; GPIOB = 1, refer chapter 7.2.4
	if(pGPIOx == GPIOA){
		return 0;
	}
	else if(pGPIOx == GPIOB){
		return 1;
	}else if(pGPIOx == GPIOC){
		return 2;
	}else if(pGPIOx == GPIOD){
		return 3;
	}else if(pGPIOx == GPIOE){
		return 4;
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
	else{ //interupt mode, code later. Focus on SYSCFG register (Chapter 7) , then go to EXTI register (chapter 10) to config Rising edge or falling edge
		//1. Config FTRS
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT){

			 EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //RESET
			 EXTI->FTSR |=  (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}
		//1. Config RTRS
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){
			 EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //RESET
			 EXTI->RTSR |=  (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FRT){
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |=  (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |=  (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |=  (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}
		//2. config the GPIO port selection SYSCFG_EXTICR, this register help the choose what EXTI such as EXTI1, EXTI2, 3,4,...
        //In this register, SYSCFG_EXTICR1 control the EXTI0,1,2,3,...
//	    uint8_t chooseEXTILine = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
//	    switch (chooseEXTILine){
//	    case 0:
//	    	SYSCFG->EXTICR[0] |= (ConvertPortToCodeEXTI(pGPIOHandle->pGPIOx) << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber * 4);
//	    	break;
//	    case 1:
//	    	SYSCFG->EXTICR[1] |= (ConvertPortToCodeEXTI(pGPIOHandle->pGPIOx) << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4) * 4);
//	    	break;
//	    case 2:
//	    	SYSCFG->EXTICR[2] |= (ConvertPortToCodeEXTI(pGPIOHandle->pGPIOx) << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4) * 4);
//	    	break;
//	    case 3:
//	    	SYSCFG->EXTICR[3] |= (ConvertPortToCodeEXTI(pGPIOHandle->pGPIOx) << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4) * 4);
//	    	break;
//
//	    }
		uint8_t chooseEXTIReg = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4; // choose which EXTICR register is used 0,1,2,3
		uint8_t chooseEXTILine = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;

	    SYSCFG->EXTICR[chooseEXTIReg] |= (ConvertPortToCodeEXTI(pGPIOHandle->pGPIOx) << chooseEXTILine * 4);
		//3. enable interupt using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

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

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){ //each register control 32 IRQ number => only use 0 1 2 in stm32f4
	void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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


}
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
    uint8_t iprx = IRQNumber / 4;
    uint8_t iprx_section = IRQNumber % 4;

    uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
    *(NVIC_PR_BASE_ADDR + (iprx * 4)) |= (IRQPriority << shift_amount);
}
/*
 * Trigger Event
An event such as a rising edge or falling edge occurs on a GPIO pin configured as an external interrupt (EXTI).

EXTI Detects the Interrupt & Sets the PR Bit
If an interrupt event occurs, the corresponding bit in the EXTI->PR (Pending Register) is set to 1.
For example, if an interrupt occurs on GPIO5, then bit 5 in EXTI->PR will be set (EXTI->PR |= (1 << 5);).

NVIC Recognizes the Interrupt & Calls the ISR (Interrupt Service Routine)
When a bit in EXTI->PR is set, the NVIC (Nested Vectored Interrupt Controller) checks the interrupt vector table and calls the corresponding ISR (Interrupt Service Routine).
In this case, it calls EXTI9_5_IRQHandler(), which handles interrupts for EXTI5 to EXTI9.
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
    /* Clear the PR register corresponding to pin number */
    if(EXTI->PR & (1 << PinNumber))
    {
        /* Clear pin */
        EXTI->PR |= (1 << PinNumber);
    }
}

#endif /* SRC_GPIO_C_ */
