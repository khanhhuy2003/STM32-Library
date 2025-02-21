#include <string.h>
#include "stm32f401xx.h"
#include "gpio.h"
#include "spi.h"
#include "stddef.h"

#define BTN_PRESSED 0

void delay(void)
{
    for (uint32_t i = 0; i < 500000; i++);
}

// Declare SPI Handle Globally
SPI_Handle_t SPIHandle;

void SPI2_GPIOInits(void)
{
    GPIO_Handle_t SPIPins;
    SPIPins.pGPIOx = GPIOB;
    SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
    SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPD;
    SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

    /* SCLK Init */
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    GPIO_Init(&SPIPins);

    /* MOSI Init */
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
    GPIO_Init(&SPIPins);

    /* MISO Init */
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
    GPIO_Init(&SPIPins);

    /* NSS Init (for Hardware NSS) */
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    GPIO_Init(&SPIPins);
}

void SPI2_Inits()
{
    SPIHandle.pSPIx = SPI2;
    SPIHandle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
    SPIHandle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
    SPIHandle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV16; // Reduce Speed for Debugging
    SPIHandle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
    SPIHandle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
    SPIHandle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
    SPIHandle.SPIConfig.SPI_SSM = SPI_SSM_DI;  // Enable Hardware NSS

    SPI_Init(&SPIHandle);
    SPI_SSOEConfig(SPI2, ENABLE);  // Enable Automatic NSS Control
}

int main(void)
{
    char user_data[] = "HUY";

    /* Initialize GPIO pins to behave as SPI2 pins */
    SPI2_GPIOInits();

    /* Initialize SPI2 peripheral parameters */
    SPI2_Inits();

    /* Enable SPI */
    SPI_PeripheralControl(SPI2, ENABLE);

    /* Enable SPI Interrupt */
    SPI_IRQInterruptConfig(IRQ_NO_SPI2, ENABLE);

    while (1)
    {
        // Send data in Interrupt Mode
    	GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_12, SET);
        if (SPIHandle.TxState != SPI_BUSY_IN_TX)
        {
            SPI_SendDataInterruptMode(&SPIHandle, (uint8_t*)user_data, strlen(user_data));
        }

        // Wait until SPI transmission is complete
        while (SPIHandle.TxState == SPI_BUSY_IN_TX);
        GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_12, RESET);
        // Small delay
        delay();
    }

    return 0;
}

/* SPI2 Interrupt Handler */
void SPI2_IRQHandler(void)
{
    SPI_IRQHandling(&SPIHandle);
}
