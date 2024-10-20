#include "i2c.h"


/*
PB8 : I2C1 SCL
PB9 : I2C1 SDA
*/


#define GPIOBEN				(1U<<1)
#define I2C1EN				(1U<<21)
#define CR1_SWRST			(1U<<15)
#define CR1_NOSTRETCH  		(1U<<7)
#define CR1_ENGC			(1U<<6)
#define CR2_DMAEN			(1U<<11)
#define CR2_LAST			(1U<<12)
#define CR1_PE				(1U<<0)

#define HIFCR_CTCIF6		(1U<<21)
#define HIFCR_CTCIF5		(1U<<11)


#define HIFSR_TCIF5		(1U<<11)
#define HIFSR_TCIF6		(1U<<21)

#define PERIPH_CLK			16

#define SR2_BUSY			(1U<<1)

#define I2C_100KHZ				80   // 0B 0101 0000
#define SD_MODE_MAX_RISE_TIME	17

#define CR1_START			(1U<<8)
#define SR1_SB				(1U<<0)
#define SR1_ADDR			(1U<<1)
#define SR1_TXE				(1U<<7)
#define CR1_ACK				(1U<<10)
#define SR1_BTF				(1U<<2)
#define CR1_STOP			(1U<<9)

uint8_t g_rx_cmplt;
uint8_t g_tx_cmplt;

void i2c1_init(void)
{
	/********I2C GPIO Configuration*************/
	/*Enable clock access to GPIOB*/
	RCC->AHB1ENR |= GPIOBEN;

	/*Set PB8 and PB9 mode to alternate function mode*/
	/*PB8*/
	GPIOB->MODER &= ~(1U<<16);
	GPIOB->MODER |= (1U<<17);

	/*PB9*/
	GPIOB->MODER &= ~(1U<<18);
	GPIOB->MODER |= (1U<<19);

	/*Set PB8 and PB9 alternate function type to I2C1 (AF4)  */
	/*PB8*/
	GPIOB->AFR[1] &=~(1U<<0);
	GPIOB->AFR[1] &=~(1U<<1);
	GPIOB->AFR[1] |=(1U<<2);
	GPIOB->AFR[1] &=~(1U<<3);

	/*PB9*/
	GPIOB->AFR[1] &=~(1U<<4);
	GPIOB->AFR[1] &=~(1U<<5);
	GPIOB->AFR[1] |=(1U<<6);
	GPIOB->AFR[1] &=~(1U<<7);

	/*SCL and SDA respectively*/
	/*Set output type of PB8 and PB9 to open-drain*/
	GPIOB->OTYPER |=(1U<<8);
	GPIOB->OTYPER |=(1U<<9);

	/********I2C  Configuration*************/
	/*Enable clock access to I2C1*/
	RCC->APB1ENR |= I2C1EN;

	/*Reset I2C module*/
	I2C1->CR1 = CR1_SWRST;

	/*Release the reset*/
	I2C1->CR1 &= ~CR1_SWRST;

	/*Enable clock stretching*/
	I2C1->CR1 &=~CR1_NOSTRETCH;

	/*Disable General Call*/
	I2C1->CR1 &=~CR1_ENGC;

	/*Select to use DMA*/
//	I2C1->CR2 |=CR2_DMAEN;

	/*Enable LAST*/
	I2C1->CR2 |=CR2_LAST;

	/*Set source clock speed*/
	I2C1->CR2 |=PERIPH_CLK;

	 /*Set I2C to standard mode, 100kHz clock*/
	I2C1->CCR = I2C_100KHZ; /*Based on Computation*/

	I2C1->TRISE = SD_MODE_MAX_RISE_TIME;

	/*Enable I2C module*/
	I2C1->CR1 |=CR1_PE;

}

void I2C1_Start(void) {
    I2C1->CR1 |= I2C_CR1_START; // Generate START condition
    while (!(I2C1->SR1 & I2C_SR1_SB)); // Wait for SB bit to be set
    I2C1->SR1; // Clear SB by reading SR1
}

void I2C1_Stop(void) {
    I2C1->CR1 |= I2C_CR1_STOP; // Generate STOP condition
}

void I2C1_Write(uint8_t data) {
    while (!(I2C1->SR1 & I2C_SR1_TXE)); // Wait until DR is empty
    I2C1->DR = data; // Write data to DR
    while (!(I2C1->SR1 & I2C_SR1_BTF)); // Wait until byte transfer finished
}
