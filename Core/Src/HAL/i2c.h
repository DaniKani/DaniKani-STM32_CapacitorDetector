#ifndef I2C_H
#define I2C_H

#include "stdio.h"
#include "stm32f446xx.h"  // Adjust to your STM32 series header

void I2C1_Init(void);
void I2C1_Start(void);
void I2C1_Stop(void);
void I2C1_Write(uint8_t data);


#endif /*I2C_H*/

