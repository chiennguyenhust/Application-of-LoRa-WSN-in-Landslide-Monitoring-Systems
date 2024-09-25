/*
 * DS18B20.h
 *
 *  Created on: Sep 23, 2024
 *      Author: chien
 */

#ifndef DS18B20_H_
#define DS18B20_H_

#include <stm32f4xx.h>
#include "Timer_Delay.h"



#define DS18B20_PORT GPIOA
#define DS18B20_PIN  GPIO_PIN_0


uint8_t DS18B20_Start(void);
void DS18B20_Write (uint8_t data);
uint8_t DS18B20_Read(void);

void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) ;
void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);



#endif /* DS18B20_H_ */
