/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : platform_config.h
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : Evaluation board specific configuration file.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PLATFORM_CONFIG_H
#define __PLATFORM_CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* global variable ------------------------------------------------------------*/
/* global  define ------------------------------------------------------------*/
#define  keypad_0           0x01 
#define  keypad_1           0x02  
#define  keypad_2           0x03 
#define  keypad_3           0x04
#define  GPIO_LED1          GPIOC    
#define  GPIO_PIN_LED1      GPIO_Pin_4 
#define  GPIO_LED2          GPIOC    
#define  GPIO_PIN_LED2      GPIO_Pin_5  
#define  GPIO_LED3          GPIOB    
#define  GPIO_PIN_LED3      GPIO_Pin_0 
#define  GPIO_LED4          GPIOB    
#define  GPIO_PIN_LED4      GPIO_Pin_1
#define  GPIO_LED5          GPIOF    
#define  GPIO_PIN_LED5      GPIO_Pin_11 
extern u16 KEYPAD_STATE;
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void RCC_Configuration(void);
void NVIC_Configuration(void);
void GPIO_Configuration(void);
void USART_Configuration(void);
void Delaytime(u32 nCount);
void EXTI_Configuration(void);
void Light_up_LED(GPIO_TypeDef* GPIOx , u16 GPIO_Pin);

#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)

#endif /* __PLATFORM_CONFIG_H */

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
