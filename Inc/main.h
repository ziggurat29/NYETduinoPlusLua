/**
  ******************************************************************************
  * File Name          : main.hpp
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define PWR_LED_OFF_Pin GPIO_PIN_13
#define PWR_LED_OFF_GPIO_Port GPIOC
#define SWITCH1_A_Pin GPIO_PIN_14
#define SWITCH1_A_GPIO_Port GPIOC
#define PWR_CTRL_ETHERNET_Pin GPIO_PIN_15
#define PWR_CTRL_ETHERNET_GPIO_Port GPIOC
#define A0_Pin GPIO_PIN_0
#define A0_GPIO_Port GPIOC
#define A1_Pin GPIO_PIN_1
#define A1_GPIO_Port GPIOC
#define A2_Pin GPIO_PIN_2
#define A2_GPIO_Port GPIOC
#define A3_Pin GPIO_PIN_3
#define A3_GPIO_Port GPIOC
#define D8_Pin GPIO_PIN_0
#define D8_GPIO_Port GPIOA
#define D7_Pin GPIO_PIN_1
#define D7_GPIO_Port GPIOA
#define D3_Pin GPIO_PIN_2
#define D3_GPIO_Port GPIOA
#define D2_Pin GPIO_PIN_3
#define D2_GPIO_Port GPIOA
#define ENC_INT_Pin GPIO_PIN_4
#define ENC_INT_GPIO_Port GPIOA
#define ENC_SCK_Pin GPIO_PIN_5
#define ENC_SCK_GPIO_Port GPIOA
#define D9_Pin GPIO_PIN_6
#define D9_GPIO_Port GPIOA
#define ENC_MOSI_Pin GPIO_PIN_7
#define ENC_MOSI_GPIO_Port GPIOA
#define A4_Pin GPIO_PIN_4
#define A4_GPIO_Port GPIOC
#define A5_Pin GPIO_PIN_5
#define A5_GPIO_Port GPIOC
#define MICROSD_CS_Pin GPIO_PIN_0
#define MICROSD_CS_GPIO_Port GPIOB
#define PWR_CTRL_MICROSD_Pin GPIO_PIN_1
#define PWR_CTRL_MICROSD_GPIO_Port GPIOB
#define CTRL_OF_PWR_HEADERS_Pin GPIO_PIN_2
#define CTRL_OF_PWR_HEADERS_GPIO_Port GPIOB
#define D10_Pin GPIO_PIN_10
#define D10_GPIO_Port GPIOB
#define SWITCH1_B_Pin GPIO_PIN_11
#define SWITCH1_B_GPIO_Port GPIOB
#define D4_Pin GPIO_PIN_12
#define D4_GPIO_Port GPIOB
#define D13_Pin GPIO_PIN_13
#define D13_GPIO_Port GPIOB
#define D12_Pin GPIO_PIN_14
#define D12_GPIO_Port GPIOB
#define D11_Pin GPIO_PIN_15
#define D11_GPIO_Port GPIOB
#define ENC_CS_Pin GPIO_PIN_8
#define ENC_CS_GPIO_Port GPIOC
#define CLOCK_OUT_25MHZ_Pin GPIO_PIN_8
#define CLOCK_OUT_25MHZ_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_10
#define LED_GPIO_Port GPIOA
#define MICROSD_SCK_Pin GPIO_PIN_10
#define MICROSD_SCK_GPIO_Port GPIOC
#define MICROSD_MISO_Pin GPIO_PIN_11
#define MICROSD_MISO_GPIO_Port GPIOC
#define MICROSD_MOSI_Pin GPIO_PIN_12
#define MICROSD_MOSI_GPIO_Port GPIOC
#define ENC_RESET_Pin GPIO_PIN_2
#define ENC_RESET_GPIO_Port GPIOD
#define ENC_MISO_Pin GPIO_PIN_4
#define ENC_MISO_GPIO_Port GPIOB
#define MICRO_SD_CARD_INSERTED_Pin GPIO_PIN_5
#define MICRO_SD_CARD_INSERTED_GPIO_Port GPIOB
#define SCL_Pin GPIO_PIN_6
#define SCL_GPIO_Port GPIOB
#define SDA_Pin GPIO_PIN_7
#define SDA_GPIO_Port GPIOB
#define D5_Pin GPIO_PIN_8
#define D5_GPIO_Port GPIOB
#define D6_Pin GPIO_PIN_9
#define D6_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
 #define USE_FULL_ASSERT    1U 

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
