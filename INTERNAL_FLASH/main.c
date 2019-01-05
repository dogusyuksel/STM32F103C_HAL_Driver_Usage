/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define ADDR_FLASH_PAGE_0     ((uint32_t)0x08000000) /* Base @ of Page 0, 1 Kbytes */
	#define ADDR_FLASH_PAGE_1     ((uint32_t)0x08000400) /* Base @ of Page 1, 1 Kbytes */
	#define ADDR_FLASH_PAGE_2     ((uint32_t)0x08000800) /* Base @ of Page 2, 1 Kbytes */
	#define ADDR_FLASH_PAGE_3     ((uint32_t)0x08000C00) /* Base @ of Page 3, 1 Kbytes */
	#define ADDR_FLASH_PAGE_4     ((uint32_t)0x08001000) /* Base @ of Page 4, 1 Kbytes */
	#define ADDR_FLASH_PAGE_5     ((uint32_t)0x08001400) /* Base @ of Page 5, 1 Kbytes */
	#define ADDR_FLASH_PAGE_6     ((uint32_t)0x08001800) /* Base @ of Page 6, 1 Kbytes */
	#define ADDR_FLASH_PAGE_7     ((uint32_t)0x08001C00) /* Base @ of Page 7, 1 Kbytes */
	#define ADDR_FLASH_PAGE_8     ((uint32_t)0x08002000) /* Base @ of Page 8, 1 Kbytes */
	#define ADDR_FLASH_PAGE_9     ((uint32_t)0x08002400) /* Base @ of Page 9, 1 Kbytes */
	#define ADDR_FLASH_PAGE_10    ((uint32_t)0x08002800) /* Base @ of Page 10, 1 Kbytes */
	#define ADDR_FLASH_PAGE_11    ((uint32_t)0x08002C00) /* Base @ of Page 11, 1 Kbytes */
	#define ADDR_FLASH_PAGE_12    ((uint32_t)0x08003000) /* Base @ of Page 12, 1 Kbytes */
	#define ADDR_FLASH_PAGE_13    ((uint32_t)0x08003400) /* Base @ of Page 13, 1 Kbytes */
	#define ADDR_FLASH_PAGE_14    ((uint32_t)0x08003800) /* Base @ of Page 14, 1 Kbytes */
	#define ADDR_FLASH_PAGE_15    ((uint32_t)0x08003C00) /* Base @ of Page 15, 1 Kbytes */
	#define ADDR_FLASH_PAGE_16    ((uint32_t)0x08004000) /* Base @ of Page 16, 1 Kbytes */
	#define ADDR_FLASH_PAGE_17    ((uint32_t)0x08004400) /* Base @ of Page 17, 1 Kbytes */
	#define ADDR_FLASH_PAGE_18    ((uint32_t)0x08004800) /* Base @ of Page 18, 1 Kbytes */
	#define ADDR_FLASH_PAGE_19    ((uint32_t)0x08004C00) /* Base @ of Page 19, 1 Kbytes */
	#define ADDR_FLASH_PAGE_20    ((uint32_t)0x08005000) /* Base @ of Page 20, 1 Kbytes */
	#define ADDR_FLASH_PAGE_21    ((uint32_t)0x08005400) /* Base @ of Page 21, 1 Kbytes */
	#define ADDR_FLASH_PAGE_22    ((uint32_t)0x08005800) /* Base @ of Page 22, 1 Kbytes */
	#define ADDR_FLASH_PAGE_23    ((uint32_t)0x08005C00) /* Base @ of Page 23, 1 Kbytes */
	#define ADDR_FLASH_PAGE_24    ((uint32_t)0x08006000) /* Base @ of Page 24, 1 Kbytes */
	#define ADDR_FLASH_PAGE_25    ((uint32_t)0x08006400) /* Base @ of Page 25, 1 Kbytes */
	#define ADDR_FLASH_PAGE_26    ((uint32_t)0x08006800) /* Base @ of Page 26, 1 Kbytes */
	#define ADDR_FLASH_PAGE_27    ((uint32_t)0x08006C00) /* Base @ of Page 27, 1 Kbytes */
	#define ADDR_FLASH_PAGE_28    ((uint32_t)0x08007000) /* Base @ of Page 28, 1 Kbytes */
	#define ADDR_FLASH_PAGE_29    ((uint32_t)0x08007400) /* Base @ of Page 29, 1 Kbytes */
	#define ADDR_FLASH_PAGE_30    ((uint32_t)0x08007800) /* Base @ of Page 30, 1 Kbytes */
	#define ADDR_FLASH_PAGE_31    ((uint32_t)0x08007C00) /* Base @ of Page 31, 1 Kbytes */
	#define ADDR_FLASH_PAGE_32    ((uint32_t)0x08008000) /* Base @ of Page 32, 1 Kbytes */
	#define ADDR_FLASH_PAGE_33    ((uint32_t)0x08008400) /* Base @ of Page 33, 1 Kbytes */
	#define ADDR_FLASH_PAGE_34    ((uint32_t)0x08008800) /* Base @ of Page 34, 1 Kbytes */
	#define ADDR_FLASH_PAGE_35    ((uint32_t)0x08008C00) /* Base @ of Page 35, 1 Kbytes */
	#define ADDR_FLASH_PAGE_36    ((uint32_t)0x08009000) /* Base @ of Page 36, 1 Kbytes */
	#define ADDR_FLASH_PAGE_37    ((uint32_t)0x08009400) /* Base @ of Page 37, 1 Kbytes */
	#define ADDR_FLASH_PAGE_38    ((uint32_t)0x08009800) /* Base @ of Page 38, 1 Kbytes */
	#define ADDR_FLASH_PAGE_39    ((uint32_t)0x08009C00) /* Base @ of Page 39, 1 Kbytes */
	#define ADDR_FLASH_PAGE_40    ((uint32_t)0x0800A000) /* Base @ of Page 40, 1 Kbytes */
	#define ADDR_FLASH_PAGE_41    ((uint32_t)0x0800A400) /* Base @ of Page 41, 1 Kbytes */
	#define ADDR_FLASH_PAGE_42    ((uint32_t)0x0800A800) /* Base @ of Page 42, 1 Kbytes */
	#define ADDR_FLASH_PAGE_43    ((uint32_t)0x0800AC00) /* Base @ of Page 43, 1 Kbytes */
	#define ADDR_FLASH_PAGE_44    ((uint32_t)0x0800B000) /* Base @ of Page 44, 1 Kbytes */
	#define ADDR_FLASH_PAGE_45    ((uint32_t)0x0800B400) /* Base @ of Page 45, 1 Kbytes */
	#define ADDR_FLASH_PAGE_46    ((uint32_t)0x0800B800) /* Base @ of Page 46, 1 Kbytes */
	#define ADDR_FLASH_PAGE_47    ((uint32_t)0x0800BC00) /* Base @ of Page 47, 1 Kbytes */
	#define ADDR_FLASH_PAGE_48    ((uint32_t)0x0800C000) /* Base @ of Page 48, 1 Kbytes */
	#define ADDR_FLASH_PAGE_49    ((uint32_t)0x0800C400) /* Base @ of Page 49, 1 Kbytes */
	#define ADDR_FLASH_PAGE_50    ((uint32_t)0x0800C800) /* Base @ of Page 50, 1 Kbytes */
	#define ADDR_FLASH_PAGE_51    ((uint32_t)0x0800CC00) /* Base @ of Page 51, 1 Kbytes */
	#define ADDR_FLASH_PAGE_52    ((uint32_t)0x0800D000) /* Base @ of Page 52, 1 Kbytes */
	#define ADDR_FLASH_PAGE_53    ((uint32_t)0x0800D400) /* Base @ of Page 53, 1 Kbytes */
	#define ADDR_FLASH_PAGE_54    ((uint32_t)0x0800D800) /* Base @ of Page 54, 1 Kbytes */
	#define ADDR_FLASH_PAGE_55    ((uint32_t)0x0800DC00) /* Base @ of Page 55, 1 Kbytes */
	#define ADDR_FLASH_PAGE_56    ((uint32_t)0x0800E000) /* Base @ of Page 56, 1 Kbytes */
	#define ADDR_FLASH_PAGE_57    ((uint32_t)0x0800E400) /* Base @ of Page 57, 1 Kbytes */
	#define ADDR_FLASH_PAGE_58    ((uint32_t)0x0800E800) /* Base @ of Page 58, 1 Kbytes */
	#define ADDR_FLASH_PAGE_59    ((uint32_t)0x0800EC00) /* Base @ of Page 59, 1 Kbytes */
	#define ADDR_FLASH_PAGE_60    ((uint32_t)0x0800F000) /* Base @ of Page 60, 1 Kbytes */
	#define ADDR_FLASH_PAGE_61    ((uint32_t)0x0800F400) /* Base @ of Page 61, 1 Kbytes */
	#define ADDR_FLASH_PAGE_62    ((uint32_t)0x0800F800) /* Base @ of Page 62, 1 Kbytes */
	#define ADDR_FLASH_PAGE_63    ((uint32_t)0x0800FC00) /* Base @ of Page 63, 1 Kbytes */
	#define ADDR_FLASH_PAGE_64    ((uint32_t)0x08010000) /* Base @ of Page 64, 1 Kbytes */
	#define ADDR_FLASH_PAGE_65    ((uint32_t)0x08010400) /* Base @ of Page 65, 1 Kbytes */
	#define ADDR_FLASH_PAGE_66    ((uint32_t)0x08010800) /* Base @ of Page 66, 1 Kbytes */
	#define ADDR_FLASH_PAGE_67    ((uint32_t)0x08010C00) /* Base @ of Page 67, 1 Kbytes */
	#define ADDR_FLASH_PAGE_68    ((uint32_t)0x08011000) /* Base @ of Page 68, 1 Kbytes */
	#define ADDR_FLASH_PAGE_69    ((uint32_t)0x08011400) /* Base @ of Page 69, 1 Kbytes */
	#define ADDR_FLASH_PAGE_70    ((uint32_t)0x08011800) /* Base @ of Page 70, 1 Kbytes */
	#define ADDR_FLASH_PAGE_71    ((uint32_t)0x08011C00) /* Base @ of Page 71, 1 Kbytes */
	#define ADDR_FLASH_PAGE_72    ((uint32_t)0x08012000) /* Base @ of Page 72, 1 Kbytes */
	#define ADDR_FLASH_PAGE_73    ((uint32_t)0x08012400) /* Base @ of Page 73, 1 Kbytes */
	#define ADDR_FLASH_PAGE_74    ((uint32_t)0x08012800) /* Base @ of Page 74, 1 Kbytes */
	#define ADDR_FLASH_PAGE_75    ((uint32_t)0x08012C00) /* Base @ of Page 75, 1 Kbytes */
	#define ADDR_FLASH_PAGE_76    ((uint32_t)0x08013000) /* Base @ of Page 76, 1 Kbytes */
	#define ADDR_FLASH_PAGE_77    ((uint32_t)0x08013400) /* Base @ of Page 77, 1 Kbytes */
	#define ADDR_FLASH_PAGE_78    ((uint32_t)0x08013800) /* Base @ of Page 78, 1 Kbytes */
	#define ADDR_FLASH_PAGE_79    ((uint32_t)0x08013C00) /* Base @ of Page 79, 1 Kbytes */
	#define ADDR_FLASH_PAGE_80    ((uint32_t)0x08014000) /* Base @ of Page 80, 1 Kbytes */
	#define ADDR_FLASH_PAGE_81    ((uint32_t)0x08014400) /* Base @ of Page 81, 1 Kbytes */
	#define ADDR_FLASH_PAGE_82    ((uint32_t)0x08014800) /* Base @ of Page 82, 1 Kbytes */
	#define ADDR_FLASH_PAGE_83    ((uint32_t)0x08014C00) /* Base @ of Page 83, 1 Kbytes */
	#define ADDR_FLASH_PAGE_84    ((uint32_t)0x08015000) /* Base @ of Page 84, 1 Kbytes */
	#define ADDR_FLASH_PAGE_85    ((uint32_t)0x08015400) /* Base @ of Page 85, 1 Kbytes */
	#define ADDR_FLASH_PAGE_86    ((uint32_t)0x08015800) /* Base @ of Page 86, 1 Kbytes */
	#define ADDR_FLASH_PAGE_87    ((uint32_t)0x08015C00) /* Base @ of Page 87, 1 Kbytes */
	#define ADDR_FLASH_PAGE_88    ((uint32_t)0x08016000) /* Base @ of Page 88, 1 Kbytes */
	#define ADDR_FLASH_PAGE_89    ((uint32_t)0x08016400) /* Base @ of Page 89, 1 Kbytes */
	#define ADDR_FLASH_PAGE_90    ((uint32_t)0x08016800) /* Base @ of Page 90, 1 Kbytes */
	#define ADDR_FLASH_PAGE_91    ((uint32_t)0x08016C00) /* Base @ of Page 91, 1 Kbytes */
	#define ADDR_FLASH_PAGE_92    ((uint32_t)0x08017000) /* Base @ of Page 92, 1 Kbytes */
	#define ADDR_FLASH_PAGE_93    ((uint32_t)0x08017400) /* Base @ of Page 93, 1 Kbytes */
	#define ADDR_FLASH_PAGE_94    ((uint32_t)0x08017800) /* Base @ of Page 94, 1 Kbytes */
	#define ADDR_FLASH_PAGE_95    ((uint32_t)0x08017C00) /* Base @ of Page 95, 1 Kbytes */
	#define ADDR_FLASH_PAGE_96    ((uint32_t)0x08018000) /* Base @ of Page 96, 1 Kbytes */
	#define ADDR_FLASH_PAGE_97    ((uint32_t)0x08018400) /* Base @ of Page 97, 1 Kbytes */
	#define ADDR_FLASH_PAGE_98    ((uint32_t)0x08018800) /* Base @ of Page 98, 1 Kbytes */
	#define ADDR_FLASH_PAGE_99    ((uint32_t)0x08018C00) /* Base @ of Page 99, 1 Kbytes */
	#define ADDR_FLASH_PAGE_100   ((uint32_t)0x08019000) /* Base @ of Page 100, 1 Kbytes */
	#define ADDR_FLASH_PAGE_101   ((uint32_t)0x08019400) /* Base @ of Page 101, 1 Kbytes */
	#define ADDR_FLASH_PAGE_102   ((uint32_t)0x08019800) /* Base @ of Page 102, 1 Kbytes */
	#define ADDR_FLASH_PAGE_103   ((uint32_t)0x08019C00) /* Base @ of Page 103, 1 Kbytes */
	#define ADDR_FLASH_PAGE_104   ((uint32_t)0x0801A000) /* Base @ of Page 104, 1 Kbytes */
	#define ADDR_FLASH_PAGE_105   ((uint32_t)0x0801A400) /* Base @ of Page 105, 1 Kbytes */
	#define ADDR_FLASH_PAGE_106   ((uint32_t)0x0801A800) /* Base @ of Page 106, 1 Kbytes */
	#define ADDR_FLASH_PAGE_107   ((uint32_t)0x0801AC00) /* Base @ of Page 107, 1 Kbytes */
	#define ADDR_FLASH_PAGE_108   ((uint32_t)0x0801B000) /* Base @ of Page 108, 1 Kbytes */
	#define ADDR_FLASH_PAGE_109   ((uint32_t)0x0801B400) /* Base @ of Page 109, 1 Kbytes */
	#define ADDR_FLASH_PAGE_110   ((uint32_t)0x0801B800) /* Base @ of Page 110, 1 Kbytes */
	#define ADDR_FLASH_PAGE_111   ((uint32_t)0x0801BC00) /* Base @ of Page 111, 1 Kbytes */
	#define ADDR_FLASH_PAGE_112   ((uint32_t)0x0801C000) /* Base @ of Page 112, 1 Kbytes */
	#define ADDR_FLASH_PAGE_113   ((uint32_t)0x0801C400) /* Base @ of Page 113, 1 Kbytes */
	#define ADDR_FLASH_PAGE_114   ((uint32_t)0x0801C800) /* Base @ of Page 114, 1 Kbytes */
	#define ADDR_FLASH_PAGE_115   ((uint32_t)0x0801CC00) /* Base @ of Page 115, 1 Kbytes */
	#define ADDR_FLASH_PAGE_116   ((uint32_t)0x0801D000) /* Base @ of Page 116, 1 Kbytes */
	#define ADDR_FLASH_PAGE_117   ((uint32_t)0x0801D400) /* Base @ of Page 117, 1 Kbytes */
	#define ADDR_FLASH_PAGE_118   ((uint32_t)0x0801D800) /* Base @ of Page 118, 1 Kbytes */
	#define ADDR_FLASH_PAGE_119   ((uint32_t)0x0801DC00) /* Base @ of Page 119, 1 Kbytes */
	#define ADDR_FLASH_PAGE_120   ((uint32_t)0x0801E000) /* Base @ of Page 120, 1 Kbytes */
	#define ADDR_FLASH_PAGE_121   ((uint32_t)0x0801E400) /* Base @ of Page 121, 1 Kbytes */
	#define ADDR_FLASH_PAGE_122   ((uint32_t)0x0801E800) /* Base @ of Page 122, 1 Kbytes */
	#define ADDR_FLASH_PAGE_123   ((uint32_t)0x0801EC00) /* Base @ of Page 123, 1 Kbytes */
	#define ADDR_FLASH_PAGE_124   ((uint32_t)0x0801F000) /* Base @ of Page 124, 1 Kbytes */
	#define ADDR_FLASH_PAGE_125   ((uint32_t)0x0801F400) /* Base @ of Page 125, 1 Kbytes */
	#define ADDR_FLASH_PAGE_126   ((uint32_t)0x0801F800) /* Base @ of Page 126, 1 Kbytes */
	#define ADDR_FLASH_PAGE_127   ((uint32_t)0x0801FC00) /* Base @ of Page 127, 1 Kbytes */
	
	#define A_FLASH_PAGE_SIZE 0x400
	#define FLASH_PAGE_MAX_DATA_COUNT (A_FLASH_PAGE_SIZE / sizeof(uint32_t)) //it is 
	#define FLASH_MAX_PAGE_BASE_ADDRESS 	ADDR_FLASH_PAGE_64 //since our MCU is 103C8, 8 means 64kByte flash
	#define FLASH_DATA_PART_START_ADDRESS ADDR_FLASH_PAGE_60
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART3_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
//it holds a page sized data
uint32_t OnePageDataHolder[FLASH_PAGE_MAX_DATA_COUNT];
	
void FlashLock(void);
void WriteData(uint32_t address, uint32_t *data, uint32_t len);
void ReadData(uint32_t address, uint32_t *data, uint32_t len);
void CLEARFLASH(void);
void ERASEPAGE(uint32_t baseAddress);
void FlashUnlock(void);
uint32_t ReadWord(uint32_t address);
void ReadAndTransferData(uint32_t address);
	
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
uint32_t data = 0x19, readdata = 0;
uint32_t address = FLASH_DATA_PART_START_ADDRESS;
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();

  /* USER CODE BEGIN 2 */

	//CLEARFLASH();

	ReadData(address, &readdata, 1);
	if(readdata != data)
		WriteData(address, &data, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_RED_Pin|LED_GREEN_Pin|LED_BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_INPUT_Pin */
  GPIO_InitStruct.Pin = USER_INPUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_INPUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_RED_Pin LED_GREEN_Pin LED_BLUE_Pin */
  GPIO_InitStruct.Pin = LED_RED_Pin|LED_GREEN_Pin|LED_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
void FlashLock(void)
{
	HAL_FLASH_Lock();
}

void ReadAndTransferData(uint32_t address)
{
	//control input data first
	assert(address <= (ADDR_FLASH_PAGE_64 - sizeof(uint32_t)));
	assert(address >= FLASH_DATA_PART_START_ADDRESS);
	
  uint32_t BaseAddress = address;
	if(((address - FLASH_DATA_PART_START_ADDRESS) % A_FLASH_PAGE_SIZE) != 0)
	{
		int dummy = (int)((address - FLASH_DATA_PART_START_ADDRESS) / A_FLASH_PAGE_SIZE);
		BaseAddress = (dummy * A_FLASH_PAGE_SIZE) + FLASH_DATA_PART_START_ADDRESS;
	}	
	
	int i = 0;
	
	for(i=0;i<FLASH_PAGE_MAX_DATA_COUNT;i++)
		OnePageDataHolder[i] = 0;
	
	for(i = 0; i < FLASH_PAGE_MAX_DATA_COUNT; i++)
	{
		(OnePageDataHolder[i]) = ReadWord(BaseAddress);
    BaseAddress += 4;
	}
}

void WriteData(uint32_t address, uint32_t *data, uint32_t len)
{	
	//control the input
	assert((data != NULL) && (len != 0));

	assert(address >= FLASH_DATA_PART_START_ADDRESS);
	assert(address <= FLASH_MAX_PAGE_BASE_ADDRESS);
	assert((address % sizeof(uint32_t)) == 0);

	//In order to hold other data, save them to an array
	ReadAndTransferData(address);

	//then find the base address of the input address
	uint32_t BaseAddress = address;
	if(((address - FLASH_DATA_PART_START_ADDRESS) % A_FLASH_PAGE_SIZE) != 0)
	{
		int dummy = (int)((address - FLASH_DATA_PART_START_ADDRESS) / A_FLASH_PAGE_SIZE);
		BaseAddress = (dummy * A_FLASH_PAGE_SIZE) + FLASH_DATA_PART_START_ADDRESS;
	}

	/* Unlock the Flash Bank1 Program Erase controller */
	FlashUnlock();

	/* Clear All pending flags */
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);	

	/* Erase the FLASH pages */
	ERASEPAGE(BaseAddress);


	int i = 0;
	int j = 0;
	//change array data with user specified data
	int dataIndex = (int)((address - BaseAddress) / sizeof(uint32_t));

	assert((dataIndex + len) <= FLASH_PAGE_MAX_DATA_COUNT);

	for(i = dataIndex; i < (len + dataIndex); i++)
	{
		OnePageDataHolder[i] = data[j++];
	}

	/* Program Flash related whole page */
	uint32_t Address = BaseAddress;

	for(i = 0; i < FLASH_PAGE_MAX_DATA_COUNT; i++)
	{
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, OnePageDataHolder[i]);
		Address = Address + 4;
	}

	//lock the flash
	FlashLock();
}

void ReadData(uint32_t address, uint32_t *data, uint32_t len)
{
	//control the input
	assert((data != NULL) && (len != 0));
	
	assert(address >= FLASH_DATA_PART_START_ADDRESS);
	assert(address <= FLASH_MAX_PAGE_BASE_ADDRESS);
	assert((address % sizeof(uint32_t)) == 0);
	
	uint32_t Address = address;
	int i = 0;
	for(i = 0; i < len; i++)
	{
		data[i] = ReadWord(Address);
		Address += 4;
	}	
}

void CLEARFLASH(void)
{
	/* Unlock the Flash Bank1 Program Erase controller */
	FlashUnlock();

	/* Clear All pending flags */
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);	


	/* Erase the FLASH pages */
	int EraseCounter = 0, NbrOfPage = 0;
	NbrOfPage = (FLASH_MAX_PAGE_BASE_ADDRESS - FLASH_DATA_PART_START_ADDRESS) / A_FLASH_PAGE_SIZE;
	for(EraseCounter = 0; (EraseCounter < NbrOfPage); EraseCounter++)
	{
		ERASEPAGE(FLASH_DATA_PART_START_ADDRESS + (A_FLASH_PAGE_SIZE * EraseCounter));
	}

	//lock the flash
	FlashLock();
}

void FlashUnlock(void)
{
	HAL_FLASH_Unlock();
}

uint32_t ReadWord(uint32_t address)
{
	return (*(__IO uint32_t*) address);
}

void ERASEPAGE(uint32_t baseAddress)
{
	FLASH_EraseInitTypeDef EraseInitStruct;
	
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.PageAddress = baseAddress;
	EraseInitStruct.NbPages = 1;
	
	uint32_t pError = 0;
	HAL_FLASHEx_Erase(&EraseInitStruct, &pError);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
