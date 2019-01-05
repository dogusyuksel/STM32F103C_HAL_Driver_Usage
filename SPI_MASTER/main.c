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
#define MC25LCxxx_SPI_RDID 		0xAB	//1010 1011 Release from Deep power-down and read electronic signature
#define MC25LCxxx_SPI_RDSR 		0x05	//0000 0101 Read STATUS register
#define MC25LCxxx_SPI_WREN 		0x06	//0000 0110 Set the write enable latch (enable write operations)
#define MC25LCxxx_SPI_WRDI 		0x04	//0000 0100 Reset the write enable latch (disable write operations)
#define MC25LCxxx_SPI_WRITE 	0x02  //0000 0010 Write data to memory array beginning at selected address
#define MC25LCxxx_SPI_READ 		0x03 	//0000 0011 Read data from memory array beginning at selected address
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
unsigned char MC25LC512_ReleaseDeepPowerDownMode(void)
{

	unsigned char SendOneByte;
	uint8_t RecieveByteOfReleaseDeepPowerMode=0;
	SendOneByte=MC25LCxxx_SPI_RDID;
	
	HAL_GPIO_WritePin(EEPROM_1_CS_GPIO_Port,EEPROM_1_CS_Pin,GPIO_PIN_RESET);
	HAL_Delay(5);// Reset The spi Chip //Reset means Enable
	
	HAL_SPI_Transmit(&hspi1, &SendOneByte,1,200);
	
	HAL_SPI_Receive(&hspi1, &RecieveByteOfReleaseDeepPowerMode, 1,200) ;//Address of Manufaturer id High
	HAL_SPI_Receive(&hspi1, &RecieveByteOfReleaseDeepPowerMode, 1,200) ;//Address of Manufaturer id Low
	HAL_SPI_Receive(&hspi1, &RecieveByteOfReleaseDeepPowerMode, 1,200) ;//Manufaturer id
	
	HAL_GPIO_WritePin(EEPROM_1_CS_GPIO_Port,EEPROM_1_CS_Pin,GPIO_PIN_SET); //set means DISABLE
	HAL_Delay(5);// Reset The spi Chip //Reset means Enable
	
	return RecieveByteOfReleaseDeepPowerMode;
	
}

unsigned char MC25LC512_ReadStatusRegister(void)
{

	unsigned char SendOneByte=0;
	unsigned char ReceiveOneByte;
	SendOneByte=MC25LCxxx_SPI_RDSR;
	
	HAL_GPIO_WritePin(EEPROM_1_CS_GPIO_Port,EEPROM_1_CS_Pin,GPIO_PIN_RESET);
	HAL_Delay(5);// Reset The spi Chip //Reset means Enable
	
	HAL_SPI_Transmit(&hspi1,&SendOneByte,1,200);
	HAL_SPI_Receive(&hspi1, &ReceiveOneByte, 1,200) ;//Address of Manufaturer id High
	
	HAL_GPIO_WritePin(EEPROM_1_CS_GPIO_Port,EEPROM_1_CS_Pin,GPIO_PIN_SET);
	HAL_Delay(5);// Reset The spi Chip //Reset means Enable
	return ReceiveOneByte;
}

void MC25LC512_WriteEnableOrDisable(unsigned char EnableOrDisable)
{
	unsigned char SendOneByte=0;
	
	HAL_GPIO_WritePin(EEPROM_1_CS_GPIO_Port,EEPROM_1_CS_Pin,GPIO_PIN_RESET);
	HAL_Delay(5);// Reset The spi Chip //Reset means Enable
	
	if(EnableOrDisable==1)//enable status
	{
			SendOneByte=MC25LCxxx_SPI_WREN;
	}
	else
	{
			SendOneByte=MC25LCxxx_SPI_WRDI;
	}
	HAL_SPI_Transmit(&hspi1 , &SendOneByte, 1, 200) ;	
	
	HAL_GPIO_WritePin(EEPROM_1_CS_GPIO_Port,EEPROM_1_CS_Pin,GPIO_PIN_SET);
	HAL_Delay(5);// Reset The spi Chip //Reset means Enable
}

void MC25LC512_WriteBytes(uint16_t AddresOfData, unsigned char *WriteArrayOfEEProm,unsigned char SizeOfArray){

	unsigned char SendOneByte;
	
	HAL_GPIO_WritePin(EEPROM_1_CS_GPIO_Port,EEPROM_1_CS_Pin,GPIO_PIN_RESET);
	HAL_Delay(5);// Reset The spi Chip //Reset means Enable
	
	SendOneByte = MC25LCxxx_SPI_WRITE;
	
	HAL_SPI_Transmit(&hspi1, &SendOneByte, 1, 200);
	SendOneByte= AddresOfData>>8;
	HAL_SPI_Transmit(&hspi1, &SendOneByte, 1, 200);//High byte of address
	SendOneByte= AddresOfData & 0x00FF;
	HAL_SPI_Transmit(&hspi1, &SendOneByte, 1, 200);//Low byte of address
	HAL_SPI_Transmit(&hspi1, WriteArrayOfEEProm , SizeOfArray, SizeOfArray*20) ;
	HAL_Delay(4);
	
	HAL_GPIO_WritePin(EEPROM_1_CS_GPIO_Port,EEPROM_1_CS_Pin,GPIO_PIN_SET);
	HAL_Delay(5);// Reset The spi Chip //Reset means Enable
	
	MC25LC512_WriteEnableOrDisable(1);
}

void MC25LC512_Read(uint16_t AddresOfData, unsigned char *DataArrayOfEEProm,unsigned char SizeOfArray)
{		
	unsigned char SendOneByte;

	HAL_GPIO_WritePin(EEPROM_1_CS_GPIO_Port,EEPROM_1_CS_Pin,GPIO_PIN_RESET);
	HAL_Delay(5);// Reset The spi Chip //Reset means Enable
	
	SendOneByte = MC25LCxxx_SPI_READ;//Config the Device
	
	HAL_SPI_Transmit(&hspi1, &SendOneByte, 1, 200);
	
	SendOneByte= AddresOfData>>8;
	HAL_SPI_Transmit(&hspi1, &SendOneByte, 1, 200);//High byte of address
	SendOneByte= AddresOfData & 0x00FF;
	HAL_SPI_Transmit(&hspi1, &SendOneByte, 1, 200);//Low byte of address

	HAL_SPI_Receive(&hspi1, DataArrayOfEEProm, SizeOfArray, SizeOfArray*30) ;//Receive Amount of  Data from EEPROM

	HAL_GPIO_WritePin(EEPROM_1_CS_GPIO_Port,EEPROM_1_CS_Pin,GPIO_PIN_SET);
	HAL_Delay(5);// Reset The spi Chip //Reset means Enable
}	
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
unsigned char readDataFromEepromBuffer[4] = {0, 0, 0, 0};
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
  MX_SPI1_Init();

  /* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(EEPROM_1_CS_GPIO_Port,EEPROM_1_CS_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(EEPROM_2_CS_GPIO_Port,EEPROM_2_CS_Pin,GPIO_PIN_SET);
	HAL_Delay(250);
	
	//initialize MC25LC512
	HAL_GPIO_WritePin(EEPROM_1_CS_GPIO_Port,EEPROM_1_CS_Pin,GPIO_PIN_SET); //set means DISABLE
	HAL_Delay(5);
	
	MC25LC512_ReleaseDeepPowerDownMode();
	MC25LC512_ReadStatusRegister();
	MC25LC512_WriteEnableOrDisable(1);
	//end of initialize
	
	//read data first
	MC25LC512_Read(0x00d0, readDataFromEepromBuffer, 4);
	
	//write data to eeprom
	int i = 0;
	for(i=0;i<4;i++)
		readDataFromEepromBuffer[i] = 0;
	MC25LC512_WriteBytes(0x00d0, (unsigned char*)"dodo", 4);
	MC25LC512_Read(0x00d0, readDataFromEepromBuffer, 4);
	
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

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

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
  HAL_GPIO_WritePin(GPIOB, EEPROM_1_CS_Pin|EEPROM_2_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : EEPROM_1_CS_Pin EEPROM_2_CS_Pin */
  GPIO_InitStruct.Pin = EEPROM_1_CS_Pin|EEPROM_2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
