# STM32F103C_HAL_Driver_Usage
This repo is kind of a cheat sheet for STM HAL Driver Usage

In this ReadMe file, I will try to explain how to use CubeMx program to use STM32F103C MCU pheripherals.


HAL_DRIVERS -> CubeMX General Steps
-------------------------------------

  1. SYS -> Debug -> Serial Wire
  2. RCC -> HSE -> Crystal
  3. Configuration Tab -> System Part -> RCC Button -> NVIC Settings Tab -> Enable RCC Global Interrupt
  4. Make Clock Config in Clock Configuration tab (select HSE and arrange PLL mul etc)
  5. To generate the code, just remember to select MDK ARM 5 
  
  
GPIO OUTPUT
--------------
  1. Select a pin
  2. Select GPIO_OUTPUT
  3. Right Click and ENTER USER LABEL
  4. Insert for example "LED_GREEN"



GPIO INPUT WITH INTERRUPT
-----------------------------
  1. Select a pin
  2. Select GPIO_EXTI Mode
  3. Insert a user label
  4. Configuration Tab -> System Part -> NVIC Button -> NVIC Tab -> Enable EXTI Interrupt Tick -> Select priority and subPriority
  5. Configuration Tab -> System Part -> GPIO Button -> Select the pin -> and select GPIO Mode for example Trigger in Rising Edge
  
    /**
    * @brief  EXTI line detection callbacks.
    * @param  GPIO_Pin: Specifies the pins connected EXTI line
    * @retval None
    */
    __weak void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
    {
      /* Prevent unused argument(s) compilation warning */
      UNUSED(GPIO_Pin);
      /* NOTE: This function Should not be modified, when the callback is needed,
               the HAL_GPIO_EXTI_Callback could be implemented in the user file
       */
    }


TIMER
-------------------
  1. Select a timer and Clock Source -> Internal Clock Source
  2. Control Clock Configuration tabs and see "APB1 and APB2 Timer Clock" part. In my condition they are equal and 72MHz.
  3. Configuration Tab -> System -> NVIC -> NVIC Tab -> Open the Timer Global Interrupt -> Arrange interrupt priority and subPreority.
  4. Configuration Tab -> Control Tab -> Select the Timer that you want to arrange -> Parameter Settings Tab -> Arrange Prescalar and Counter Period
  eg: Our clock source is 72MHz, lets say arrange 500ms timer then insert prescalar: 72000 and period: 500 (these are uint32_t variables)
  because 72M / (72000 * 500) = 0.5 this means MILLIS, 500 ms
  5. You finally activate the time after Timer Init function and before the main loop with this code "HAL_TIM_Base_Start_IT(&htim2);"
  Here, you should think that you activate the TIM2 for example.

    /**
      * @brief  Period elapsed callback in non blocking mode 
      * @param  htim : TIM handle
      * @retval None
      */
      __weak void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
      {
        /* Prevent unused argument(s) compilation warning */
        UNUSED(htim);
        /* NOTE : This function Should not be modified, when the callback is needed,
                  the __HAL_TIM_PeriodElapsedCallback could be implemented in the user file
         */
      }
      
      
ANALOG
------------------
1. Select the pin that you want to use as ADC pin
2. Enter user labeled name to that pin
3. Be careful about the ADC clock prescalar, and enable its global NVIC interrupt
4. Configuration Tab -> Analog -> ADC1 (for example) -> Parameter Setting -> Continuous Conversation Mode -> Enabled
4. Configuration Tab -> Analog -> ADC1 (for example) -> Parameter Setting -> Scan Conversation Mode -> Enabled //if multiple analog channels
5. Configuration Tab -> Analog -> ADC1 (for example) -> Parameter Setting -> NoF Conversation : Equal with the analog channels
6. If there are multiple analog channels, than you can arrange sample time and reading order by using RANK part.
7. If there is single channel, you do not need to configure dma and use the code below;

            HAL_ADC_Start(&hadc1);
            HAL_ADC_PollForConversion(&hadc1, 100);
            uint16_t adcResult = HAL_ADC_GetValue(&hadc1);
            HAL_ADC_Stop(&hadc1);
	
8. Otherwise, you need to configure dma.
9. Configuration Tab -> Analog -> ADC1 -> DMA Tab and select Add button and then select ADC1 in DMA Request
10. Select DMA Mode Circular and data width as half-word/half-word (wrt your adc pin resolution)
NOTE THAT: set no too fast adc reading cycle in rank config, select all your pins in rank canfig

            uint16_t myAnalogValues[2] = {0, 0}; //outside of the main
            HAL_ADC_Start_DMA(&hadc1, (uint32_t *)myAnalogValues, 2); //after init adc, before main loop



UART
-----------------
1. Select any uart from cubeMX for example Uart3
2. Select Mode -> Asynch
3. Goto Configuration -> Connectivity -> USART3 -> Parameter Setting and set baudrate
4. Then Goto Configuration -> System -> NVIC -> NVIC and open USART3 Global Interrupt and configure priorities


		/**
		  * @brief  Rx Transfer completed callbacks.
		  * @param  huart: pointer to a UART_HandleTypeDef structure that contains
		  *                the configuration information for the specified UART module.
		  * @retval None
		  */
		__weak void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
		{
		  /* Prevent unused argument(s) compilation warning */
		  UNUSED(huart);
		  /* NOTE: This function Should not be modified, when the callback is needed,
			   the HAL_UART_RxCpltCallback could be implemented in the user file
		   */
		}


SPI
--------------
1. Select SPI mode as Full Duplex Master
2. Configuration -> Connectivity -> SPI -> Parameter Setting -> Set "Prescalar" to arrange "Baudrate"
3. Use these functions to communicate the device

	HAL_SPI_Transmit
	
	HAL_SPI_Receive

