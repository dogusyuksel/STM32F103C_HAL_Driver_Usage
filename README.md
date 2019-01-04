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
