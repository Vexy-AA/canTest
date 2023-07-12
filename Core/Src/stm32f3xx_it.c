/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f3xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint32_t timeNow = 0;
uint32_t timeOld = 0;
uint8_t buf[100];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_FS;
extern CAN_HandleTypeDef hcan;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F3xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f3xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles USB low priority or CAN_RX0 interrupts.
  */
void USB_LP_CAN_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN USB_LP_CAN_RX0_IRQn 0 */
  canRxInt(&hcan,0);
  return;
  /* USER CODE END USB_LP_CAN_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan);
  /* USER CODE BEGIN USB_LP_CAN_RX0_IRQn 1 */
  
  /* USER CODE END USB_LP_CAN_RX0_IRQn 1 */
}

/**
  * @brief This function handles CAN_RX1 interrupt.
  */
void CAN_RX1_IRQHandler(void)
{
  /* USER CODE BEGIN CAN_RX1_IRQn 0 */
  canRxInt(&hcan,1);
  return;
  /* USER CODE END CAN_RX1_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan);
  /* USER CODE BEGIN CAN_RX1_IRQn 1 */

  /* USER CODE END CAN_RX1_IRQn 1 */
}

/**
  * @brief This function handles TIM8 update interrupt.
  */
void TIM8_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_UP_IRQn 0 */
  LL_TIM_ClearFlag_UPDATE(TIM8);
  timeNow = HAL_GetTick();
  int32_t deltaT = timeNow - timeOld - 1000;
  timeOld = timeNow;
  if (deltaT <  -1 || deltaT > 1){
    //uint8_t nBytes = snprintf((char*)buf,sizeof(buf),"%ld\n\r",deltaT);
    LL_TIM_SetAutoReload(TIM8,(LL_TIM_GetAutoReload(TIM8) - deltaT));
    //CDC_Transmit_FS(buf, nBytes);
  }
  /* USER CODE END TIM8_UP_IRQn 0 */
  /* USER CODE BEGIN TIM8_UP_IRQn 1 */

  /* USER CODE END TIM8_UP_IRQn 1 */
}

/**
  * @brief This function handles USB low priority interrupt remap.
  */
void USB_LP_IRQHandler(void)
{
  /* USER CODE BEGIN USB_LP_IRQn 0 */

  /* USER CODE END USB_LP_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_FS);
  /* USER CODE BEGIN USB_LP_IRQn 1 */

  /* USER CODE END USB_LP_IRQn 1 */
}

/* USER CODE BEGIN 1 */


void SPI4_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}
void FPU_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}
void TIM20_CC_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}
void TIM20_TRG_COM_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}
void TIM20_UP_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}
void TIM20_BRK_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}


void USB_HP_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}
void I2C3_ER_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}
void I2C3_EV_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}
void COMP7_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}
void COMP4_5_6_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}
void COMP1_2_3_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}
void ADC4_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}
void DMA2_Channel5_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}
void DMA2_Channel4_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}
void DMA2_Channel3_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}

void DMA2_Channel2_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}

void DMA2_Channel1_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}

void TIM7_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}

void TIM6_DAC_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}

void UART5_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}

void UART4_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}

void SPI3_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}

void FMC_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}
void ADC3_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}
void TIM8_CC_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}
void TIM8_TRG_COM_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}

void TIM8_BRK_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}
void USBWakeUp_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}
void EXTI15_10_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}
void USART3_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}
void USART2_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}
void USART1_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}
void SPI2_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}
void SPI1_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}
void I2C2_ER_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}
void I2C2_EV_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}
void I2C1_ER_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}
void I2C1_EV_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}
void TIM4_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}
void TIM3_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}
void TIM2_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}
void TIM1_CC_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}
void TIM1_TRG_COM_TIM17_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}
void TIM1_UP_TIM16_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}
void TIM1_BRK_TIM15_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}
void EXTI9_5_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}
void CAN_SCE_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}

void USB_HP_CAN_TX_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}
void ADC1_2_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}
void DMA1_Channel7_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}
void DMA1_Channel6_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}
void DMA1_Channel5_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}
void DMA1_Channel4_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}
void DMA1_Channel3_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}
void DMA1_Channel2_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}
void DMA1_Channel1_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}
void EXTI4_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}
void EXTI3_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}
void EXTI2_TSC_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}
void EXTI1_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}
void EXTI0_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}
void RCC_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}
void FLASH_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}
void RTC_WKUP_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}
void TAMP_STAMP_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}
void PVD_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}
void WWDG_IRQHandler(void)
{
  uint8_t cnt = 0;
  while(1)
  {
    cnt++;
  }
}






/* USER CODE END 1 */
