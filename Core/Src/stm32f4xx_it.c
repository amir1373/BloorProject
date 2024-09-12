/* USER CODE BEGIN Header */
/**
******************************************************************************
* @file    stm32f4xx_it.c
* @brief   Interrupt Service Routines.
******************************************************************************
* @attention
*
* Copyright (c) 2024 STMicroelectronics.
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
#include "stm32f4xx_it.h"
#include "FreeRTOS.h"
#include "task.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mb.h"
#include "mbport.h"
#include "math.h"
#include "stdlib.h"
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
extern int init_bit ;
extern int modbus_state_bit ;
extern uint16_t downcounter;
extern int counter_column;
int toggle_led = 0;

extern int pulseTimeLevelArm1 ;
extern int pulseTimeLevelArm2 ;
extern int pulseTimeColumn1 ;
extern int pulseTimeColumn2 ;
extern int timeOrRepeatRemainingMilis;
extern int startProcess ;
extern int timeOrRepeatRemaining;
extern int timeRepeatMode ;
extern int restInExtentionmilis ;
extern int restInFlexionmilis ;
extern uint16_t restInFlexion ;
extern uint16_t restInExtention;
typedef struct{
  int16_t velocity;
  int64_t position;
  uint32_t last_counter_value;
  int direction;
  int isMoving;
}encoder_instance;
extern TIM_HandleTypeDef htim1;
extern int encoderValueP; 
extern encoder_instance enc_instance_mot;
extern void update_encoder(encoder_instance *encoder_value, TIM_HandleTypeDef *htim);
extern int chartCounter ;

extern int encoder_position ;
extern int encoder_velocity ;
extern int levelArmAngle;
extern uint16_t levelArmCalibrationDone ;
extern uint16_t levelArmPulsePerDegree;
extern int encoder_direction ;

extern int totalWorkTimeCounter ;
extern uint16_t totalWorkTime ;
extern int totalWorkTimeWriteFlag;


extern int columnAutoUpCounter ;
extern int columnAutoDownCounter ;

extern int columnProtectCounter ;
extern int levelArmProtectCounter ;
extern int columnPositionCounter;
extern int modbusErrorCounter ;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim11;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
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
  HAL_NVIC_SystemReset();
  /* USER CODE END NonMaskableInt_IRQn 0 */
  HAL_RCC_NMI_IRQHandler();
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
  HAL_NVIC_SystemReset();
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
  HAL_NVIC_SystemReset();
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
  HAL_NVIC_SystemReset();
  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */
  HAL_NVIC_SystemReset();
  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */
  
  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
  toggle_led ++;
  if(toggle_led >=1000){
    toggle_led = 0;
    HAL_GPIO_TogglePin(LED_ACK_GPIO_Port, LED_ACK_Pin);
    chartCounter = chartCounter +1 ;
  }
  pulseTimeLevelArm1 ++ ;
  pulseTimeLevelArm2 ++;
  pulseTimeColumn1 ++;
  pulseTimeColumn2 ++;
  totalWorkTimeCounter ++ ;
  columnAutoUpCounter ++;
  columnAutoDownCounter ++;
  modbusErrorCounter ++ ;
  
  columnProtectCounter ++;
  levelArmProtectCounter ++;
  columnPositionCounter ++ ;
  
  
  if(totalWorkTimeCounter == 60){
    totalWorkTime = totalWorkTime + 1;
    totalWorkTimeWriteFlag = 1;
    
  }
  
  if(startProcess > 0 && timeRepeatMode == 1){
    timeOrRepeatRemainingMilis -- ;
    timeOrRepeatRemaining = (timeOrRepeatRemainingMilis /60)/1000;
  }  
  if(startProcess == 3){
    restInExtentionmilis -- ;
    restInExtention = restInExtentionmilis / 1000;
  }
  if(startProcess == 5){
    restInFlexionmilis -- ;
    restInFlexion = restInFlexionmilis / 1000;
  }
  encoderValueP = __HAL_TIM_GET_COUNTER(&htim1);
  // measure velocity, position
  update_encoder(&enc_instance_mot, &htim1);
  encoder_direction = enc_instance_mot.direction;
  if(enc_instance_mot.position < 0 ){
    
    
    encoder_position = abs((int)enc_instance_mot.position);
    
  }else if(enc_instance_mot.position >=0){
    encoder_position = enc_instance_mot.position; 
  }
  encoder_velocity = enc_instance_mot.velocity ;
  if(levelArmCalibrationDone == 1){
    levelArmAngle = encoder_position / levelArmPulsePerDegree ;
    
  }
  
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
#if (INCLUDE_xTaskGetSchedulerState == 1 )
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
  {
#endif /* INCLUDE_xTaskGetSchedulerState */
  xPortSysTickHandler();
#if (INCLUDE_xTaskGetSchedulerState == 1 )
  }
#endif /* INCLUDE_xTaskGetSchedulerState */
  /* USER CODE BEGIN SysTick_IRQn 1 */
  
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles ADC1, ADC2 and ADC3 global interrupts.
  */
void ADC_IRQHandler(void)
{
  /* USER CODE BEGIN ADC_IRQn 0 */
  
  /* USER CODE END ADC_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  HAL_ADC_IRQHandler(&hadc2);
  /* USER CODE BEGIN ADC_IRQn 1 */
  
  /* USER CODE END ADC_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
  */
void TIM1_UP_TIM10_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */
  
  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  HAL_TIM_IRQHandler(&htim10);
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */
  
  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/**
  * @brief This function handles TIM1 trigger and commutation interrupts and TIM11 global interrupt.
  */
void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_TRG_COM_TIM11_IRQn 0 */
  
  /* USER CODE END TIM1_TRG_COM_TIM11_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  HAL_TIM_IRQHandler(&htim11);
  /* USER CODE BEGIN TIM1_TRG_COM_TIM11_IRQn 1 */
  
  /* USER CODE END TIM1_TRG_COM_TIM11_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
  
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
  
  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */
  modbusErrorCounter = 0;
  uint32_t tmp_flag = __HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE);
  uint32_t tmp_it_source = __HAL_UART_GET_IT_SOURCE(&huart3, UART_IT_RXNE);
  
  if((tmp_flag != RESET) && (tmp_it_source != RESET)) {
    pxMBFrameCBByteReceived();
    __HAL_UART_CLEAR_PEFLAG(&huart3); 
    
    if(init_bit == 0){
      init_bit = 1;
      modbus_state_bit = 0;
    }else{
      modbus_state_bit =1;
    }
    /*    */
    return;
  }else{
    modbus_state_bit = 0 ;
  }
  
  if((__HAL_UART_GET_FLAG(&huart3, UART_FLAG_TXE) != RESET) &&(__HAL_UART_GET_IT_SOURCE(&huart3, UART_IT_TXE) != RESET)) {
    pxMBFrameCBTransmitterEmpty();    
    return ;
  }
  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */
  
  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */
  
  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(ENCODER_BMW_Pin);
  HAL_GPIO_EXTI_IRQHandler(IR_Sensor_Pin);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */
  
  /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
  * @brief This function handles TIM8 trigger and commutation interrupts and TIM14 global interrupt.
  */
void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_TRG_COM_TIM14_IRQn 0 */
  
  /* USER CODE END TIM8_TRG_COM_TIM14_IRQn 0 */
  HAL_TIM_IRQHandler(&htim8);
  /* USER CODE BEGIN TIM8_TRG_COM_TIM14_IRQn 1 */
  
  /* USER CODE END TIM8_TRG_COM_TIM14_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */
  if(__HAL_TIM_GET_FLAG(&htim7, TIM_FLAG_UPDATE) != RESET && __HAL_TIM_GET_IT_SOURCE(&htim7, TIM_IT_UPDATE) !=RESET) {
    __HAL_TIM_CLEAR_IT(&htim7, TIM_IT_UPDATE);
    if (!--downcounter)
      pxMBPortCBTimerExpired();
  }
  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */
  
  /* USER CODE END TIM7_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream0 global interrupt.
  */
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */
  
  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */
  
  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream2 global interrupt.
  */
void DMA2_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream2_IRQn 0 */
  
  /* USER CODE END DMA2_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA2_Stream2_IRQn 1 */
  
  /* USER CODE END DMA2_Stream2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
