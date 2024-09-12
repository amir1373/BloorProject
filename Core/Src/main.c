/* USER CODE BEGIN Header */
/**
******************************************************************************
* @file           : main.c
* @brief          : Main program body
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mb.h"
#include "mbport.h"
#include "mbutils.h"


#include <stdio.h>
#include <string.h>
#include "eeprom.h"
#include <math.h>
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;

/* Definitions for MotorTask */
osThreadId_t MotorTaskHandle;
const osThreadAttr_t MotorTask_attributes = {
  .name = "MotorTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ModbusTask */
osThreadId_t ModbusTaskHandle;
const osThreadAttr_t ModbusTask_attributes = {
  .name = "ModbusTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DAC_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM8_Init(void);
static void MX_USB_OTG_FS_USB_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM11_Init(void);
static void MX_ADC2_Init(void);
void MotorTaskFunc(void *argument);
void ModbusTaskFunc(void *argument);

/* USER CODE BEGIN PFP */

uint8_t testSerialData[10];
// modbus input register start address
#define REG_INPUT_START 1
// modbus number of input registers
#define REG_INPUT_NREGS 256

static USHORT usRegInputStart = REG_INPUT_START;
static USHORT usRegInputBuf[REG_INPUT_NREGS];
#define REG_HOLDING_START               1
#define REG_HOLDING_NREGS               256
static USHORT   usRegHoldingStart = REG_HOLDING_START;
static USHORT   usRegHoldingBuf[REG_HOLDING_NREGS];
# define  M_DISCRETE_INPUT_START         0
# define  M_DISCRETE_INPUT_NDISCRETES    16
////////////////////
# define  S_COIL_START                   0
# define  S_COIL_NCOILS                  64
static USHORT   usSCoilStart     = S_COIL_START;
static BOOL    ucSCoilBuf[S_COIL_NCOILS]  ;
//////////////////////////////////////////////////////////////////
# define  S_DISCRETE_INPUT_START         0
# define  S_DISCRETE_INPUT_NDISCRETES    16
static USHORT   usSDiscInStart            = S_DISCRETE_INPUT_START;
static UCHAR    ucSDiscInBuf[S_DISCRETE_INPUT_NDISCRETES];


int init_bit = 0;
int modbus_state_bit = 0;

int flag_column_up = 0;
int flag_column_down = 0;
int flag_arm_cw = 0;
int flag_arm_ccw = 0;
int stop_flag = 0;
int column_flag = 0;
int action_flag = 0;
int encoder_ir = 0;
int encoder_ir_counter = 0;
int encoder_bmw = 0;
int flag_encoder_ir = 0;
int flag_encoder_bmw = 0;
int time_encoder_ir = 0;
int time_encoder_bmw=0;
int encoder_ir_pstate = 0;

/////
int maxPWMSpeed = 2100;
int minPWMSpeed = 700;
//////
int columnPWMMax = 2100;
int columnPWMMin = 500;



uint16_t VirtAddVarTab[64] = {0x10,0x20,0x30,0x40,0x50,0x60,0x70,0x80,0x90,0xA0,0xB0,0xC0,0xD0,0xE,0xF0,
0x100,0x110,0x120,0x130,0x140,0x150,0x160,0x170,0x180,0x190,0x1A0,0x1B0,0x1C0,
0x1D0,0x1E0,0x1F0,0x200,0x210,0x220,0x230,0x240,0x250,0x260,0x270,0x280,0x290,0x2A0,0x2B0,
0x2C0,0x2D0,0x2E0,0x2F0,0x300,0x310,0x320,0x330,0x340,0x350,0x360,0x370,0x380,0x390,0x3A0,0x3B0,0x3C0,0x3D0,0x3E0,0x3F0,0x400
};


uint16_t column_position = 0;
uint16_t arm_speed = 0;
uint16_t arm_time_m = 0;
uint16_t arm_time_s = 0;
uint16_t arm_degree = 0;
uint16_t arm_repeat = 0;
uint16_t arm_pulse_encode = 0;
uint16_t column_pulse_encode = 0;
uint16_t virginity_bit= 0;
int flash_state = 0;
int flash_page_change = 0;
int eepromResetBit = 0;
int MOTOR_EN = 0;
int counter_column = 0;
int EncoderValue1 = 0;
int patient_flag = 0;

uint16_t alarm_column = 0;
uint16_t alarm_level_arm = 0;
uint16_t error_reset = 0;
uint16_t levelArmCalibrationFlag = 0;
uint16_t level_arm_calibration_value1 = 0;
uint16_t level_arm_calibration_value2 = 0;
uint16_t speedMeasurementFlag1 = 0;
uint16_t speedMeasurementFlag2 = 0;
uint16_t speedMeasurementFlag3 = 0;
uint16_t pulseTimeLevelArm1 = 0;
uint16_t pulseTimeLevelArm2 = 0;
uint16_t pulseTimeColumn1 = 0;
uint16_t pulseTimeColumn2 = 0;
uint16_t levelArmMaxSpeed1 = 0;
uint16_t levelArmMaxSpeed2 = 0;
uint16_t levelArmMaxSpeedTotal = 0;
uint16_t levelArmMinSpeed1 = 0;
uint16_t levelArmMinSpeed2 = 0;
uint16_t levelArmMinSpeedTotal = 0;
uint16_t levelArmPulsePerDegree = 0;
uint16_t columnCalibration = 0;
uint16_t columnMinHeight  = 0;
uint16_t columnMaxHeight = 0;
uint16_t columnMaxCM = 0;
uint16_t columnMaxPulse = 0;
float columnPulsePerCm  = 0;
uint16_t columnPulsePerCmee = 0;
uint16_t levelArmCalibrationDone = 0;
uint16_t columnCalibrationDone = 0;
uint16_t protectColumnTime = 0;
uint16_t protectLevelArmTime = 0;
uint16_t totalWorkTime = 0;
uint16_t columnPosition = 0;
float columnPositionFloat = 0;

uint16_t restInFlexion = 0;
uint16_t restInExtention = 0;
uint16_t timeOrRepeat = 0;
uint16_t extentionSpeed = 0;
uint16_t flextionSpeed = 0;
uint16_t AID = 0;
uint16_t AIDROM = 0;
uint16_t ADD = 0;
uint16_t ADDROM = 0;
uint16_t ASR = 0;
uint16_t ASRROM = 0;
uint16_t AR = 0;
uint16_t ARROM = 0;
int columnPositionOld = 0 ;
int irEncodeDir =0 ;
int columnIsMoving = 0;
int bootFlag = 0;
int BMWencoderDir = 0;
int isLevelArmMoving = 0;
int levelArmAngle = 0;
int ROM = 0;
int timeRepeatMode = 0;
int startProcess = 0;
int timeOrRepeatRemaining = 0;
int timeOrRepeatRemainingMilis = 0;
int test = 2;
int actionDirection = 0;
int restInExtentionmilis = 0;
int restInFlexionmilis = 0;
int endProcess = 0;
int started = 0;
int negativeMove = 0;

int encoderValueP; 

int encoder_position = 0;
int encoder_velocity = 0;
int encoder_direction = 0;
int armFlag = 0;
int chartCounter = 0;
int torqueValue = 0;
int totalWorkTimeCounter = 0;
int totalWorkTimeWriteFlag = 0;

int columnAutoUpFlag = 0;
int columnAutoDownFlag = 0;
int columnAutoUpCounter = 0;
int columnAutoDownCounter = 0;
int operationalSpeedExtention = 0;
int operationalSpeedflexion = 0;
int columnProtectCounter = 0;
int levelArmProtectCounter = 0;
int armIsMoving = 0;
int columnProtectTimeMilis = 0;
int levelArmProtectTimeMilis = 0;
int columnPositionCounter = 0;
int columnStall = 0;
int columnStopFlag = 0;
int levelArmStall = 0;
int levelArmStopFlag  = 0;

int modbusErrorCounter = 0;
int modbusErrorTime = 10000;
int modbusErrorFlag = 0;
int buzzerState = 0;


typedef struct{
  int16_t velocity;
  int64_t position;
  uint32_t last_counter_value;
  int direction;
  int isMoving;
}encoder_instance;

encoder_instance enc_instance_mot;


eMBErrorCode eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs ){
  eMBErrorCode    eStatus = MB_ENOERR;
  int             iRegIndex;
  
  if( ( usAddress >= REG_INPUT_START )&& ( usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS ) ){
    iRegIndex = ( int )( usAddress - usRegInputStart );
    while( usNRegs > 0 ){
      *pucRegBuffer++ =
        ( unsigned char )( usRegInputBuf[iRegIndex] >> 8 );
      *pucRegBuffer++ =
        ( unsigned char )( usRegInputBuf[iRegIndex] & 0xFF );
      iRegIndex++;
      usNRegs--;
    }
    
    //HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
    //     HAL_GPIO_TogglePin(LED_ACK_GPIO_Port, LED_ACK_Pin);
    // HAL_GPIO_WritePin(LED_ACK_GPIO_Port, LED_ACK_Pin,GPIO_PIN_SET);
  }
  else
  {
    //HAL_GPIO_TogglePin(LED_ACK_GPIO_Port, LED_ACK_Pin);
    // HAL_GPIO_WritePin(LED_ACK_GPIO_Port, LED_ACK_Pin,GPIO_PIN_RESET);
    eStatus = MB_ENOREG;			
  }
  
  return eStatus;
}

eMBErrorCode eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs,eMBRegisterMode eMode ){
  eMBErrorCode    eStatus = MB_ENOERR;
  int  iRegIndex;
  
  if( ( usAddress >= REG_HOLDING_START ) && ( usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS ) ){
    iRegIndex = ( int )( usAddress - usRegHoldingStart );
    switch ( eMode ){
    case MB_REG_READ:
      while( usNRegs > 0 ){
        *pucRegBuffer++ = ( unsigned char )( usRegHoldingBuf[iRegIndex] >> 8 );
        *pucRegBuffer++ = ( unsigned char )( usRegHoldingBuf[iRegIndex] & 0xFF );
        iRegIndex++;
        usNRegs--;
      }
      break;
      
    case MB_REG_WRITE:
      while( usNRegs > 0 )
      {
        usRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
        usRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
        iRegIndex++;
        usNRegs--;
      }
    }
  }
  else
  {
    eStatus = MB_ENOREG;
  }
  return eStatus;
}


eMBErrorCode eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils,eMBRegisterMode eMode ){
  eMBErrorCode    eStatus = MB_ENOERR;
  USHORT          iRegIndex , iRegBitIndex , iNReg;
  UCHAR *         pucCoilBuf;
  USHORT          COIL_START;
  USHORT          COIL_NCOILS;
  USHORT          usCoilStart;
  iNReg =  usNCoils / 8 + 1;
  
  pucCoilBuf = ucSCoilBuf;
  COIL_START = S_COIL_START;
  COIL_NCOILS = S_COIL_NCOILS;
  usCoilStart = usSCoilStart;
  
  /* it already plus one in modbus function method. */
  usAddress--;
  
  if( ( usAddress >= COIL_START ) &&( usAddress + usNCoils <= COIL_START + COIL_NCOILS ) ){
    iRegIndex = (USHORT) (usAddress - usCoilStart) / 8;
    iRegBitIndex = (USHORT) (usAddress - usCoilStart) % 8;
    switch ( eMode )
    {
      /* read current coil values from the protocol stack. */
    case MB_REG_READ:
      while (iNReg > 0){
        *pucRegBuffer++ = xMBUtilGetBits(&pucCoilBuf[iRegIndex++],iRegBitIndex, 8);
        iNReg--;
      }
      pucRegBuffer--;
      /* last coils */
      usNCoils = usNCoils % 8;
      /* filling zero to high bit */
      *pucRegBuffer = *pucRegBuffer << (8 - usNCoils);
      *pucRegBuffer = *pucRegBuffer >> (8 - usNCoils);
      break;
      
      /* write current coil values with new values from the protocol stack. */
    case MB_REG_WRITE:
      while (iNReg > 1){
        xMBUtilSetBits(&pucCoilBuf[iRegIndex++], iRegBitIndex, 8,*pucRegBuffer++);
        iNReg--;
      }
      /* last coils */
      usNCoils = usNCoils % 8;
      /* xMBUtilSetBits has bug when ucNBits is zero */
      if (usNCoils != 0)
      {
        xMBUtilSetBits(&pucCoilBuf[iRegIndex++], iRegBitIndex, usNCoils,*pucRegBuffer++);
      }
      break;
    }
  }
  else
  {
    eStatus = MB_ENOREG;
  }
  return eStatus;
}

eMBErrorCode eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete ){
  eMBErrorCode    eStatus = MB_ENOERR;
  USHORT          iRegIndex , iRegBitIndex , iNReg;
  UCHAR *         pucDiscreteInputBuf;
  USHORT          DISCRETE_INPUT_START;
  USHORT          DISCRETE_INPUT_NDISCRETES;
  USHORT          usDiscreteInputStart;
  iNReg =  usNDiscrete / 8 + 1;
  
  pucDiscreteInputBuf = ucSDiscInBuf;
  DISCRETE_INPUT_START = S_DISCRETE_INPUT_START;
  DISCRETE_INPUT_NDISCRETES = S_DISCRETE_INPUT_NDISCRETES;
  usDiscreteInputStart = usSDiscInStart;
  
  /* it already plus one in modbus function method. */
  usAddress--;
  
  if ((usAddress >= DISCRETE_INPUT_START)&& (usAddress + usNDiscrete    <= DISCRETE_INPUT_START + DISCRETE_INPUT_NDISCRETES)){
    iRegIndex = (USHORT) (usAddress - usDiscreteInputStart) / 8;
    iRegBitIndex = (USHORT) (usAddress - usDiscreteInputStart) % 8;
    
    while (iNReg > 0){
      *pucRegBuffer++ = xMBUtilGetBits(&pucDiscreteInputBuf[iRegIndex++],iRegBitIndex, 8);
      iNReg--;
    }
    pucRegBuffer--;
    /* last discrete */
    usNDiscrete = usNDiscrete % 8;
    /* filling zero to high bit */
    *pucRegBuffer = *pucRegBuffer << (8 - usNDiscrete);
    *pucRegBuffer = *pucRegBuffer >> (8 - usNDiscrete);
  }else{
    eStatus = MB_ENOREG;
  }
  
  return eStatus;
}

void setPWM(TIM_HandleTypeDef timer, uint32_t channel,uint16_t pulse){
  __HAL_TIM_SetCompare(&timer,channel,pulse);
  
}
void setARR(TIM_HandleTypeDef timer, uint32_t channel,uint16_t pulse){
  
  __HAL_TIM_SetAutoreload(&timer,pulse);
  //__HAL_TIM_SetCompare(&timer,channel,pulse/2);
}


// EXTI Line9 External Interrupt ISR Handler CallBackFun
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /*
  if(GPIO_Pin == IR_Sensor2_Pin)
  {
  if(flag_encoder_ir == 1){
  encoder_ir = encoder_ir + 1;
  flag_encoder_ir = 0 ;
  time_encoder_ir = TIM10->CNT ;
  
}else if(flag_encoder_ir == 0){
  
  flag_encoder_ir = 1 ;
  TIM10->CNT = 0;
  time_encoder_ir = TIM10->CNT ;
  
}
  
  
}
  */ 
  if(GPIO_Pin == ENCODER_BMW_Pin)
  {
    
    if(flag_encoder_bmw == 0){
      flag_encoder_bmw = 1 ;
      //TIM11->CNT = 0;
    }else if(flag_encoder_bmw == 1){
      flag_encoder_bmw = 0 ;
      encoder_bmw = encoder_bmw + 1 ;
      time_encoder_bmw = TIM11->CNT ;
      if(levelArmCalibrationDone == 1){
        if(encoder_bmw == levelArmPulsePerDegree){
          
          levelArmAngle = levelArmAngle + (1*BMWencoderDir);
          
          if(levelArmAngle < 0){
            levelArmAngle = 359 ;
          }
          if(levelArmAngle > 359){
            levelArmAngle = 0 ;
            
          }
          encoder_bmw = 0;
        }
      }
      
    }
    
  }   
  
}





//device and eeprom init function
void Device_Init(void){
  EE_ReadVariable(VirtAddVarTab[0], &virginity_bit);//reading fisrt boot
  
  if(virginity_bit!=1 ){
    EE_WriteVariable(VirtAddVarTab[0],1);
    EE_WriteVariable(VirtAddVarTab[1],75);
    EE_WriteVariable(VirtAddVarTab[2],270);
    EE_WriteVariable(VirtAddVarTab[3],77);
    EE_WriteVariable(VirtAddVarTab[4],110);
    EE_WriteVariable(VirtAddVarTab[5],0);
    EE_WriteVariable(VirtAddVarTab[6],120);
    EE_WriteVariable(VirtAddVarTab[7],17);
    EE_WriteVariable(VirtAddVarTab[8],2);
    EE_WriteVariable(VirtAddVarTab[9],0);
    EE_WriteVariable(VirtAddVarTab[10],3);
    EE_WriteVariable(VirtAddVarTab[11],0);
    EE_WriteVariable(VirtAddVarTab[12],75);
    EE_WriteVariable(VirtAddVarTab[14],5);
    EE_WriteVariable(VirtAddVarTab[15],5);
    EE_WriteVariable(VirtAddVarTab[16],5);
    EE_WriteVariable(VirtAddVarTab[17],6);
    EE_WriteVariable(VirtAddVarTab[18],6);
    EE_WriteVariable(VirtAddVarTab[19],5);
    EE_WriteVariable(VirtAddVarTab[20],5);
    EE_WriteVariable(VirtAddVarTab[21],5);
    EE_WriteVariable(VirtAddVarTab[22],5);
    EE_WriteVariable(VirtAddVarTab[23],1);
    EE_WriteVariable(VirtAddVarTab[24],5);
    EE_WriteVariable(VirtAddVarTab[25],5);
    EE_WriteVariable(VirtAddVarTab[26],5);
    EE_WriteVariable(VirtAddVarTab[27],5);
    
  }
  ///////////////////////////////////////////
  
  
  EE_ReadVariable(VirtAddVarTab[0],&virginity_bit);
  EE_ReadVariable(VirtAddVarTab[1],&columnMinHeight);
  EE_ReadVariable(VirtAddVarTab[2],&columnMaxHeight);
  EE_ReadVariable(VirtAddVarTab[3],&columnPulsePerCmee);
  columnPulsePerCm = (float)(columnPulsePerCmee) / 10 ;
  EE_ReadVariable(VirtAddVarTab[4],&columnMaxCM);
  EE_ReadVariable(VirtAddVarTab[5],&columnCalibrationDone);
  EE_ReadVariable(VirtAddVarTab[6],&levelArmPulsePerDegree);
  EE_ReadVariable(VirtAddVarTab[7],&levelArmMaxSpeedTotal);
  EE_ReadVariable(VirtAddVarTab[8],&levelArmMinSpeedTotal);
  EE_ReadVariable(VirtAddVarTab[9],&levelArmCalibrationDone);
  EE_ReadVariable(VirtAddVarTab[10],&protectLevelArmTime);
  EE_ReadVariable(VirtAddVarTab[11],&totalWorkTime);
  EE_ReadVariable(VirtAddVarTab[12],&columnPosition);
  columnPositionFloat = ((float)columnPosition)/100;
  EE_ReadVariable(VirtAddVarTab[14],&restInFlexion);
  EE_ReadVariable(VirtAddVarTab[15],&restInExtention);
  EE_ReadVariable(VirtAddVarTab[16],&timeOrRepeat);
  EE_ReadVariable(VirtAddVarTab[17],&extentionSpeed);
  EE_ReadVariable(VirtAddVarTab[18],&flextionSpeed);
  EE_ReadVariable(VirtAddVarTab[19],&AID);
  EE_ReadVariable(VirtAddVarTab[20],&AIDROM);
  EE_ReadVariable(VirtAddVarTab[21],&ADD);
  EE_ReadVariable(VirtAddVarTab[22],&ADDROM);
  EE_ReadVariable(VirtAddVarTab[23],&ASR);
  EE_ReadVariable(VirtAddVarTab[24],&ASRROM);
  EE_ReadVariable(VirtAddVarTab[25],&AR);
  EE_ReadVariable(VirtAddVarTab[26],&ARROM);  
  EE_ReadVariable(VirtAddVarTab[27],&protectColumnTime);    
  columnProtectTimeMilis = protectColumnTime * 1000;
  levelArmProtectTimeMilis = protectLevelArmTime * 1000;
  
  
  
  
}


//eeprom update
void EEPROM_UPDATE(void){
  
  
  EE_WriteVariable(VirtAddVarTab[0],virginity_bit);//virginity_bit
  osDelay(5);
  EE_ReadVariable(VirtAddVarTab[0],&virginity_bit);
  osDelay(5);
  EE_WriteVariable(VirtAddVarTab[1],columnMinHeight );
  osDelay(5);
  EE_ReadVariable(VirtAddVarTab[1],&columnMinHeight);
  osDelay(5);
  EE_WriteVariable(VirtAddVarTab[2],columnMaxHeight );
  osDelay(5);
  EE_ReadVariable(VirtAddVarTab[2],&columnMaxHeight);
  osDelay(5);
  EE_WriteVariable(VirtAddVarTab[3],columnPulsePerCmee );
  osDelay(5);
  EE_ReadVariable(VirtAddVarTab[3],&columnPulsePerCmee);
  osDelay(5);
  columnPulsePerCm = (float)(columnPulsePerCmee) / 10 ;
  osDelay(5);
  EE_WriteVariable(VirtAddVarTab[4],columnMaxCM );
  osDelay(5);
  EE_ReadVariable(VirtAddVarTab[4],&columnMaxCM);
  osDelay(5);
  EE_WriteVariable(VirtAddVarTab[5],columnCalibrationDone );
  osDelay(5);
  EE_ReadVariable(VirtAddVarTab[5],&columnCalibrationDone);
  osDelay(5);
  EE_WriteVariable(VirtAddVarTab[6],levelArmPulsePerDegree );
  osDelay(5);
  EE_ReadVariable(VirtAddVarTab[6],&levelArmPulsePerDegree);
  osDelay(5);
  EE_WriteVariable(VirtAddVarTab[7],levelArmMaxSpeedTotal );
  osDelay(5);
  EE_ReadVariable(VirtAddVarTab[7],&levelArmMaxSpeedTotal);
  osDelay(5);
  EE_WriteVariable(VirtAddVarTab[8],levelArmMinSpeedTotal );
  osDelay(5);
  EE_ReadVariable(VirtAddVarTab[8],&levelArmMinSpeedTotal);
  osDelay(5);
  EE_WriteVariable(VirtAddVarTab[9],levelArmCalibrationDone );
  osDelay(5);
  EE_ReadVariable(VirtAddVarTab[9],&levelArmCalibrationDone);
  osDelay(5);
  EE_WriteVariable(VirtAddVarTab[10],protectLevelArmTime );
  osDelay(5);
  EE_ReadVariable(VirtAddVarTab[10],&protectLevelArmTime);
  osDelay(5);
  EE_WriteVariable(VirtAddVarTab[11],totalWorkTime );
  osDelay(5);
  EE_ReadVariable(VirtAddVarTab[11],&totalWorkTime);
  osDelay(5);
  EE_WriteVariable(VirtAddVarTab[12],columnPosition);
  osDelay(5);
  EE_ReadVariable(VirtAddVarTab[12],&columnPosition);
  osDelay(5);
  columnPositionFloat = ((float)columnPosition)/100;
  osDelay(5);
  EE_WriteVariable(VirtAddVarTab[14],restInFlexion );
  osDelay(5);
  EE_ReadVariable(VirtAddVarTab[14],&restInFlexion);
  osDelay(5);
  EE_WriteVariable(VirtAddVarTab[15],restInExtention );
  osDelay(5);
  EE_ReadVariable(VirtAddVarTab[15],&restInExtention);
  osDelay(5);
  EE_WriteVariable(VirtAddVarTab[16],timeOrRepeat );
  osDelay(5);
  EE_ReadVariable(VirtAddVarTab[16],&timeOrRepeat);
  osDelay(5);
  EE_WriteVariable(VirtAddVarTab[17],extentionSpeed );
  osDelay(5);
  EE_ReadVariable(VirtAddVarTab[17],&extentionSpeed);
  osDelay(5);
  EE_WriteVariable(VirtAddVarTab[18],flextionSpeed);
  osDelay(5);
  EE_ReadVariable(VirtAddVarTab[18],&flextionSpeed);
  osDelay(5);
  EE_WriteVariable(VirtAddVarTab[19],AID );
  osDelay(5);
  EE_ReadVariable(VirtAddVarTab[19],&AID);
  osDelay(5);
  EE_WriteVariable(VirtAddVarTab[20],AIDROM );
  osDelay(5);
  EE_ReadVariable(VirtAddVarTab[20],&AIDROM);
  osDelay(5);
  EE_WriteVariable(VirtAddVarTab[21],ADD );
  osDelay(5);
  EE_ReadVariable(VirtAddVarTab[21],&ADD);
  osDelay(5);
  EE_WriteVariable(VirtAddVarTab[22],ADDROM );
  osDelay(5);
  EE_ReadVariable(VirtAddVarTab[22],&ADDROM);
  osDelay(5);
  EE_WriteVariable(VirtAddVarTab[23],ASR);
  osDelay(5);
  EE_ReadVariable(VirtAddVarTab[23],&ASR);
  osDelay(5);
  EE_WriteVariable(VirtAddVarTab[24],ASRROM);
  osDelay(5);
  EE_ReadVariable(VirtAddVarTab[24],&ASRROM);
  osDelay(5);
  EE_WriteVariable(VirtAddVarTab[25],AR);
  osDelay(5);
  EE_ReadVariable(VirtAddVarTab[25],&AR);
  osDelay(5);
  EE_WriteVariable(VirtAddVarTab[26],ARROM);
  osDelay(5);
  EE_ReadVariable(VirtAddVarTab[26],&ARROM);  
  osDelay(5);
  EE_WriteVariable(VirtAddVarTab[27],protectColumnTime);
  osDelay(5);
  EE_ReadVariable(VirtAddVarTab[27],&protectColumnTime);  
  columnProtectTimeMilis = protectColumnTime * 1000;
  levelArmProtectTimeMilis = protectLevelArmTime * 1000;
  
  
  
  
  
}
//eeprom update END



void update_encoder(encoder_instance *encoder_value, TIM_HandleTypeDef *htim)             
{
  uint32_t temp_counter = __HAL_TIM_GET_COUNTER(htim);
  static uint8_t first_time = 0;
  if(!first_time)
  {
    encoder_value ->velocity = 0;
    first_time = 1;
  }
  else
  {
    if(temp_counter == encoder_value ->last_counter_value)
    {
      encoder_value ->velocity = 0;
      
    }
    else if(temp_counter > encoder_value ->last_counter_value)
    {
      if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
      {
        
        encoder_value ->direction = -1;
        encoder_value ->velocity = -encoder_value ->last_counter_value -
          (__HAL_TIM_GET_AUTORELOAD(htim)-temp_counter);
      }
      else
      {
        encoder_value ->direction = 1;
        encoder_value ->velocity = temp_counter -                             
          encoder_value ->last_counter_value;
      }
    }
    else
    {
      if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
      {
        encoder_value ->direction = -1;
	encoder_value ->velocity = temp_counter -                           
          encoder_value ->last_counter_value;
      }
      else
      {
        encoder_value ->direction = 1;
	encoder_value ->velocity = temp_counter +
          (__HAL_TIM_GET_AUTORELOAD(htim) -                                   
           encoder_value ->last_counter_value);
      }
    }
  }
  encoder_value ->position += encoder_value ->velocity;
  encoder_value ->last_counter_value = temp_counter;                                         
}


////////holding register values 
/*
all the values should be subtracted - 1
start in HMI = W40050.0 key
end in HMI  = W40051.0 key
turn cw = W40007.0 key
turn ccw = W40008.0 key
ok = W40059.0 key
start/end val = W40054 value read
speed slider = W40022 value read
time key  = W40052.0 key
repeat key = W40053.0 key
time subtract = W40060.0 key
time increase = W40061.0 key
time value = W40055 value read
rest 1 subtract = W40062.0 key
rest 1 add = W40063.0 key
rest 1 value  = W40056 key
rest 2 subtract = W40064.0 key
rest 2 add = W40021.0 key
rest 2 value  = W40057 key
start total = W40014.0 key
column up = W40005.0 key
column down = W40006.0 key
all stop = W40020.0 key
column position  = W40058 value write from board
extention speed = W40028 value read 
rest time indicator = W40037 value write
run time indicator = W40041 value write
time gone =  W40036 value write
ASR PROM = W40065 value read
ASR TIME = W40066 value read
AID PROM = W40067 value read
AID TIME = W40068 value read
ADD PROM = W40069 value read
ADD TIME = W40070 value read
AR PROM = W40071 value read
AR TIME = W40072 value read
angle security time = W40073 value read
column protect time = W40074 value read
column calibrate set zero = W40075.0 key
column min  height = W40076 value read
column max height = W40077 value read
column max height cm = W40078 value read
level arm set zero  = W40079.0 key
level arm set max = W40080.0 key
level arm max pulse val 180 degree  = W40081 value read
start calibrate level arm key = W40082.0 key 
level arm speed 1 min = W40083 value write form board
level arm speed 1 max = W40084 value write form board
column set min key = = W40085.0
column set max key = = W40086.0
column pulse disply = W40087
column set cm = W40088.0
patient key =  W40101 key write to hmi
emergency key = W40102 key write to hmi 
alarm patient key = w40103
alarm emergency = W40104
error reset = W40110
columnProtect Setkey = W40111.0
levelarmprotectset ket = W40112.0
areverse setkey = W40113.0
addsetkey = W40114.0
aidsetkey = W40115.0
arsetkey = W40116.0
resetFactorysetkey = W40117.0
set_speed key = w40118.0
ROM RESET = w40119.0
time remaining = w40120
time in rest1 = w40121
time in rest1 = w40122
column state indicator = w40123
arm state indicator = w40124
stop Final = w40125
torque value = w40126
time value chart = w40127
level arm calibration done =  w40128
column calibration done =  w40129

//calibration indicator addresses 
columnLowIndicator =  w40130
columnLowCM =  w40131
columnMaxPulse = w40132
columnMaxCm = w40133
levelArmSetZero = w40134
levelArmMax = w40135
LevelArmGetSpeed = w40136

column calibration reset = w40137

column calibration reset = w40138


///line equation for speed
///y=116.66666666666667x+116.66666666666652





*/
//////////


///////eeprom address and variables
/*
VirtAddVarTab[1] = columnMinHeight;
VirtAddVarTab[2] = columnMaxHeight;
VirtAddVarTab[3] = (int)(columnPulsePerCm * 10);
VirtAddVarTab[4] = columnMaxCM;
VirtAddVarTab[5] = columnCalibrationDone;
VirtAddVarTab[6] = levelArmPulsePerDegree;
VirtAddVarTab[7] = levelArmMaxSpeedTotal;
VirtAddVarTab[8] = levelArmMinSpeedTotal;
VirtAddVarTab[9] = levelArmCalibrationDone;
VirtAddVarTab[0] = protectLevelArmTime;
VirtAddVarTab[11] = totalWorkTime;
VirtAddVarTab[12] = columnPosition;
VirtAddVarTab[14] = restInFlexion;
VirtAddVarTab[15] = restInExtention;
VirtAddVarTab[16] = timeOrRepeat;
VirtAddVarTab[17] = extentionSpeed;
VirtAddVarTab[18] = flextionSpeed;
VirtAddVarTab[19] = AID;
VirtAddVarTab[20] = AIDROM;
VirtAddVarTab[21] = ADD;
VirtAddVarTab[22] = ADDROM;
VirtAddVarTab[23] = ASR;
VirtAddVarTab[24] = ASRROM;
VirtAddVarTab[25] = AR;
VirtAddVarTab[26] = ARROM;






*/
///////eeprom address and variables END
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  MX_TIM7_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_DAC_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI3_Init();
  MX_TIM8_Init();
  MX_USB_OTG_FS_USB_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  HAL_FLASH_Unlock();
  flash_state = EE_Init();
  Device_Init();
  
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of MotorTask */
  MotorTaskHandle = osThreadNew(MotorTaskFunc, NULL, &MotorTask_attributes);

  /* creation of ModbusTask */
  ModbusTaskHandle = osThreadNew(ModbusTaskFunc, NULL, &ModbusTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */
  
  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */
  
  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  
  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */
  
  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */
  
  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */
  
  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */
  
  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */
  
  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */
  
  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */
  
  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */
  
  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */
  
  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */
  
  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */
  
  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */
  
  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */
  
  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */
  
  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */
  
  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */
  
  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */
  
  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  
  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */
  
  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */
  
  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 8-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2100-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
  
  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */
  
  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */
  
  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */
  
  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */
  
  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */
  
  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 0;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 65535;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */
  
  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */
  
  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */
  
  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */
  
  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */
  
  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */
  
  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 84-1;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 65535;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */
  
  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */
  
  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */
  
  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 84-1;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 65535;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */
  
  /* USER CODE END TIM11_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */
  
  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */
  
  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  
  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */
  
  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */
  
  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 19200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */
  
  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_USB_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */
  
  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */
  
  /* USER CODE END USB_OTG_FS_Init 1 */
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */
  
  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5|Relay2_Pin|LED_ACK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MOTOR_EN1_Pin|MOTOR_EN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USART1_DIR_GPIO_Port, USART1_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Emergency_Pin Input2_Pin */
  GPIO_InitStruct.Pin = Emergency_Pin|Input2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PE5 Relay2_Pin LED_ACK_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_5|Relay2_Pin|LED_ACK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTOR_EN1_Pin MOTOR_EN2_Pin */
  GPIO_InitStruct.Pin = MOTOR_EN1_Pin|MOTOR_EN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : IR_SENSOR2_Pin Patient_key_Pin */
  GPIO_InitStruct.Pin = IR_SENSOR2_Pin|Patient_key_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : IR_Sensor_Pin */
  GPIO_InitStruct.Pin = IR_Sensor_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(IR_Sensor_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ENCODER_BMW_Pin */
  GPIO_InitStruct.Pin = ENCODER_BMW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ENCODER_BMW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Buzzer_Pin */
  GPIO_InitStruct.Pin = Buzzer_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Buzzer_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA10 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USART1_DIR_Pin */
  GPIO_InitStruct.Pin = USART1_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USART1_DIR_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_MotorTaskFunc */
/**
* @brief  Function implementing the MotorTask thread.
* @param  argument: Not used
* @retval None
*/
/* USER CODE END Header_MotorTaskFunc */
void MotorTaskFunc(void *argument)
{
  /* USER CODE BEGIN 5 */
  
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  //HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_IC_Start(&htim1, TIM_CHANNEL_1);
  //HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start(&htim10);
  HAL_TIM_Base_Start(&htim11);
  
  
  /* Infinite loop */
  for(;;)
  {
    // time_encoder_bmw = TIM10->CNT ;
    //EncoderValue1 = (TIM1->CNT)>>2 ;

    osDelay(1);

    if(modbusErrorCounter > modbusErrorTime){
      modbusErrorFlag = 1;
    }else{
      modbusErrorFlag = 0;
    }
    EncoderValue1 = __HAL_TIM_GET_COUNTER(&htim1) ;
    HAL_GPIO_WritePin(Buzzer_GPIO_Port,Buzzer_Pin,(GPIO_PinState)buzzerState);
    HAL_GPIO_WritePin(MOTOR_EN1_GPIO_Port,MOTOR_EN1_Pin,(GPIO_PinState)MOTOR_EN);
    HAL_GPIO_WritePin(MOTOR_EN2_GPIO_Port,MOTOR_EN2_Pin,(GPIO_PinState)MOTOR_EN);
    column_flag = HAL_GPIO_ReadPin(Emergency_GPIO_Port,Emergency_Pin) ;
    patient_flag = HAL_GPIO_ReadPin(Patient_key_GPIO_Port,Patient_key_Pin) ;
    encoder_ir =  HAL_GPIO_ReadPin(IR_SENSOR2_GPIO_Port,IR_SENSOR2_Pin) ;
    usRegHoldingBuf[100] = patient_flag ;
    usRegHoldingBuf[101] = column_flag ;  
    usRegHoldingBuf[54] = timeOrRepeat ;
    usRegHoldingBuf[40] = levelArmAngle ;
    usRegHoldingBuf[55] = restInFlexion ;  
    usRegHoldingBuf[56] = restInExtention ;
    usRegHoldingBuf[57] = (int)columnPositionFloat ;
    usRegHoldingBuf[53] = levelArmAngle ;
    usRegHoldingBuf[36] = 0 ;
    usRegHoldingBuf[35] = ROM ;
    usRegHoldingBuf[119] = timeOrRepeatRemaining ;
    usRegHoldingBuf[120] = restInExtention ;
    usRegHoldingBuf[121] = restInFlexion ;
    usRegHoldingBuf[122] = column_flag ;
    usRegHoldingBuf[123] = armFlag ;  
    usRegHoldingBuf[125] = torqueValue ;
    usRegHoldingBuf[126] = chartCounter ;  
    usRegHoldingBuf[127] = levelArmCalibrationDone ;
    usRegHoldingBuf[128] = columnCalibrationDone ;      
    usRegHoldingBuf[88] = totalWorkTime ;  
    operationalSpeedExtention =(int) ((float)usRegHoldingBuf[21])*116.67+116.67;
    operationalSpeedflexion = (int)((float)usRegHoldingBuf[27])*116.67+116.67;
    
    if(totalWorkTimeWriteFlag == 1){//counting total work time
      totalWorkTimeWriteFlag = 0;
      EE_WriteVariable(VirtAddVarTab[11],totalWorkTime );
      EE_ReadVariable(VirtAddVarTab[11],&totalWorkTime);
    }
    
    if(column_flag == 1){//changing column and arm state and error
      armFlag = 0;
    }else{
      armFlag = 1;
      
    }
    
    if(bootFlag == 0){//reading extention speed and flexion speed
      bootFlag = 1 ;
      usRegHoldingBuf[21] = extentionSpeed ;  
      usRegHoldingBuf[27] = flextionSpeed ;
    }
    
    usRegHoldingBuf[103] = alarm_column;
    usRegHoldingBuf[104] = alarm_level_arm;
    error_reset = usRegHoldingBuf[109];
    
    //reading keys in quick start menue and all other menues
    
    if(usRegHoldingBuf[59] == 1){//reducing time or repeat
      usRegHoldingBuf[59] = 0 ;
      timeOrRepeat = timeOrRepeat - 1 ;
      EE_WriteVariable(VirtAddVarTab[16],timeOrRepeat );
      EE_ReadVariable(VirtAddVarTab[16],&timeOrRepeat);       
    }
    
    if(usRegHoldingBuf[60] == 1){//increasing flexion speed
      usRegHoldingBuf[60] = 0 ;
      timeOrRepeat = timeOrRepeat + 1 ;
      EE_WriteVariable(VirtAddVarTab[16],timeOrRepeat );
      EE_ReadVariable(VirtAddVarTab[16],&timeOrRepeat);      
    }
    
    if(usRegHoldingBuf[61] == 1){//decreasing rest in flexion
      usRegHoldingBuf[61] = 0 ;
      restInFlexion = restInFlexion - 1 ;
      EE_WriteVariable(VirtAddVarTab[14],restInFlexion );
      EE_ReadVariable(VirtAddVarTab[14],&restInFlexion);     
    }
    
    if(usRegHoldingBuf[62] == 1){//increasing rest in flexion
      usRegHoldingBuf[62] = 0 ;
      restInFlexion = restInFlexion + 1 ;
      EE_WriteVariable(VirtAddVarTab[14],restInFlexion );
      EE_ReadVariable(VirtAddVarTab[14],&restInFlexion);    
    }
    
    if(usRegHoldingBuf[63] == 1){//decreasing rest in extention
      usRegHoldingBuf[63] = 0 ;
      restInExtention = restInExtention - 1 ;
      EE_WriteVariable(VirtAddVarTab[15],restInExtention );
      EE_ReadVariable(VirtAddVarTab[15],&restInExtention);    
    }
    
    if(usRegHoldingBuf[20] == 1){//increeasing rest in extention
      usRegHoldingBuf[20] = 0 ;
      restInExtention = restInExtention + 1 ;
      EE_WriteVariable(VirtAddVarTab[15],restInExtention );
      EE_ReadVariable(VirtAddVarTab[15],&restInExtention);    
    }
    
    if(usRegHoldingBuf[117] == 1){//setting speeds into eeprom
      usRegHoldingBuf[117] = 0 ;
      extentionSpeed =  usRegHoldingBuf[21] ; 
      flextionSpeed = usRegHoldingBuf[27] ;
      EE_WriteVariable(VirtAddVarTab[17],extentionSpeed );
      EE_ReadVariable(VirtAddVarTab[17],&extentionSpeed);
      EE_WriteVariable(VirtAddVarTab[18],flextionSpeed);
      EE_ReadVariable(VirtAddVarTab[18],&flextionSpeed);
    }
    
    if(usRegHoldingBuf[52] == 1 && usRegHoldingBuf[51] == 0){//chanign between time and repeat mode
      usRegHoldingBuf[51] = 1 ;
      timeRepeatMode = 2; //repeat mode
    }else if(usRegHoldingBuf[51] == 0 ){
      timeRepeatMode = 1; //time mode
      usRegHoldingBuf[52] = 0;
    }
    
    if(usRegHoldingBuf[118] ==  1){//setting  reset mode
      usRegHoldingBuf[118] = 0 ;
      usRegHoldingBuf[49] = 0 ; 
      usRegHoldingBuf[50] = 0 ; 
      usRegHoldingBuf[51] = 0;
      usRegHoldingBuf[52] = 0;
    }
    
    
    
    
    //reading keys in quick start menue and all other menues END
    
    
    //saving column position in eeprom every one second
    if(columnPositionCounter > 3000){
      columnPositionCounter = 0;
      columnPosition = (int)(columnPositionFloat*100);
      EE_WriteVariable(VirtAddVarTab[12],columnPosition);
      osDelay(10);
      EE_ReadVariable(VirtAddVarTab[12],&columnPosition);
      osDelay(10);
      columnPositionFloat = ((float)columnPosition) / 100;
      
    }
    
    
    
    
    //saving column position in eeprom every one second END
    
    //level arm calibration process
    
    if(usRegHoldingBuf[78]==1){//calibration level arm set zero
      usRegHoldingBuf[133] = 1;  
      usRegHoldingBuf[78] = 0;
      encoder_bmw = 0;
      TIM1->CNT = 0;
      osDelay(100);
      enc_instance_mot.position = 0;
      osDelay(100);
      encoder_position = 0;
      osDelay(100);
      
      levelArmCalibrationFlag = 1;
      levelArmCalibrationDone = 0;
      EE_WriteVariable(VirtAddVarTab[9],levelArmCalibrationDone );
      EE_ReadVariable(VirtAddVarTab[9],&levelArmCalibrationDone);
    }
    
    if(levelArmCalibrationFlag == 1 && usRegHoldingBuf[79] == 1 ){
      usRegHoldingBuf[134] = 1;  
      usRegHoldingBuf[79] = 0;
      level_arm_calibration_value1 = encoder_position ; 
      level_arm_calibration_value2 = abs(encoder_position) ; 
      usRegHoldingBuf[80] = level_arm_calibration_value1;
      levelArmPulsePerDegree =  level_arm_calibration_value1 / 180 ;
      levelArmCalibrationFlag = 2;
      EE_WriteVariable(VirtAddVarTab[6],levelArmPulsePerDegree );
      osDelay(10);
      EE_ReadVariable(VirtAddVarTab[6],&levelArmPulsePerDegree);
      
      
    }
    
    if(levelArmCalibrationFlag == 2 && usRegHoldingBuf[81] == 1){
      usRegHoldingBuf[135] = 1;  
      
      if(column_flag == 1){
        alarm_column = 1;
      }
      
      if(alarm_column == 0){
        TIM1->CNT = 0;
        osDelay(100);
        enc_instance_mot.position = 0;
        osDelay(100);
        encoder_position = 0;
        osDelay(100);
        usRegHoldingBuf[81] = 0;
        levelArmCalibrationFlag = 3;
        osDelay(200);
        
      }
    }
    
    if(levelArmCalibrationFlag == 3 && column_flag == 0 && encoder_position < level_arm_calibration_value1){
      flag_arm_cw = 1;
      flag_arm_ccw = 0;
      MOTOR_EN = 1;
      
      setPWM(htim3, TIM_CHANNEL_4,maxPWMSpeed);
      setPWM(htim3, TIM_CHANNEL_3,0);  
      
      if(speedMeasurementFlag1 == 0){
        speedMeasurementFlag1 = 1;
        pulseTimeLevelArm1 = 0;
      }
      
    }
    
    if(levelArmCalibrationFlag == 3 && column_flag == 0 && encoder_position >= level_arm_calibration_value1){
      flag_arm_cw = 0;
      flag_arm_ccw = 0;
      MOTOR_EN = 0;
      TIM1->CNT = 0;
      osDelay(100);
      encoder_bmw = 0;
      enc_instance_mot.position = 0;
      osDelay(100);
      encoder_position = 0;
      osDelay(100);
      levelArmCalibrationFlag =4 ;
      setPWM(htim3, TIM_CHANNEL_4,0);
      setPWM(htim3, TIM_CHANNEL_3,0); 
      osDelay(200);
      
      if(speedMeasurementFlag1 == 1){
        speedMeasurementFlag1 = 0;
        levelArmMaxSpeed1 =  180/(pulseTimeLevelArm1/1000);
        
      }
      osDelay(3000);
    }
    
    if(levelArmCalibrationFlag == 4 && column_flag == 0 && encoder_position < level_arm_calibration_value1){
      flag_arm_cw = 0;
      flag_arm_ccw = 1;
      MOTOR_EN = 1;
      setPWM(htim3, TIM_CHANNEL_4,0);
      setPWM(htim3, TIM_CHANNEL_3,maxPWMSpeed);  
      if(speedMeasurementFlag2 == 0){
        speedMeasurementFlag2 = 1;
        pulseTimeLevelArm2 = 0;
      }
      
    }
    
    if(levelArmCalibrationFlag == 4 && column_flag == 0 && encoder_position >= level_arm_calibration_value1){
      flag_arm_cw = 0;
      flag_arm_ccw = 0;
      MOTOR_EN = 0;
      encoder_bmw = 0;
      TIM1->CNT = 0;
      osDelay(100);
      enc_instance_mot.position = 0;
      osDelay(100);
      encoder_position = 0;
      osDelay(100);
      levelArmCalibrationFlag =5 ;
      
      setPWM(htim3, TIM_CHANNEL_4,0);
      setPWM(htim3, TIM_CHANNEL_3,0);
      osDelay(200);
      if(speedMeasurementFlag2 == 1){
        speedMeasurementFlag2 = 0;
        levelArmMaxSpeed2 =  180/(pulseTimeLevelArm2/1000);
        levelArmMaxSpeedTotal = (levelArmMaxSpeed1 + levelArmMaxSpeed2)/2;
        usRegHoldingBuf[82] = levelArmMaxSpeedTotal;
        EE_WriteVariable(VirtAddVarTab[7],levelArmMaxSpeedTotal );
        osDelay(10);
        EE_ReadVariable(VirtAddVarTab[7],&levelArmMaxSpeedTotal);
      }
      
      osDelay(3000);
    }
    
    if(levelArmCalibrationFlag == 5 && column_flag == 0 && encoder_position < level_arm_calibration_value1){
      flag_arm_cw = 1;
      flag_arm_ccw = 0;
      MOTOR_EN = 1;
      setPWM(htim3, TIM_CHANNEL_4,minPWMSpeed);
      setPWM(htim3, TIM_CHANNEL_3,0);  
      if(speedMeasurementFlag1 == 0){
        speedMeasurementFlag1 = 1;
        pulseTimeLevelArm1 = 0;
      }
      
    }
    
    if(levelArmCalibrationFlag == 5 && column_flag == 0 && encoder_position >= level_arm_calibration_value1){
      flag_arm_cw = 0;
      flag_arm_ccw = 0;
      MOTOR_EN = 0;
      encoder_bmw = 0;
      TIM1->CNT = 0;
      osDelay(100);
      enc_instance_mot.position = 0;
      osDelay(100);
      encoder_position = 0;
      osDelay(100);
      levelArmCalibrationFlag =6 ;
      
      setPWM(htim3, TIM_CHANNEL_4,0);
      setPWM(htim3, TIM_CHANNEL_3,0);  
      osDelay(200);
      if(speedMeasurementFlag1 == 1){
        speedMeasurementFlag1 = 0;
        levelArmMinSpeed1 =  180/(pulseTimeLevelArm1/1000);
        
      }
      
      osDelay(3000);
    }
    
    if(levelArmCalibrationFlag == 6 && column_flag == 0 && encoder_position < level_arm_calibration_value1){
      flag_arm_cw = 0;
      flag_arm_ccw = 1;
      MOTOR_EN = 1;
      setPWM(htim3, TIM_CHANNEL_4,0);
      setPWM(htim3, TIM_CHANNEL_3,minPWMSpeed);  
      if(speedMeasurementFlag2 == 0){
        speedMeasurementFlag2 = 1;
        pulseTimeLevelArm2 = 0;
      }
      
    }
    
    if(levelArmCalibrationFlag == 6 && column_flag == 0 && encoder_position >= level_arm_calibration_value1){
      flag_arm_cw = 0;
      flag_arm_ccw = 0;
      MOTOR_EN = 0;
      encoder_bmw = 0;
      TIM1->CNT = 0;
      osDelay(100);
      enc_instance_mot.position = 0;
      osDelay(100);
      encoder_position = 0;
      osDelay(100);
      
      levelArmCalibrationFlag =0 ;
      setPWM(htim3, TIM_CHANNEL_4,0);
      setPWM(htim3, TIM_CHANNEL_3,0);  
      osDelay(200);
      
      if(speedMeasurementFlag2 == 1){
        speedMeasurementFlag2 = 0;
        levelArmMinSpeed2 =  180/(pulseTimeLevelArm2/1000);
        levelArmMinSpeedTotal = (levelArmMinSpeed1 + levelArmMinSpeed2)/2;
        usRegHoldingBuf[83] = levelArmMinSpeedTotal;
        EE_WriteVariable(VirtAddVarTab[8],levelArmMinSpeedTotal );
        osDelay(10);
        EE_ReadVariable(VirtAddVarTab[8],&levelArmMinSpeedTotal);
        
      }
      
      osDelay(3000);
      levelArmCalibrationDone = 1;
      EE_WriteVariable(VirtAddVarTab[9],levelArmCalibrationDone );
      osDelay(10);
      EE_ReadVariable(VirtAddVarTab[9],&levelArmCalibrationDone);
      
    }  
    
    //level arm calibration process END
    
    
    //column calibration process
    
    if(usRegHoldingBuf[74]==1){//calibrate column set zero  
      usRegHoldingBuf[74] = 0;
      columnCalibration = 1 ;
      columnCalibrationDone = 0;
      usRegHoldingBuf[129] = 1;
      EE_WriteVariable(VirtAddVarTab[5],columnCalibrationDone );
      EE_ReadVariable(VirtAddVarTab[5],&columnCalibrationDone);
    }
    
    if(columnCalibration == 1 && usRegHoldingBuf[84] == 1 ){
      usRegHoldingBuf[130] = 1;
      columnMinHeight =usRegHoldingBuf[75]; 
      columnCalibration =2 ;
      usRegHoldingBuf[84] = 0;
      encoder_ir_counter = 0;
      EE_WriteVariable(VirtAddVarTab[1],columnMinHeight );
      osDelay(10);
      EE_ReadVariable(VirtAddVarTab[1],&columnMinHeight);
      
      
    }
    
    if(columnCalibration == 2 && usRegHoldingBuf[85] == 0 ){
      
      usRegHoldingBuf[86] = encoder_ir_counter; 
      
    }
    
    if(columnCalibration == 2 && usRegHoldingBuf[85] == 1 ){
      usRegHoldingBuf[131] = 1;     
      columnCalibration =3 ;
      usRegHoldingBuf[85] = 0;
      usRegHoldingBuf[86] = encoder_ir_counter;
      columnCalibration =4 ;
      usRegHoldingBuf[85] = 0;
      columnMaxHeight = usRegHoldingBuf[76];
      EE_WriteVariable(VirtAddVarTab[2],columnMaxHeight );
      osDelay(10);
      EE_ReadVariable(VirtAddVarTab[2],&columnMaxHeight);
    }
    
    
    if(columnCalibration == 4 && usRegHoldingBuf[87] == 1 ){
      usRegHoldingBuf[132] = 1;  
      columnMinHeight =usRegHoldingBuf[75]; 
      columnCalibration =0 ;
      usRegHoldingBuf[87] = 0;
      columnMaxCM = usRegHoldingBuf[77];
      columnPulsePerCm = (float)((float)columnMaxHeight/ (float)(columnMaxCM - columnMinHeight));
      columnPulsePerCmee = (int)(columnPulsePerCm * 10);
      EE_WriteVariable(VirtAddVarTab[3],columnPulsePerCmee );
      osDelay(10);
      EE_ReadVariable(VirtAddVarTab[3],&columnPulsePerCmee);
      columnPulsePerCm = (float)(columnPulsePerCmee) / 10 ;
      columnCalibrationDone = 1;
      EE_WriteVariable(VirtAddVarTab[5],columnCalibrationDone );
      osDelay(10);
      EE_ReadVariable(VirtAddVarTab[5],&columnCalibrationDone);   
      
      columnPosition = columnMinHeight ; 
      
      EE_WriteVariable(VirtAddVarTab[12],(columnPosition*100));
      EE_ReadVariable(VirtAddVarTab[12],&columnPosition);
      columnPositionFloat = ((float)columnPosition)/100;
      encoder_ir_counter = 0;
    }
    
    //column calibration process END
    
    //reading column protect
    if(usRegHoldingBuf[110]==1){
      usRegHoldingBuf[110]=0;
      protectColumnTime = usRegHoldingBuf[73];
      
      EE_WriteVariable(VirtAddVarTab[27],protectColumnTime);
      EE_ReadVariable(VirtAddVarTab[27],&protectColumnTime);  
      columnProtectTimeMilis = protectColumnTime * 1000;
      
    }
    
    //reading column protect END
    
    //reading levelarm protect
    if(usRegHoldingBuf[111]==1){
      usRegHoldingBuf[111]=0;
      protectLevelArmTime = usRegHoldingBuf[72];
      EE_WriteVariable(VirtAddVarTab[10],protectLevelArmTime );
      EE_ReadVariable(VirtAddVarTab[10],&protectLevelArmTime);
      
      levelArmProtectTimeMilis = protectLevelArmTime * 1000;
      
    }
    
    //reading levelarm protect END
    
    //reading AR
    if(usRegHoldingBuf[112]==1){
      usRegHoldingBuf[112]=0;
      ARROM = usRegHoldingBuf[70];
      AR = usRegHoldingBuf[71];
      
      EE_WriteVariable(VirtAddVarTab[25],AR);
      EE_ReadVariable(VirtAddVarTab[25],&AR);
      EE_WriteVariable(VirtAddVarTab[26],ARROM);
      EE_ReadVariable(VirtAddVarTab[26],&ARROM);  
      
    }
    
    //reading AR protect END
    
    //reading ADD protect
    if(usRegHoldingBuf[113]==1){
      usRegHoldingBuf[113]=0;
      ADDROM = usRegHoldingBuf[68];
      ADD = usRegHoldingBuf[69];
      
      EE_WriteVariable(VirtAddVarTab[21],ADD );
      EE_ReadVariable(VirtAddVarTab[21],&ADD);
      EE_WriteVariable(VirtAddVarTab[22],ADDROM );
      EE_ReadVariable(VirtAddVarTab[22],&ADDROM);
      
      
    }
    
    //reading ADD protect END
    
    //reading ASR protect
    if(usRegHoldingBuf[115]==1){
      usRegHoldingBuf[115]=0;
      ASRROM = usRegHoldingBuf[64];
      ASR = usRegHoldingBuf[65];
      EE_WriteVariable(VirtAddVarTab[23],ASR);
      EE_ReadVariable(VirtAddVarTab[23],&ASR);
      EE_WriteVariable(VirtAddVarTab[24],ASRROM);
      EE_ReadVariable(VirtAddVarTab[24],&ASRROM);
      
    }
    
    //reading ASR protect END
    
    //reading AID protect
    if(usRegHoldingBuf[114]==1){
      usRegHoldingBuf[114]=0;
      AIDROM = usRegHoldingBuf[66];
      AID = usRegHoldingBuf[67];
      EE_WriteVariable(VirtAddVarTab[19],AID );
      EE_ReadVariable(VirtAddVarTab[19],&AID);
      EE_WriteVariable(VirtAddVarTab[20],AIDROM );
      EE_ReadVariable(VirtAddVarTab[20],&AIDROM);
      
    }
    
    //reading AID protect END
    
    //reading Reset Factory
    if(usRegHoldingBuf[116]==1){
      usRegHoldingBuf[116]=0;
      
      eepromResetBit = 1;
      
    }
    
    //reading Reset Factory END
    
    // stop key in start menu
    if(usRegHoldingBuf[124] == 1 && startProcess != 0 ){
      usRegHoldingBuf[124] = 0 ;
      startProcess = 0;
      flag_arm_cw = 0;
      flag_arm_ccw = 0;
      MOTOR_EN = 0;
      setPWM(htim3, TIM_CHANNEL_4,0);
      setPWM(htim3, TIM_CHANNEL_3,0);
      actionDirection = 0;
      started = 0;
      //negativeMove = 0;
      endProcess = 1;
      
    }
    
    
    // stop key in start menu END
    
    //changing work direction based on encoder direction
    if(encoder_direction == 1){
      negativeMove = 1;
    }
    
    else if(encoder_direction == -1){
      negativeMove = 0;
      
    }   
    
    //changing work direction based on encoder direction END
    
    
    //////work process
    if(columnCalibrationDone == 1 && levelArmCalibrationDone == 1){
      if(encoder_ir_counter != 0 && columnIsMoving == 0){
        columnPosition = (int)(columnPositionFloat*100);
        EE_WriteVariable(VirtAddVarTab[12],columnPosition);
        EE_ReadVariable(VirtAddVarTab[12],&columnPosition);
        columnPositionFloat = ((float)columnPosition)/100;
        usRegHoldingBuf[57] = (int) columnPositionFloat ;
        // encoder_ir_counter = 0;
      }
      
      if(usRegHoldingBuf[49] == 0 && usRegHoldingBuf[58] == 1){
        levelArmAngle = 0;
        TIM1->CNT = 0;
        osDelay(100);
        enc_instance_mot.position = 0;
        osDelay(100);
        encoder_position = 0;
        osDelay(100);        
        usRegHoldingBuf[58] = 0 ;
        usRegHoldingBuf[49] = 1 ; 
        usRegHoldingBuf[50] = 1 ;  
      }
      
      if(usRegHoldingBuf[49] ==  1 && usRegHoldingBuf[50] == 1 && usRegHoldingBuf[58] == 1){
        usRegHoldingBuf[58] = 0 ;
        ROM = levelArmAngle ;
        
      }
      
      if(usRegHoldingBuf[13] == 1 && startProcess == 0){
        usRegHoldingBuf[13] = 0;
        timeOrRepeatRemaining = timeOrRepeat;
        if(timeRepeatMode == 1){
          timeOrRepeatRemainingMilis = timeOrRepeat * 1000 * 60 ;
        }
        
        startProcess = 1;
        started  = 1;
        usRegHoldingBuf[13] = 0 ; 
      }
      
      if(timeOrRepeatRemaining > 0 && startProcess > 0){
        
        if(levelArmAngle > 0 && startProcess == 1){
          restInExtentionmilis = restInExtention * 1000;
          restInFlexionmilis = restInFlexion * 1000;
          startProcess = 2;
          if(negativeMove == 1){
            actionDirection = 1 ;
            
          }
          
          else{
            actionDirection = 0 ; 
          }
          
          if(actionDirection == 0){
            MOTOR_EN = 1;
            HAL_GPIO_WritePin(MOTOR_EN1_GPIO_Port,MOTOR_EN1_Pin,(GPIO_PinState)MOTOR_EN);
            HAL_GPIO_WritePin(MOTOR_EN2_GPIO_Port,MOTOR_EN2_Pin,(GPIO_PinState)MOTOR_EN);
            BMWencoderDir = -1 ;
            setPWM(htim3, TIM_CHANNEL_3,operationalSpeedExtention);
            setPWM(htim3, TIM_CHANNEL_4,0);
            armIsMoving = 1;
          }
          
          if(actionDirection == 1){
            MOTOR_EN = 1;
            HAL_GPIO_WritePin(MOTOR_EN1_GPIO_Port,MOTOR_EN1_Pin,(GPIO_PinState)MOTOR_EN);
            HAL_GPIO_WritePin(MOTOR_EN2_GPIO_Port,MOTOR_EN2_Pin,(GPIO_PinState)MOTOR_EN);
            BMWencoderDir = 1 ;
            setPWM(htim3, TIM_CHANNEL_3,0);
            setPWM(htim3, TIM_CHANNEL_4,operationalSpeedflexion);    
            armIsMoving = 1;
          } 
          
        } 
        
        
        if(levelArmAngle <= 0 && startProcess == 2){
          if(timeRepeatMode == 2){
            timeOrRepeatRemaining =  timeOrRepeatRemaining - 1;
          }
          
          flag_arm_cw = 0;
          flag_arm_ccw = 0;
          MOTOR_EN = 0;
          HAL_GPIO_WritePin(MOTOR_EN1_GPIO_Port,MOTOR_EN1_Pin,(GPIO_PinState)MOTOR_EN);
          HAL_GPIO_WritePin(MOTOR_EN2_GPIO_Port,MOTOR_EN2_Pin,(GPIO_PinState)MOTOR_EN);
          setPWM(htim3, TIM_CHANNEL_4,0);
          setPWM(htim3, TIM_CHANNEL_3,0);
          armIsMoving = 0;
          startProcess = 3;
        }
        
        if(startProcess == 3 && restInExtention ==0){
          startProcess = 4;
          EE_ReadVariable(VirtAddVarTab[14],&restInFlexion);
          EE_ReadVariable(VirtAddVarTab[15],&restInExtention);
          actionDirection = abs((actionDirection - 1)) ;
          if(actionDirection == 0){
            MOTOR_EN = 1;
            BMWencoderDir = -1 ;
            HAL_GPIO_WritePin(MOTOR_EN1_GPIO_Port,MOTOR_EN1_Pin,(GPIO_PinState)MOTOR_EN);
            HAL_GPIO_WritePin(MOTOR_EN2_GPIO_Port,MOTOR_EN2_Pin,(GPIO_PinState)MOTOR_EN);
            setPWM(htim3, TIM_CHANNEL_3,operationalSpeedflexion);
            setPWM(htim3, TIM_CHANNEL_4,0);  
            armIsMoving = 1;
            
          }
          
          if(actionDirection == 1){
            MOTOR_EN = 1;
            BMWencoderDir = 1 ;
            HAL_GPIO_WritePin(MOTOR_EN1_GPIO_Port,MOTOR_EN1_Pin,(GPIO_PinState)MOTOR_EN);
            HAL_GPIO_WritePin(MOTOR_EN2_GPIO_Port,MOTOR_EN2_Pin,(GPIO_PinState)MOTOR_EN);
            setPWM(htim3, TIM_CHANNEL_3,0);
            setPWM(htim3, TIM_CHANNEL_4,operationalSpeedExtention);   
            armIsMoving = 1;
          }
          
        }
        
        if(startProcess == 4 && levelArmAngle >= ROM){
          if(timeRepeatMode == 2){
            timeOrRepeatRemaining =  timeOrRepeatRemaining - 1;
          }
          
          flag_arm_cw = 0;
          flag_arm_ccw = 0;
          MOTOR_EN = 0;
          HAL_GPIO_WritePin(MOTOR_EN1_GPIO_Port,MOTOR_EN1_Pin,(GPIO_PinState)MOTOR_EN);
          HAL_GPIO_WritePin(MOTOR_EN2_GPIO_Port,MOTOR_EN2_Pin,(GPIO_PinState)MOTOR_EN);
          setPWM(htim3, TIM_CHANNEL_4,0);
          setPWM(htim3, TIM_CHANNEL_3,0);
          armIsMoving = 0;
          startProcess = 5; 
          
        }
        
        if(startProcess == 5 && restInFlexion ==0){
          startProcess = 1;
          EE_ReadVariable(VirtAddVarTab[14],&restInFlexion);
          EE_ReadVariable(VirtAddVarTab[15],&restInExtention);
          
        }
        
      }
      else if(timeOrRepeatRemaining <= 0 && started == 1){
        endProcess = 1;
        
      }
      
    }
    
    
    //////work process END    
    if(error_reset == 1){//resetting all alarms
      levelArmStall = 0;
      alarm_column = 0;
      error_reset = 0;
      alarm_level_arm = 0;
      columnStall = 0;
      columnStopFlag = 0;
      levelArmStopFlag = 0;
      usRegHoldingBuf[109] = error_reset;
    }
    
    //reading ir encode
    if((encoder_ir != encoder_ir_pstate) && flag_encoder_ir == 0){
      
      encoder_ir_pstate = encoder_ir;
      flag_encoder_ir = 1 ;
      //TIM10->CNT = 0;
      //time_encoder_ir = TIM10->CNT ;
      columnProtectCounter = 0;
      
      
    }
    
    if(flag_encoder_ir == 1 && (encoder_ir != encoder_ir_pstate)  ){
      encoder_ir_pstate = encoder_ir;
      encoder_ir_counter = encoder_ir_counter + 1;
      flag_encoder_ir = 0 ;
      columnPositionFloat = (columnPositionFloat + (((irEncodeDir * (1/(columnPulsePerCm)))))) ;
      columnPosition = (int)(columnPositionFloat*100);
      //time_encoder_ir = TIM10->CNT ;
      columnProtectCounter = 0;
      
      
    }
    
    //reading ir encode END
    
    
    //manual control keys
    if(levelArmCalibrationFlag <=2 && startProcess == 0){
      if(usRegHoldingBuf[19]==1){//stop key
        stop_flag = 1;
        //usRegHoldingBuf[19] = 0;
        
      }
      
      if(usRegHoldingBuf[4]==1){//column up
        if(columnAutoUpFlag == 0 ){
          columnAutoUpFlag =1;
          columnAutoUpCounter = 0;
        }
        flag_column_up = 1;
        //columnProtectCounter = 0;
        // usRegHoldingBuf[4] = 0;
        if(column_flag == 0){
          alarm_column = 2;
        }
        
      }else if(usRegHoldingBuf[4]==1 && columnAutoUpFlag ==1  && columnAutoUpCounter > 5000){
        flag_column_up = 1;
        // usRegHoldingBuf[4] = 0;
        if(column_flag == 0){
          alarm_column = 2;
        }
        
      }else if(usRegHoldingBuf[4]==0){
        columnAutoUpFlag =0;
        columnAutoUpCounter = 0;  
        
      }
      
      
      if(usRegHoldingBuf[5]==1){//column down
        if(columnAutoDownFlag == 0 ){
          columnAutoDownFlag =1;
          //columnAutoDownCounter = 0;
        }
        columnProtectCounter = 0;
        
        //usRegHoldingBuf[5] = 0;
        flag_column_down = 1;
        if(column_flag == 0){
          alarm_column = 2;
        }
        
      }else if(usRegHoldingBuf[5]==1 && columnAutoDownFlag ==1  && columnAutoDownCounter > 5000){
        flag_column_down = 1;
        // usRegHoldingBuf[4] = 0;
        if(column_flag == 0){
          alarm_column = 2;
        }
        
      }else if(usRegHoldingBuf[5]==0){
        columnAutoDownFlag =0;
        columnAutoDownCounter = 0;  
        
      }
      
      if(usRegHoldingBuf[7]==1){//arm counter clockwise
        //usRegHoldingBuf[7] = 0;
        flag_arm_cw = 1;
        if(column_flag == 1){
          alarm_column = 1;
        }
        
      }
      
      if(usRegHoldingBuf[6]==1){//arm  clockwise
        //  usRegHoldingBuf[6] = 0;
        flag_arm_ccw = 1;
        if(column_flag == 1){
          alarm_column = 1;
        }
        
      }
      
      if(column_flag == 0){
        
        if(flag_arm_cw == 1){
          flag_arm_cw = 0;
          flag_arm_ccw = 0;
          MOTOR_EN = 1;
          BMWencoderDir = 1 ;
          armIsMoving = 1;
          
          setPWM(htim3, TIM_CHANNEL_4,maxPWMSpeed);
          setPWM(htim3, TIM_CHANNEL_3,0);    
          
        }
        
        if(flag_arm_ccw == 1){
          flag_arm_cw = 0;
          flag_arm_ccw = 0;
          MOTOR_EN = 1;
          BMWencoderDir = -1 ;
          armIsMoving = 1;
          setPWM(htim3, TIM_CHANNEL_3,maxPWMSpeed);
          setPWM(htim3, TIM_CHANNEL_4,0);     
        }
        
      }
      
      if(column_flag == 1){
        
        if(flag_column_down == 1){
          irEncodeDir = -1 ;
          MOTOR_EN = 1;
          flag_column_up = 0;
          flag_column_down = 0;
          setPWM(htim3, TIM_CHANNEL_3,columnPWMMax);
          setPWM(htim3, TIM_CHANNEL_4,0);
          columnIsMoving = 1;
        }
        
        if(flag_column_up == 1){
          irEncodeDir = 1 ;
          MOTOR_EN = 1;
          flag_column_up = 0;
          flag_column_down = 0;
          setPWM(htim3, TIM_CHANNEL_4,columnPWMMax);
          setPWM(htim3, TIM_CHANNEL_3,0);
          columnIsMoving = 1;
          
        }
        
      }
      
      if(usRegHoldingBuf[6]==0 && usRegHoldingBuf[7]==0 && usRegHoldingBuf[4]==0 && usRegHoldingBuf[5]==0 && columnAutoUpFlag ==0 && columnAutoDownFlag ==0   ){
        stop_flag = 1;
      }    
      
      
      if(stop_flag == 1){
        stop_flag = 0;
        flag_column_up = 0;
        flag_column_down = 0;
        flag_arm_cw = 0;
        flag_arm_ccw = 0;
        armIsMoving = 0;
        MOTOR_EN = 0;
        columnAutoDownFlag = 0;
        columnAutoUpFlag = 0;
        setPWM(htim3, TIM_CHANNEL_4,0);
        setPWM(htim3, TIM_CHANNEL_3,0);
        
        if(columnIsMoving == 1){
          columnIsMoving = 0;
          columnProtectCounter = 0;
        }
        
        
      }
      
      
      if(columnStopFlag == 1){
        columnStopFlag = 0;
        flag_column_up = 0;
        flag_column_down = 0;
        MOTOR_EN = 0;
        columnAutoDownFlag = 0;
        columnAutoUpFlag = 0;
        setPWM(htim3, TIM_CHANNEL_4,0);
        setPWM(htim3, TIM_CHANNEL_3,0);
        
        if(columnIsMoving == 1){
          columnIsMoving = 0;
          columnProtectCounter = 0;
        }
      }
      
      
      if(levelArmStopFlag == 1){
        levelArmStopFlag = 0;
        flag_arm_cw = 0;
        flag_arm_ccw = 0;
        MOTOR_EN = 0;
        setPWM(htim3, TIM_CHANNEL_4,0);
        setPWM(htim3, TIM_CHANNEL_3,0);
        
        
        
        
      }
      
    }
    
    //manual control keys END
    
    if(eepromResetBit == 1){
      EE_WriteVariable(VirtAddVarTab[0],0 );//virginity_bit
      osDelay(50);
      Device_Init();
      eepromResetBit = 0;
      
      
    }
    
    
    //updating eeprom
    if(flash_page_change == 1){
      flash_page_change = 0;
      EEPROM_UPDATE();
      
    }
    
    
    //updating eeprom
    
    
    if(endProcess == 1){
      flag_arm_cw = 0;
      flag_arm_ccw = 0;
      MOTOR_EN = 0;
      setPWM(htim3, TIM_CHANNEL_4,0);
      setPWM(htim3, TIM_CHANNEL_3,0);
      startProcess = 0; 
      endProcess = 0;
      started = 0;
      
    }
    
    
    
    //column height protection 
    
    if(columnCalibrationDone == 1){
      if(((int)columnPositionFloat) >=columnMaxCM ){
        if(usRegHoldingBuf[4]==1){
          stop_flag = 1 ; 
          
        }
        
        
      }else if(((int)columnPositionFloat) <= columnMinHeight){
        if(usRegHoldingBuf[5]==1){
          stop_flag = 1 ;
          
        }
        
      }
      if(((int)columnPositionFloat) >columnMaxCM ||((int)columnPositionFloat) < columnMinHeight){
        alarm_column = 4;
        
      }
      
    }
    
    
    
    //column height protection END   
    
    //column calibration reset
    if(usRegHoldingBuf[136] == 1 ){
      usRegHoldingBuf[136] = 0 ;
      columnCalibrationDone = 0;
      osDelay(2);
      EE_WriteVariable(VirtAddVarTab[5],columnCalibrationDone );
      osDelay(10);
      EE_ReadVariable(VirtAddVarTab[5],&columnCalibrationDone);
      
    }
    
    
    //column calibration reset END
    
    //Arm calibration reset
    if(usRegHoldingBuf[137] == 1 ){
      usRegHoldingBuf[137] = 0 ;
      levelArmCalibrationDone = 0;
      osDelay(2);
      EE_WriteVariable(VirtAddVarTab[9],levelArmCalibrationDone );
      osDelay(10);
      EE_ReadVariable(VirtAddVarTab[9],&levelArmCalibrationDone);
      
    }
    
    
    //Arm calibration reset END
    
    //column protection time
    if(columnIsMoving== 0 ){
      columnProtectCounter = 0;
      
    }
    
    if(columnIsMoving == 1 && columnProtectCounter > columnProtectTimeMilis && encoder_ir_pstate == encoder_ir && columnStall ==0){
      alarm_column = 4;
      columnIsMoving =0;
      columnStall = 1 ;
    }
    if(columnStall == 1){
      columnStopFlag = 1 ;
      alarm_column = 4;
      
      
    }
    
    //column protection time END
    
    //arm protection
    
    if(armIsMoving == 0){
      levelArmProtectCounter = 0;
      
    }
    if(enc_instance_mot.velocity != 0){
      levelArmProtectCounter = 0;
      
    }
    
    if(armIsMoving == 1 && levelArmProtectCounter > levelArmProtectTimeMilis &&  enc_instance_mot.velocity == 0){
      levelArmStall  = 1;
      armIsMoving = 0;
      alarm_column = 8;
      
      
      
    }
    if(levelArmStall == 1){
      levelArmStopFlag = 1;
      alarm_column = 8;
      
    }
    
    
    //arm protection END
    
    
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_ModbusTaskFunc */
/**
* @brief Function implementing the ModbusTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ModbusTaskFunc */
void ModbusTaskFunc(void *argument)
{
  /* USER CODE BEGIN ModbusTaskFunc */
  /* Infinite loop */
  eMBErrorCode eStatus =  eMBInit (  MB_RTU ,  1 , 3 ,  19200 ,  MB_PAR_NONE  ) ; 
  eStatus =  eMBEnable ( ) ; 
  for(;;)
  {
    //HAL_UART_Transmit(&huart3, (uint8_t*)"test\r\n", 6, 50);
    eMBPoll();   
    
    // osDelay(2000);
    //  HAL_UART_Receive(&huart3,testSerialData,9,HAL_MAX_DELAY);
  }
  /* USER CODE END ModbusTaskFunc */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
