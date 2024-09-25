/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bmp280.h"
#include "uart_printf.h"
#include "stdio.h"
#include "math.h"
#include "i2c.h"
#include "adc.h"
#include "MATH.h"
#include "DS18B20.h"
#include "SX1278.h"
#include "spi.h"
#include "gpio.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define P0 101325.0  // Ap suất chuẩn ở mực nước biển (Pa)
#define ALTITUDE_CONSTANT 44330.0  // Hằng số cho tính toán độ cao
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
float altitude ,pressure, temperature;
uint16_t AD_RES = 0;
uint16_t soilMoisturePercent;

uint8_t Presence ;
float  Temperature;
uint8_t  Temp_byte1, Temp_byte2;
uint16_t TEMP;

BMP280_HandleTypedef bmp280;
// Lora
SX1278_hw_t SX1278_hw;
SX1278_t SX1278;

int master;
int ret;
char buffer[512];
int message;
int message_length;


/* USER CODE END Variables */
osThreadId ReadSensorTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartReadSensorTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

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
  /* definition and creation of ReadSensorTask */
  osThreadDef(ReadSensorTask, StartReadSensorTask, osPriorityNormal, 0, 256);
  ReadSensorTaskHandle = osThreadCreate(osThread(ReadSensorTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartReadSensorTask */
/**
  * @brief  Function implementing the ReadSensorTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartReadSensorTask */
void StartReadSensorTask(void const * argument)
{
  /* USER CODE BEGIN StartReadSensorTask */

//    bmp280_init_default_params(&bmp280.params);
//    bmp280.addr = BMP280_I2C_ADDRESS_0;
//    bmp280.i2c = &hi2c1;
//    // Khởi tạo BMP280, nếu thất bại thì hiển thị thông báo
//    while (!(bmp280_init(&bmp280, &bmp280.params))) {
//        printf("BMP280 initialization failed\n");
//        osDelay(1);
//    }
//    // Kiểm tra nếu khởi tạo thành công
//    if (bmp280.id == BMP280_CHIP_ID) {
//        printf("BMP280 init successfully\n");
//        osDelay(10);
//    }

		printf("Mode: Master\r\n");

		//initialize LoRa module
		SX1278_hw.dio0.port = DIO0_GPIO_Port;
		SX1278_hw.dio0.pin = DIO0_Pin;
		SX1278_hw.nss.port = NSS_GPIO_Port;
		SX1278_hw.nss.pin = NSS_Pin;
		SX1278_hw.reset.port = RST_GPIO_Port;
		SX1278_hw.reset.pin = RST_Pin;
		SX1278_hw.spi = &hspi3;

		SX1278.hw = &SX1278_hw;

		printf("Configuring LoRa module\r\n");
		SX1278_init(&SX1278, 434000000, SX1278_POWER_17DBM, SX1278_LORA_SF_7,
		SX1278_LORA_BW_125KHZ, SX1278_LORA_CR_4_5, SX1278_LORA_CRC_EN, 10);
		printf("Done configuring LoRaModule\r\n");


		ret = SX1278_LoRaEntryTx(&SX1278, 16, 2000);
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);




  /* Infinite loop */
  while(1)
  {

		printf("Master ...\r\n");
		osDelay(1000);
		printf("Sending package...\r\n");

		message_length = sprintf(buffer, "Hello %d", message);
		ret = SX1278_LoRaEntryTx(&SX1278, message_length, 2000);
		printf("Entry: %d\r\n", ret);

		printf("Sending %s\r\n", buffer);
		ret = SX1278_LoRaTxPacket(&SX1278, (uint8_t*) buffer,
				message_length, 2000);
		message += 1;

		printf("Transmission: %d\r\n", ret);
		printf("Package sent...\r\n");

//     temperature = bmp280_read_temperature(&bmp280);
//     pressure = bmp280_read_pressure(&bmp280);
//     altitude = ALTITUDE_CONSTANT * (1 - pow(pressure / P0, 1 / 5.255));
//     printf("Temperature: %.2f C  Pressure: %.2f Pa  Altitude: %.2f m \n", temperature, pressure, altitude );
//     osDelay(500);

//      Start ADC Conversion
//      Pass (The ADC Instance, Result Buffer Address, Buffer Length)
//      HAL_ADC_Start_DMA(&hadc1, &AD_RES, 1);
//      soilMoisturePercent = MAP(AD_RES, 0, 3727, 0, 100);
//      osDelay(1);
//      printf("soilMoisturePercent: %d %%\n",100 - soilMoisturePercent);
//      osDelay(500);

//     Presence = DS18B20_Start ();
//     DS18B20_Write (0xCC);  // skip ROM
//     DS18B20_Write (0x44);  // convert t
//
//     Presence = DS18B20_Start ();
//     DS18B20_Write (0xCC);  // skip ROM
//     DS18B20_Write (0xBE);  // Read Scratch-pad
//
//     Temp_byte1 = DS18B20_Read();
//     Temp_byte2 = DS18B20_Read();
//     TEMP = ((Temp_byte2<<8))|Temp_byte1;
//     Temperature = (float)TEMP/16.0;  // resolution is 0.0625
//
//     osDelay(500);
//     printf("Temperature_BMP280: %.2f C  Temperature_DS18B20: %.2f C\n", temperature, Temperature);

  
  }
  /* USER CODE END StartReadSensorTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
