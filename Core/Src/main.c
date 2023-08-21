/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can_utils.h"

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

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */


void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan){
    C620_WhenTxMailboxCompleteCallbackCalled(hcan);
    CANLib_WhenTxMailbox0_1_2CompleteCallbackCalled(hcan);
}

void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *hcan){
    C620_WhenTxMailboxAbortCallbackCalled(hcan);
    CANLib_WhenTxMailbox0_1_2AbortCallbackCalled(hcan);
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan){
    C620_WhenTxMailboxCompleteCallbackCalled(hcan);
    CANLib_WhenTxMailbox0_1_2CompleteCallbackCalled(hcan);
}

void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *hcan){
    C620_WhenTxMailboxAbortCallbackCalled(hcan);
    CANLib_WhenTxMailbox0_1_2AbortCallbackCalled(hcan);
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan){
    C620_WhenTxMailboxCompleteCallbackCalled(hcan);
    CANLib_WhenTxMailbox0_1_2CompleteCallbackCalled(hcan);
}

void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *hcan){
    C620_WhenTxMailboxAbortCallbackCalled(hcan);
    CANLib_WhenTxMailbox0_1_2AbortCallbackCalled(hcan);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
    WhenCANRxFifo0MsgPending(hcan, &num_of_devices);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan){
    C620_WhenCANRxFifo1MsgPending(hcan);
}
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
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
    setbuf(stdout, NULL);
    setbuf(stderr, NULL);

    /*
     * ===== CANLib Settings =====
     * */
    printf("Start Initializing CAN System:Begin\n\r");
    CAN_SystemInit(&hcan2); // F7のCAN通信のinit

    num_of_devices.mcmd3 = 1;
    num_of_devices.mcmd4 = 0;
    num_of_devices.air = 0;
    num_of_devices.servo = 0;

    printf("Start Initializing CAN System:End\n\r");
    CAN_WaitConnect(&num_of_devices);

    mcmd_handlers[0].device.node_type = NODE_MCMD3;
    mcmd_handlers[0].device.node_id = 1;
    mcmd_handlers[0].device.device_num = 0;
    mcmd_handlers[0].ctrl_param.ctrl_type = MCMD_CTRL_DUTY;
    mcmd_handlers[0].ctrl_param.feedback = MCMD_FB_ENABLE;
    mcmd_handlers[0].fb_type = MCMD_FB_DUTY;
    mcmd_handlers[0].enc_dir = MCMD_DIR_FW;
    mcmd_handlers[0].rot_dir = MCMD_DIR_FW;
    mcmd_handlers[0].calib = CALIBRATION_DISABLE;

    MCMD_init(&mcmd_handlers[0]);
    MCMD_Calib(&mcmd_handlers[0]);  // キャリブレーションを行う
    MCMD_SetTarget(&mcmd_handlers[0], 0.0f);  // 目標値(0.0)を設定
    MCMD_Control_Enable(&mcmd_handlers[0]);  // 制御開始



    /*
    * ===== CANLib_RoboMas Settings =====
    * */
    Init_C620_CAN_System(&hcan1);  // Init CAN System for C620
    C620_Init(c620_dev_info_global, num_of_c620);

    for(int i=0; i<num_of_c620; i++) {
        c620_dev_info_global[i].device_id = i+1;  // 1スタートな事に注意
        c620_dev_info_global[i].ctrl_param.accel_limit = C620_ACCEL_LIMIT_ENABLE;
        c620_dev_info_global[i].ctrl_param.use_internal_offset = C620_USE_OFFSET_POS_ENABLE;
        c620_dev_info_global[i].ctrl_param.accel_limit_size = 5.0f;
        c620_dev_info_global[i].ctrl_param.ctrl_type = C620_CTRL_VEL;
        c620_dev_info_global[i].ctrl_param.pid.kp = 0.6f;
        c620_dev_info_global[i].ctrl_param.pid.ki = 0.08f;
        c620_dev_info_global[i].ctrl_param.pid.kd = 0.0f;
        c620_dev_info_global[i].ctrl_param.pid.kff = 0.0f;
    }
    for(int i=0; i<1; i++){
        C620_SetTarget(&c620_dev_info_global[i], 0.0f);  // MAX: 20.0
    }
    C620_WaitForConnect(c620_dev_info_global, 1);
    for(int i=0; i<1; i++){
        C620_ControlEnable(&(c620_dev_info_global[i]));
    }




  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
