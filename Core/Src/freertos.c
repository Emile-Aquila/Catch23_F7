/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include <usart.h>
//#include <unistd.h>

#include "can_utils.h"
#include "can.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticTimer_t osStaticTimerDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
rcl_publisher_t publisher;

actuator_msgs__msg__ActuatorMsg actuator_msg;
actuator_msgs__msg__ActuatorFeedback feedback_msg;

float _mros_duty_pre;
float _mros_target_duty;


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* USER CODE END Variables */
/* Definitions for mrosTask */
osThreadId_t mrosTaskHandle;
uint32_t mrosTaskBuffer[ 3500 ];
osStaticThreadDef_t mrosTaskControlBlock;
const osThreadAttr_t mrosTask_attributes = {
  .name = "mrosTask",
  .cb_mem = &mrosTaskControlBlock,
  .cb_size = sizeof(mrosTaskControlBlock),
  .stack_mem = &mrosTaskBuffer[0],
  .stack_size = sizeof(mrosTaskBuffer),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for LEDTask */
osThreadId_t LEDTaskHandle;
uint32_t LEDTaskBuffer[ 512 ];
osStaticThreadDef_t LEDTaskControlBlock;
const osThreadAttr_t LEDTask_attributes = {
  .name = "LEDTask",
  .cb_mem = &LEDTaskControlBlock,
  .cb_size = sizeof(LEDTaskControlBlock),
  .stack_mem = &LEDTaskBuffer[0],
  .stack_size = sizeof(LEDTaskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for C620Timer */
osTimerId_t C620TimerHandle;
osStaticTimerDef_t C620TimerControlBlock;
const osTimerAttr_t C620Timer_attributes = {
  .name = "C620Timer",
  .cb_mem = &C620TimerControlBlock,
  .cb_size = sizeof(C620TimerControlBlock),
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

#include <sys/time.h>


void pub_timer_callback(rcl_timer_t * timer, int64_t last_call_time){
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        RCSOFTCHECK(rcl_publish(&publisher, &feedback_msg, NULL));
    }
}

void subscription_callback(const void * msgin){
    const actuator_msgs__msg__ActuatorMsg * actuator_msg_pre = (const actuator_msgs__msg__ActuatorMsg * )msgin;
    actuator_msg.device = actuator_msg_pre->device;
    actuator_msg.target_value = actuator_msg_pre->target_value;
    actuator_msg.air_target = actuator_msg_pre->air_target;

    CAN_Device device_info = DeviceInfo_to_CAN_Device(&(actuator_msg.device));
    if(_mros_duty_pre != (float)(actuator_msg.target_value)){
        _mros_target_duty = (float)(actuator_msg.target_value);
        if(_mros_target_duty > 0.2f){
            _mros_target_duty = 0.2f;
        }else if(_mros_target_duty < -0.2f){
            _mros_target_duty = -0.2f;
        }
        if(device_info.node_type == NODE_MCMD4){
            MCMD_HandleTypedef* p_h_mcmd;
            for(int i=0; i<num_of_devices.mcmd4; i++){
                if((mcmd_handlers[i].device.device_num == device_info.device_num) &&
                   (mcmd_handlers[i].device.node_id == device_info.node_id)){
                    p_h_mcmd = &(mcmd_handlers[i]);
                    MCMD_SetTarget(p_h_mcmd, _mros_target_duty);
                    _mros_duty_pre = _mros_target_duty;
                    break;
                }
            }
        }
    }
}


/* USER CODE END FunctionPrototypes */

void StartMrosTask(void *argument);
void StartLEDTask(void *argument);
void C620TimerCallback(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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

  /* Create the timer(s) */
  /* creation of C620Timer */
  C620TimerHandle = osTimerNew(C620TimerCallback, osTimerPeriodic, NULL, &C620Timer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  osTimerStart(C620TimerHandle, 1);
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of mrosTask */
  mrosTaskHandle = osThreadNew(StartMrosTask, NULL, &mrosTask_attributes);

  /* creation of LEDTask */
  LEDTaskHandle = osThreadNew(StartLEDTask, NULL, &LEDTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartMrosTask */
/**
  * @brief  Function implementing the mrosTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartMrosTask */
void StartMrosTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartMrosTask */
    rmw_uros_set_custom_transport(
            true,
            (void *) &huart3,
            cubemx_transport_open,
            cubemx_transport_close,
            cubemx_transport_write,
            cubemx_transport_read);

    rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
    freeRTOS_allocator.allocate = microros_allocate;
    freeRTOS_allocator.deallocate = microros_deallocate;
    freeRTOS_allocator.reallocate = microros_reallocate;
    freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

    if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
        printf("Error on default allocators (line %d)\n", __LINE__);
    }
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // micro-ROS app
    rclc_support_t support;
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rcl_node_t node;


    // node setting
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));  //create init_options
    RCCHECK(rclc_node_init_default(&node, "f7_mros_node", "", &support));  // create node

    // create executor
    rclc_executor_t executor;
    unsigned int num_handlers = 2;
    RCCHECK(rclc_executor_init(&executor, &support.context, num_handlers, &allocator));


    // create subscriber
    rcl_subscription_t subscriber;
    const char* topic_name_sub = "mros_input";
    RCCHECK(rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(actuator_msgs, msg, ActuatorMsg), topic_name_sub));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &actuator_msg, &subscription_callback, ON_NEW_DATA));


    // create publisher
    const char* topic_name_pub = "mros_output";
    RCCHECK(rclc_publisher_init_default(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(actuator_msgs, msg, ActuatorFeedback), topic_name_pub));


    // create timer (for publisher)
    rcl_timer_t timer;
    const unsigned int timer_timeout = 25;
    RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), pub_timer_callback));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    while(1){
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        osDelay(1000);
    }

    // free resources
    RCCHECK(rcl_subscription_fini(&subscriber, &node));
    RCCHECK(rcl_publisher_fini(&publisher, &node))
    RCCHECK(rcl_node_fini(&node));
    /* USER CODE END StartMrosTask */
}

/* USER CODE BEGIN Header_StartLEDTask */
/**
* @brief Function implementing the LEDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLEDTask */
void StartLEDTask(void *argument)
{
  /* USER CODE BEGIN StartLEDTask */
    while (1){
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);  // LD2 (Blue)
        osDelay(2000);
    }
  /* USER CODE END StartLEDTask */
}

/* C620TimerCallback function */
void C620TimerCallback(void *argument)
{
  /* USER CODE BEGIN C620TimerCallback */
    C620_SendRequest(c620_dev_info_global, 1, 1000.0f, &hcan1);
  /* USER CODE END C620TimerCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

