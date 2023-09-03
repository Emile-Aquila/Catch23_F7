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
#include <rosidl_runtime_c/primitives_sequence_functions.h>

#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <actuator_msgs/msg/c620_feedback.h>
#include <usart.h>

#include "math_utils.h"
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
rcl_publisher_t publisher_mcmd;
rcl_publisher_t publisher_c620_r, publisher_c620_theta;


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* USER CODE END Variables */
/* Definitions for mrosTask */
osThreadId_t mrosTaskHandle;
uint32_t mrosTaskBuffer[ 4500 ];
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


void pub_timer_callback_mcmd(rcl_timer_t * timer, int64_t last_call_time){
    RCLC_UNUSED(last_call_time);
    actuator_msgs__msg__ActuatorFeedback feedback_msg;

    if (timer != NULL) {
        if(num_of_devices.mcmd3 != 0 || num_of_devices.mcmd4 != 0){
            uint8_t num_of_mcmd = num_of_devices.mcmd3 + num_of_devices.mcmd4;
            for(uint8_t i = 0; i<num_of_mcmd; i++){
                feedback_msg.device = CAN_Device_to_DeviceInfo(&mcmd_handlers[i].device);
                feedback_msg.fb_type = MCMD_FB_TYPE_to_ActuatorMsg(mcmd_handlers[i].fb_type);
                feedback_msg.fb_data = Get_MCMD_Feedback(&mcmd_handlers[i].device).value;
                RCSOFTCHECK(rcl_publish(&publisher_mcmd, &feedback_msg, NULL));
            }
        }
    }
}

//void pub_timer_callback_c620(rcl_timer_t * timer, int64_t last_call_time){
//    RCLC_UNUSED(last_call_time);
//    actuator_msgs__msg__ActuatorMultipleFeedback feedback_msg_c620;
//    if (timer != NULL) {
//        if(num_of_c620 > 0){
//            for(uint8_t i=1; i<num_of_c620; i++){
//                if(c620_dev_info_global[i].ctrl_param.ctrl_type == C620_CTRL_POS) {
//                    feedback_msg_c620 = Get_C620_ActuatorMultiFB(&c620_dev_info_global[i],
//                                                                 actuator_msgs__msg__ActuatorFeedback__FB_POS);
//                }else if(c620_dev_info_global[i].ctrl_param.ctrl_type == C620_CTRL_VEL){
//                    feedback_msg_c620 = Get_C620_ActuatorMultiFB(&c620_dev_info_global[i],
//                                                                 actuator_msgs__msg__ActuatorFeedback__FB_VEL);
//                }
//                RCSOFTCHECK(rcl_publish(&publisher_c620, &feedback_msg_c620, NULL));
//            }
//        }
//    }
//}


//void pub_timer_callback_c620_vel(rcl_timer_t * timer, int64_t last_call_time){
//    RCLC_UNUSED(last_call_time);
//    if (timer != NULL) {
//        std_msgs__msg__Float32MultiArray vals;
//        std_msgs__msg__Float32MultiArray__init(&vals);
//        rosidl_runtime_c__float__Sequence__init(&(vals.data), 2);
//        static float data[2];
//        for(uint8_t i=0; i<(uint8_t)2; i++){
//            data[i] = Get_C620_FeedbackData(&c620_dev_info_global[i]).velocity;
//        }
//        vals.data.data = data;
//        RCSOFTCHECK(rcl_publish(&publisher_c620_vels, &vals, NULL));
//    }
//}


void pub_timer_callback_c620_theta(rcl_timer_t * timer, int64_t last_call_time){
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        actuator_msgs__msg__C620Feedback fb;
        fb.device = C620_Device_to_DeviceInfo(&c620_dev_info_global[0]);
        C620_FeedbackData fb_data = Get_C620_FeedbackData(&c620_dev_info_global[0]);
        fb.velocity = fb_data.velocity;
        fb.current = fb_data.current;
        fb.position = fb_data.position;
        fb.target_value = c620_dev_info_global[0].ctrl_param._target_value;
        RCSOFTCHECK(rcl_publish(&publisher_c620_theta, &fb, NULL));
    }
}

void pub_timer_callback_c620_r(rcl_timer_t * timer, int64_t last_call_time){
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        actuator_msgs__msg__C620Feedback fb;
        fb.device = C620_Device_to_DeviceInfo(&c620_dev_info_global[1]);
        C620_FeedbackData fb_data = Get_C620_FeedbackData(&c620_dev_info_global[1]);
        fb.velocity = fb_data.velocity;
        fb.current = fb_data.current;
        fb.position = fb_data.position;
        fb.target_value = c620_dev_info_global[1].ctrl_param._target_value;
        RCSOFTCHECK(rcl_publish(&publisher_c620_r, &fb, NULL));
    }
}

void subscription_callback(const void * msgin){
    const actuator_msgs__msg__ActuatorMsg * p_actuator_msg = (const actuator_msgs__msg__ActuatorMsg * )msgin;
    CAN_Device device_info = DeviceInfo_to_CAN_Device(&(p_actuator_msg->device));
    float _mros_target;

    if(device_info.node_type == NODE_MCMD4 || device_info.node_type == NODE_MCMD3){
        uint8_t device_size = (device_info.node_type == NODE_MCMD3) ? num_of_devices.mcmd3 : num_of_devices.mcmd4;
        MCMD_HandleTypedef* p_h_mcmd;
        for(uint8_t i=0; i < device_size; i++){
            if((mcmd_handlers[i].device.device_num == device_info.device_num) &&
               (mcmd_handlers[i].device.node_id == device_info.node_id)){
                p_h_mcmd = &(mcmd_handlers[i]);
                _mros_target = (float)(p_actuator_msg->target_value);
                MCMD_SetTarget(p_h_mcmd, _mros_target);
                break;
            }
        }
    }else if(p_actuator_msg->device.node_type.node_type == actuator_msgs__msg__NodeType__NODE_C620){
        for(uint8_t i=0; i<num_of_c620; i++){
            if(c620_dev_info_global[i].device_id == p_actuator_msg->device.device_num){
                _mros_target = (float)p_actuator_msg->target_value;
                C620_SetTarget(&c620_dev_info_global[i], _mros_target);
                break;
            }
        }
    }else if(p_actuator_msg->device.node_type.node_type == actuator_msgs__msg__NodeType__NODE_AIR){
        for(uint8_t i=0; i<NUM_OF_AIR; i++){
            if(air_devices[i].node_id == p_actuator_msg->device.node_id && air_devices[i].device_num == p_actuator_msg->device.device_num){
                if(p_actuator_msg->air_target){
                    AirCylinder_SendOutput(&(air_devices[i]), AIR_ON);
                }else{
                    AirCylinder_SendOutput(&(air_devices[i]), AIR_OFF);
                }
                break;
            }
        }
    }
}

void subscription_callback_r(const void * msgin) {
    const actuator_msgs__msg__ActuatorMsg *p_actuator_msg = (const actuator_msgs__msg__ActuatorMsg *) msgin;
    float _mros_target;

    if(p_actuator_msg->device.node_type.node_type == actuator_msgs__msg__NodeType__NODE_C620){
        if(p_actuator_msg->device.device_num != 2)return;
        _mros_target = (float)p_actuator_msg->target_value;
//        _mros_target = 0.0f;
        C620_SetTarget(&c620_dev_info_global[1], _mros_target);
    }
}

void subscription_callback_theta(const void * msgin) {
    const actuator_msgs__msg__ActuatorMsg *p_actuator_msg = (const actuator_msgs__msg__ActuatorMsg *) msgin;
    float _mros_target;

    if(p_actuator_msg->device.node_type.node_type == actuator_msgs__msg__NodeType__NODE_C620){
        if(p_actuator_msg->device.device_num != 1)return;
        _mros_target = (float)p_actuator_msg->target_value;
//        _mros_target = 0.0f;
        C620_SetTarget(&c620_dev_info_global[0], _mros_target);
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
    printf("start Micro-ROS Task\n");

    // micro-ROS app
    rclc_support_t support;
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rcl_node_t node;


    // node setting
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));  //create init_options
    RCCHECK(rclc_node_init_default(&node, "f7_mros_node", "", &support));  // create node

    // create executor
    rclc_executor_t executor;
    unsigned int num_handlers = 7; // TODO : 忘れずに変更
    RCCHECK(rclc_executor_init(&executor, &support.context, num_handlers, &allocator));


    // create subscriber for r
    rcl_subscription_t subscriber_r;
    const char* sub_name_r = "mros_input_r";
    actuator_msgs__msg__ActuatorMsg actuator_msg_r;
    RCCHECK(rclc_subscription_init_default(&subscriber_r, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(actuator_msgs, msg, ActuatorMsg), sub_name_r));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_r, &actuator_msg_r, &subscription_callback_r, ON_NEW_DATA));

    // create subscriber for theta
    rcl_subscription_t subscriber_theta;
    const char* sub_name_theta = "mros_input_theta";
    actuator_msgs__msg__ActuatorMsg actuator_msg_theta;
    RCCHECK(rclc_subscription_init_default(&subscriber_theta, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(actuator_msgs, msg, ActuatorMsg), sub_name_theta));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_theta, &actuator_msg_theta, &subscription_callback_theta, ON_NEW_DATA));

    // create subscriber for can modules
    rcl_subscription_t subscriber;
    const char* topic_name_sub = "mros_input";
    rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
    qos_profile.depth = 2;
    actuator_msgs__msg__ActuatorMsg actuator_msg;
    RCCHECK(rclc_subscription_init(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(actuator_msgs, msg, ActuatorMsg), topic_name_sub, &qos_profile));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &actuator_msg, &subscription_callback, ON_NEW_DATA));


    // publisher for mcmd
    const char* topic_name_pub_mcmd = "mros_output_mcmd";
    RCCHECK(rclc_publisher_init_default(&publisher_mcmd, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(actuator_msgs, msg, ActuatorFeedback), topic_name_pub_mcmd));
    rcl_timer_t timer_mcmd;
    RCCHECK(rclc_timer_init_default(&timer_mcmd, &support, RCL_MS_TO_NS(40), pub_timer_callback_mcmd));
    RCCHECK(rclc_executor_add_timer(&executor, &timer_mcmd));

    // publisher for c620
//    const char* topic_name_pub_c620 = "mros_output_c620";
//    RCCHECK(rclc_publisher_init_default(&publisher_c620, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(actuator_msgs, msg, ActuatorMultipleFeedback), topic_name_pub_c620));
//    rcl_timer_t timer_c620;
//    RCCHECK(rclc_timer_init_default(&timer_c620, &support, RCL_MS_TO_NS(40), pub_timer_callback_c620));
//    RCCHECK(rclc_executor_add_timer(&executor, &timer_c620));

    // publisher for c620 r
    RCCHECK(rclc_publisher_init_default(&publisher_c620_r, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(actuator_msgs, msg, C620Feedback), "c620_r"));
    rcl_timer_t timer_c620_r;
    RCCHECK(rclc_timer_init_default(&timer_c620_r, &support, RCL_MS_TO_NS(25), pub_timer_callback_c620_r));
    RCCHECK(rclc_executor_add_timer(&executor, &timer_c620_r));

    // publisher for c620 theta
    RCCHECK(rclc_publisher_init_default(&publisher_c620_theta, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(actuator_msgs, msg, C620Feedback), "c620_theta"));
    rcl_timer_t timer_c620_theta;
    RCCHECK(rclc_timer_init_default(&timer_c620_theta, &support, RCL_MS_TO_NS(25), pub_timer_callback_c620_theta));
    RCCHECK(rclc_executor_add_timer(&executor, &timer_c620_theta));


    rclc_executor_spin(&executor);


    // free resources
    RCCHECK(rclc_executor_fini(&executor));
    RCCHECK(rcl_subscription_fini(&subscriber, &node));
    RCCHECK(rcl_subscription_fini(&subscriber_r, &node));
    RCCHECK(rcl_subscription_fini(&subscriber_theta, &node));
//    RCCHECK(rcl_publisher_fini(&publisher_mcmd, &node))
    RCCHECK(rcl_publisher_fini(&publisher_c620_theta, &node))
    RCCHECK(rcl_publisher_fini(&publisher_c620_r, &node))
    RCCHECK(rcl_timer_fini(&timer_c620_r));
    RCCHECK(rcl_timer_fini(&timer_c620_theta));
    RCCHECK(rcl_timer_fini(&timer_mcmd));
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
        osDelay(100);
    }
  /* USER CODE END StartLEDTask */
}

/* C620TimerCallback function */
void C620TimerCallback(void *argument)
{
  /* USER CODE BEGIN C620TimerCallback */
    C620_SendRequest(c620_dev_info_global, 2, 1000.0f, &hcan1);
  /* USER CODE END C620TimerCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

