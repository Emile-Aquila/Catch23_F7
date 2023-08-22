//
// Created by emile on 23/08/18.
//

#ifndef _CATCH23_F7_CAN_UTILS_H
#define _CATCH23_F7_CAN_UTILS_H

#include "main.h"

#include "CAN_C620_System.h"
#include "CAN_C620.h"
#include "CAN_Main.h"
#include <stdio.h>

#include <actuator_msgs/msg/actuator_msg.h>
#include <actuator_msgs/msg/actuator_feedback.h>
#include <actuator_msgs/msg/node_type.h>

#define NUM_OF_MCMD3 0
#define NUM_OF_MCMD4 0

extern C620_DeviceInfo c620_dev_info_global[8];
extern MCMD_HandleTypedef mcmd_handlers[NUM_OF_MCMD3+NUM_OF_MCMD4];

extern NUM_OF_DEVICES num_of_devices;
extern const uint8_t num_of_c620;

actuator_msgs__msg__DeviceInfo CAN_Device_to_DeviceInfo(CAN_Device* can_device);
CAN_Device DeviceInfo_to_CAN_Device(actuator_msgs__msg__DeviceInfo* device_info);
uint8_t MCMD_FB_TYPE_to_ActuatorMsg(MCMD_FB_TYPE fb_type);

actuator_msgs__msg__DeviceInfo C620_Device_to_DeviceInfo(C620_DeviceInfo* c620_device_info);
actuator_msgs__msg__ActuatorFeedback Get_C620_ActuatorFB(C620_DeviceInfo* c620_device_info, uint8_t act_fb_type);


#endif //_CATCH23_F7_CAN_UTILS_H
