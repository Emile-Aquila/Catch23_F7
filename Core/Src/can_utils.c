//
// Created by emile on 23/08/18.
//

#include <can_utils.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>


MCMD_HandleTypedef mcmd_handlers[1];
MCMD_Feedback_Typedef mcmd_fb[1];


C620_DeviceInfo c620_dev_info_global[8];
NUM_OF_DEVICES num_of_devices;

actuator_msgs__msg__DeviceInfo CAN_Device_to_DeviceInfo(CAN_Device* can_device){
    actuator_msgs__msg__DeviceInfo device;
    device.node_id = can_device->node_id;
    device.device_num = can_device->device_num;
    switch(can_device->node_type){
        case NODE_MCMD1:
            device.node_type.node_type = actuator_msgs__msg__NodeType__NODE_MCMD1;
            break;
        case NODE_MCMD2:
            device.node_type.node_type = actuator_msgs__msg__NodeType__NODE_MCMD2;
            break;
        case NODE_MCMD3:
            device.node_type.node_type = actuator_msgs__msg__NodeType__NODE_MCMD3;
            break;
        case NODE_MCMD4:
            device.node_type.node_type = actuator_msgs__msg__NodeType__NODE_MCMD4;
            break;
        case NODE_AIR:
            device.node_type.node_type = actuator_msgs__msg__NodeType__NODE_AIR;
            break;
        case NODE_SERVO:
            device.node_type.node_type = actuator_msgs__msg__NodeType__NODE_SERVO;
            break;
        default:
            break;
    }
    return device;
}

CAN_Device DeviceInfo_to_CAN_Device(actuator_msgs__msg__DeviceInfo* device_info){
    CAN_Device can_device;
    can_device.node_id = device_info->node_id;
    can_device.device_num = device_info->device_num;
    switch(device_info->node_type.node_type){
        case actuator_msgs__msg__NodeType__NODE_MCMD1:
            can_device.node_type = NODE_MCMD1;
            break;
        case actuator_msgs__msg__NodeType__NODE_MCMD2:
            can_device.node_type = NODE_MCMD2;
            break;
        case actuator_msgs__msg__NodeType__NODE_MCMD3:
            can_device.node_type = NODE_MCMD3;
            break;
        case actuator_msgs__msg__NodeType__NODE_MCMD4:
            can_device.node_type = NODE_MCMD4;
            break;
        case actuator_msgs__msg__NodeType__NODE_AIR:
            can_device.node_type = NODE_AIR;
            break;
        case actuator_msgs__msg__NodeType__NODE_SERVO:
            can_device.node_type = NODE_SERVO;
            break;
        default:
            break;
    }
    return can_device;
}


uint8_t MCMD_FB_TYPE_to_ActuatorMsg(MCMD_FB_TYPE fb_type){
    uint8_t ans = 0;
    switch(fb_type){
        case MCMD_FB_DUTY:
            ans = actuator_msgs__msg__ActuatorFeedback__FB_DUTY;
            break;
        case MCMD_FB_POS:
            ans = actuator_msgs__msg__ActuatorFeedback__FB_POS;
            break;
        case MCMD_FB_VEL:
            ans = actuator_msgs__msg__ActuatorFeedback__FB_VEL;
            break;
        default:
            break;
    }
    return ans;
}