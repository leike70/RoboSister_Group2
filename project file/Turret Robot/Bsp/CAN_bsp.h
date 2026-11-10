//
// Created by Tessia on 2023/12/10.
//

#ifndef PEPPER_PICKER_GIMBAL_CAN_BSP_H
#define PEPPER_PICKER_GIMBAL_CAN_BSP_H

#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "can.h"
#include "can_bsp_api.h"
#include "string.h"

class can_device_receive;

typedef void (can_rx_callback)(can_device_receive *can_receive, uint8_t *data);

/** Queue element: payload is copied into the element, never borrowed from caller. */
struct can_device_transmit_member {
    CAN_HandleTypeDef *hcan;
    uint32_t id;
    uint8_t len;
    uint8_t buf_data[8];
    uint32_t generation; /**< flush generation; stale dequeued commands are dropped */
};

class can_device_receive {
public:
    can_device_receive(CAN_HandleTypeDef *hcan_ = &hcan1, uint32_t id_ = 0,
                       can_rx_callback *callback_ = nullptr);
    bool can_set_callback(can_rx_callback *callback_);
    void can_modify_id(uint32_t id_);

    /** True when this registration belongs to hcan/id and is valid. */
    bool matches(CAN_HandleTypeDef *hcan_, uint32_t id_) const;
    bool is_registered(void) const { return registered; }

public:
    uint8_t rx_data[8];
    uint8_t rx_dlc;
    can_rx_callback *callback;
    uint8_t is_callback_none;
    bool registered;

private:
    CAN_HandleTypeDef *hcan;
    uint32_t id;
    uint32_t index;
};

class can_device_transmit {
public:
    can_device_transmit(CAN_HandleTypeDef *hcan_, uint32_t id_ = 0, uint8_t len_ = 8,
                        uint8_t *buf_data_ = nullptr);
    can_device_transmit();
    osStatus_t can_send_msg(void);

    /** Snapshot caller data. The old pointer is not retained. */
    bool set_buf_address(uint8_t *buf_address);
    bool set_data(const uint8_t *data, uint8_t len);
    bool set_id(uint32_t id);

public:
    can_device_transmit_member member;
};

#ifdef __cplusplus
extern "C" {
#endif

void can1_sending_service(void *argument);
void can2_sending_service(void *argument);
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan);

#ifdef __cplusplus
}
#endif

#endif // PEPPER_PICKER_GIMBAL_CAN_BSP_H
