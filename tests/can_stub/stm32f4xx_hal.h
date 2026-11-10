#ifndef TEST_STM32F4XX_HAL_H
#define TEST_STM32F4XX_HAL_H

#include <stdint.h>

typedef enum { HAL_OK = 0, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;

typedef struct {
    uint32_t ErrorCode;
} CAN_HandleTypeDef;

typedef struct {
    uint32_t StdId;
    uint32_t ExtId;
    uint32_t IDE;
    uint32_t RTR;
    uint32_t DLC;
    uint32_t FilterMatchIndex;
} CAN_RxHeaderTypeDef;

typedef struct {
    uint32_t StdId;
    uint32_t ExtId;
    uint32_t IDE;
    uint32_t RTR;
    uint32_t DLC;
    uint32_t TransmitGlobalTime;
} CAN_TxHeaderTypeDef;

typedef struct {
    uint32_t FilterIdHigh;
    uint32_t FilterIdLow;
    uint32_t FilterMaskIdHigh;
    uint32_t FilterMaskIdLow;
    uint32_t FilterFIFOAssignment;
    uint32_t FilterBank;
    uint32_t FilterMode;
    uint32_t FilterScale;
    uint32_t FilterActivation;
    uint32_t SlaveStartFilterBank;
} CAN_FilterTypeDef;

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

extern CAN_RxHeaderTypeDef test_rx_header;
extern uint8_t test_rx_data[8];
extern HAL_StatusTypeDef test_rx_status;
extern HAL_StatusTypeDef test_start_status1;
extern HAL_StatusTypeDef test_start_status2;
extern HAL_StatusTypeDef test_notify_status;
extern HAL_StatusTypeDef test_tx_status;
extern uint32_t test_tx_count;

#ifdef __cplusplus
extern "C" {
#endif
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef HAL_CAN_Stop(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *hcan,
                                               uint32_t active_its);
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *hcan,
                                       const CAN_FilterTypeDef *filter);
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *hcan, uint32_t fifo,
                                       CAN_RxHeaderTypeDef *header,
                                       uint8_t data[8]);
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *hcan,
                                       const CAN_TxHeaderTypeDef *header,
                                       const uint8_t data[8],
                                       uint32_t *mailbox);
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *hcan,
                                         uint32_t mailboxes);
#ifdef __cplusplus
}
#endif

#define CAN_ID_STD 0u
#define CAN_RTR_DATA 0u
#define CAN_RTR_REMOTE 1u
#define CAN_RX_FIFO0 0u
#define CAN_RX_FIFO1 1u
#define CAN_FILTERMODE_IDLIST 0u
#define CAN_FILTERSCALE_16BIT 0u
#define ENABLE 1u
#define CAN_TX_MAILBOX0 1u
#define CAN_TX_MAILBOX1 2u
#define CAN_TX_MAILBOX2 4u
#define CAN_IT_RX_FIFO0_MSG_PENDING 1u
#define CAN_IT_RX_FIFO1_MSG_PENDING 2u
#define CAN_IT_TX_MAILBOX_EMPTY 4u
#define CAN_IT_BUSOFF 8u
#define CAN_IT_ERROR_PASSIVE 16u
#define CAN_IT_ERROR_WARNING 32u
#define CAN_IT_LAST_ERROR_CODE 64u
#define CAN_IT_ERROR 128u
#define HAL_CAN_ERROR_BOF (1u << 0)

#endif
