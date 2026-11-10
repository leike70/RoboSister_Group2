/**
  ******************************************************************************
  * @file           : DJI_motor_driver.cpp
  * @author         : Tessia
  * @brief          : None
  * @attention      : None
  * @date           : 2023/12/12
  ******************************************************************************
  */



#include "DJI_motor_driver.h"
#include "Protection/app_protection.h"



typedef struct dji_can_tx {
    CAN_HandleTypeDef *can_handle;
    uint16_t id;
    uint8_t enable;
    uint8_t buf[8];
}dji_motor_can_tx_t;

dji_motor_can_tx_t can_tx_list[] = {{&hcan1, 0x200, 0,},
                                    {&hcan1, 0x1ff, 0,},
                                    {&hcan1, 0x2ff, 0,},
                                    {&hcan2, 0x200, 0,},
                                    {&hcan2, 0x1ff, 0,},
                                    {&hcan2, 0x2ff, 0,}
};

void dji_motor_data_update_callback(can_device_receive *can_receive, uint8_t *data);

static DJI_motor *g_dji_motors[16];
static uint8_t g_dji_motor_count;

DJI_motor::DJI_motor(CAN_HandleTypeDef *hcan_, can_rx_callback *callback_,
                     uint32_t id_motor, DJI_MOTOR_TYPE type_, uint8_t is_reverse)
                     :can_rx_data{},motor_data{},
                      type(type_),current_to_send(0),bias_ordinal(0),is_use_external_speed(0),
                      cmd(DJI_MOTOR_RESET_OFFSET),init_offset_f(false),is_id_false(false),

                      can_rx(hcan_,(type_==DJI_M3508||type_==DJI_M2006)?0x200+id_motor:0x204+id_motor,callback_),
                      can_tx(hcan_,(type_==DJI_M3508||type_==DJI_M2006)?(id_motor<=4?0x200:0x1ff):(id_motor<=4?0x1ff:0x2ff)),
                      velPid(0.5f, 0.1f, 10, 0.002f, 0.1f),
                      posPid(0.2f,0.1f,0.3f,0.f,0.f) {

    reverse = is_reverse;
    id_motor>0&&id_motor<9 ? id = id_motor : is_id_false = 1;
    type==DJI_GM6020?(id_motor==8?is_id_false=1:id=id_motor):id=id_motor;

    tx_enable = nullptr;
    can_tx_data = nullptr;
    feedback_ready = false;
    output_enabled = false;
    lifecycle = DJI_MOTOR_WAIT_FEEDBACK;
    last_feedback_ms = 0u;
    init_start_ms = 0u;

    callback_ == nullptr ? can_rx.can_set_callback(dji_motor_data_update_callback) : can_rx.can_set_callback(callback_);

    /* Constructor never waits for feedback and never enables a bus output. */
    dji_motor_can_tx_t *tx = nullptr;
    if(hcan_==&hcan1){
        tx = (can_tx.member.id == 0x200) ? &can_tx_list[0]
             : (can_tx.member.id == 0x1ff) ? &can_tx_list[1] : &can_tx_list[2];
    } else if (hcan_ == &hcan2) {
        tx = (can_tx.member.id == 0x200) ? &can_tx_list[3]
             : (can_tx.member.id == 0x1ff) ? &can_tx_list[4] : &can_tx_list[5];
    }
    if (tx != nullptr) {
        can_tx_data = tx->buf;
        tx_enable = &tx->enable;
        tx->enable = 0u;
    }

    if (!is_id_false && g_dji_motor_count < 16u) {
        g_dji_motors[g_dji_motor_count++] = this;
    }
}

void DJI_motor::service(uint32_t now_ms) {
    if (lifecycle == DJI_MOTOR_WAIT_FEEDBACK && !feedback_ready) {
        if (now_ms - init_start_ms > 1000u) {
            lifecycle = DJI_MOTOR_FAULT;
            output_enabled = false;
            if (tx_enable != nullptr) { *tx_enable = 0u; }
        }
    }
}

/*
 * 获取解算电机数据
 */
void dji_motor_data_update_callback(can_device_receive *can_receive, uint8_t *data) {
    /* DJI feedback frames are 8-byte frames; short frames must not count as
     * feedback or advance the encoder/offset state machine. */
    if (can_receive == nullptr || data == nullptr || can_receive->rx_dlc != 8u) {
        return;
    }
    DJI_motor *dji_motor = container_of(can_receive, DJI_motor, can_rx);
    memcpy(dji_motor->can_rx_data,data,8);
    dji_motor->motor_data.msg_cnt++;
    dji_motor->last_feedback_ms = osKernelGetTickCount();
    if (dji_motor->motor_data.msg_cnt > 50)
    {
        dji_motor->init_offset_f = 1;
        dji_motor->feedback_ready = true;
        dji_motor->lifecycle = DJI_MOTOR_READY;
    }
    /* initial value */
    if (dji_motor->init_offset_f == 0)
    {
        dji_motor->motor_data.ecd = (uint16_t)(dji_motor->can_rx.rx_data[0] << 8 | dji_motor->can_rx.rx_data[1]);
        dji_motor->motor_data.offset_ecd = dji_motor->motor_data.ecd;
        return;
    }

    dji_motor->motor_data.last_ecd = dji_motor->motor_data.ecd;
    dji_motor->motor_data.ecd = (uint16_t)(dji_motor->can_rx.rx_data[0] << 8 | dji_motor->can_rx.rx_data[1]);

    if (dji_motor->motor_data.ecd - dji_motor->motor_data.last_ecd > 4096)
    {
        dji_motor->motor_data.round_cnt--;
        dji_motor->motor_data.ecd_raw_rate = dji_motor->motor_data.ecd - dji_motor->motor_data.last_ecd - 8192;
    }
    else if (dji_motor->motor_data.ecd - dji_motor->motor_data.last_ecd < -4096)
    {
        dji_motor->motor_data.round_cnt++;
        dji_motor->motor_data.ecd_raw_rate = dji_motor->motor_data.ecd - dji_motor->motor_data.last_ecd + 8192;
    }
    else
    {
        dji_motor->motor_data.ecd_raw_rate = dji_motor->motor_data.ecd - dji_motor->motor_data.last_ecd;
    }

    dji_motor->motor_data.total_ecd = dji_motor->motor_data.round_cnt * 8192 + dji_motor->motor_data.ecd - dji_motor->motor_data.offset_ecd;
    dji_motor->motor_data.total_round = (float)(dji_motor->motor_data.total_ecd) * ENCODER_TO_ROUND;
    dji_motor->motor_data.speed_rpm = (int16_t)(dji_motor->can_rx.rx_data[2] << 8 | dji_motor->can_rx.rx_data[3]);
    dji_motor->motor_data.given_current = (int16_t)(dji_motor->can_rx.rx_data[4] << 8 | dji_motor->can_rx.rx_data[5]);
}


void DJI_motor::dji_motor_can_tx_write() {
    if (!output_enabled || can_tx_data == nullptr) { return; }
    uint8_t index = (id < 5 ? id : id - 4) * 2 - 1;
    if (index > 7u) { return; }
    taskENTER_CRITICAL();
    can_tx_data[index] = (uint8_t)(current_to_send&0xFF);
    can_tx_data[index - 1] = (uint8_t)(current_to_send>>8);
    taskEXIT_CRITICAL();
}


/**
 * 获取电机速度
 * @param void
 * @return 电机实际速度 单位rpm
 */
float DJI_motor::motor_get_speed_forward() {
    if(is_use_external_speed == 1){
        return *motor_data.external_speed;
    } else{
        return (float)motor_data.speed_rpm;
    }
}

void DJI_motor::motor_change_speed_source(float *speed) {
    is_use_external_speed = 1;
    motor_data.external_speed = speed;
}


/**
 * 获取电机位子
 * @param motor
 * @return 电机计入过零点检测的圈数
 */
float DJI_motor::motor_get_rounds_forward() {
    return motor_data.total_round;
}


/**
 * 设置电机电流
 * @param motor
 * @param current_normal 范围 -1～1
 */
void DJI_motor::motor_set_current_forward(float current) {
    if (!output_enabled) {
        current_to_send = 0;
        return;
    }
    current = current > 1.0f ? 1.0f : (current < -1.0f ? -1.0f : current);
    switch(type){
        case DJI_M3508:
            current_to_send=(int16_t)(current * MOTOR_3508_MAX_CURRENT);
            break;
        case DJI_M2006:
            current_to_send=(int16_t)(current * MOTOR_2006_MAX_CURRENT);
            break;
        case DJI_GM6020:
            current_to_send=(int16_t)(current * MOTOR_6020_MAX_CURRENT);
            break;
    }
    dji_motor_can_tx_write();
}


//speed=0.05
void DJI_motor::motor_set_speed_forward(float speed) {
    switch(type){
        case DJI_M3508:
            velPid.pid_calculate(speed, (float)(motor_get_speed_forward()) / MOTOR_3508_MAX_SPEED);
            break;
        case DJI_M2006:
            velPid.pid_calculate(speed, (float)(motor_get_speed_forward()) / MOTOR_2006_MAX_SPEED);
            break;
        case DJI_GM6020:
            velPid.pid_calculate(speed, (float)(motor_get_speed_forward()) / MOTOR_6020_MAX_SPEED);
            break;
    }
    motor_set_current_forward(velPid.out);
}


void DJI_motor::motor_set_rounds_forward(float rounds) {
    posPid.pid_calculate(rounds, (float)(motor_get_rounds_forward()));
    motor_set_speed_forward(posPid.out);
}


bool DJI_motor::motor_reset() {
    motor_data.offset_ecd = motor_data.ecd;
    motor_data.total_ecd = 0;
    motor_data.total_round = 0;
    motor_data.round_cnt = 0;
    return(true);
}


void DJI_motor::motor_control(uint32_t cmd_) {
    cmd = (DJI_CMD)cmd_;
    switch (cmd) {
        case DJI_MOTOR_CLEAR_PID:
            velPid.pid_clear();
            posPid.pid_clear();
            break;
        case DJI_MOTOR_RESET_OFFSET:
            motor_reset();
            break;
        case DJI_MOTOR_DISABLE_OFFSET:
            motor_data.offset_ecd = 0;
            break;
        case DJI_MOTOR_ADD_ONE_LAP_OFFSET:
            motor_data.offset_ecd = motor_data.offset_ecd + ENCODER_MAX;
            break;
        case DJI_MOTOR_SUB_ONE_LAP_OFFSET:
            motor_data.offset_ecd = motor_data.offset_ecd - ENCODER_MAX;
            break;
        case DJI_MOTOR_ENABLE_OUTPUT:
            if (feedback_ready && !is_id_false) {
                output_enabled = true;
                lifecycle = DJI_MOTOR_READY;
                if (tx_enable != nullptr) { *tx_enable = 1u; }
            }
            break;
        case DJI_MOTOR_DISABLE_OUTPUT:
            output_enabled = false;
            if (tx_enable != nullptr) { *tx_enable = 0u; }
            if (can_tx_data != nullptr) { memset(can_tx_data, 0, 8); }
            break;
    }
}


void DJI_motor_service(void *argument) {
    (void)argument;
    uint32_t tick = osKernelGetTickCount();
    for(;;){
        const uint32_t now_ms = osKernelGetTickCount();
        for (uint8_t m = 0u; m < g_dji_motor_count; ++m) {
            if (g_dji_motors[m] != nullptr) { g_dji_motors[m]->service(now_ms); }
        }
        const bool output_allowed = app_protection_output_allowed();
        for(int i = 0;i<6;i++){
            if(!can_tx_list[i].enable){ continue; }
            if(!output_allowed){
                memset(can_tx_list[i].buf, 0, 8);
                can_tx_list[i].enable = 0u;
                continue;
            }
            can_device_transmit msg;
            msg.member.hcan = can_tx_list[i].can_handle;
            msg.member.id = can_tx_list[i].id;
            msg.member.len = 8;
            memcpy(msg.member.buf_data, can_tx_list[i].buf, 8);
            (void)msg.can_send_msg();
        }
        tick+=1;
        osDelayUntil(tick);
    }
}
