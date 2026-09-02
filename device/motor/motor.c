/**
 * @file motor.c
 * @brief GM6020, M3508, DM and MG4005E motor-device implementations.
 */

#include "motor.h"
#include "motor_instance.h"
#include "motor_internal.h"

#include <stdarg.h>
#include <string.h>

#include "../../bsp/can/can.h"
#include "../../bsp/LED/LED.h"
#include "../../dsp/pid/pid.h"
#include "../../dsp/math.h"

volatile uint32_t g_can1_irq_count = 0U;
volatile uint32_t g_can1_frame_count = 0U;
volatile uint32_t g_can1_last_id = 0U;
volatile uint32_t g_can1_last_dlc = 0U;
volatile uint32_t g_can1_tx_ok_count = 0U;
volatile uint32_t g_can1_tx_fail_count = 0U;

/* ------------------------------ 工具化函数 ---------------------------------- */

// 电机数据组帧下发函数，传入参数为下发can句柄，canid，下发8字节数据
static void motor_send_standard(FDCAN_HandleTypeDef *handle, uint32_t identifier,
                                const uint8_t data[8])
{
    FDCAN_TxHeaderTypeDef header = {0};
    HAL_StatusTypeDef status;

    if (handle == NULL || data == NULL) {
        return;
    }

    header.Identifier = identifier & 0x7FFU;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = FDCAN_DLC_BYTES_8;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    header.MessageMarker = 0U;
    status = HAL_FDCAN_AddMessageToTxFifoQ(handle, &header, (uint8_t *)data);
    if (handle == &hfdcan1) {
        if (status == HAL_OK) {
            ++g_can1_tx_ok_count;
        } else {
            ++g_can1_tx_fail_count;
        }
    }
}

// 限幅函数
static float clamp_float(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}
static float abs_float(float value)
{
    return (value >= 0.0f) ? value : -value;
}
static float motor_sqrt_float(float value)
{
    float estimate;
    uint32_t index;

    if (value <= 0.0f) {
        return 0.0f;
    }

    estimate = (value >= 1.0f) ? value : 1.0f;
    for (index = 0U; index < 8U; ++index) {
        estimate = 0.5f * (estimate + value / estimate);
    }
    return estimate;
}
static float wrap_to_pi(float angle_rad)
{
    while (angle_rad > PI) { angle_rad -= TWO_PI; }
    while (angle_rad < -PI) { angle_rad += TWO_PI; }
    return angle_rad;
}

// DM协议要求浮点数转化为uint16
static uint16_t float_to_uint(float value, float minimum, float maximum, uint16_t bits)
{
    float scaled;
    uint32_t maximum_integer = (1UL << bits) - 1UL;

    value = clamp_float(value, minimum, maximum);
    scaled = (value - minimum) * (float)maximum_integer / (maximum - minimum);
    return (uint16_t)scaled;
}
static float uint_to_float(uint32_t value, float minimum, float maximum, uint16_t bits)
{
    uint32_t maximum_integer = (1UL << bits) - 1UL;
    return (float)value * (maximum - minimum) / (float)maximum_integer + minimum;
}

static void gm6020_send_group(FDCAN_HandleTypeDef *can_handle);
static void m3508_send_group(FDCAN_HandleTypeDef *can_handle);

/* ------------------------------ GM6020 ---------------------------------- */
// 初始化6020对象 传入要实例化电机对象，反馈id，can句柄
// 作用为清空6020初始状态，绑定can和id，确定组帧槽位，并加载pid参数
void gm6020_init(struct motor_device *motor, uint32_t motor_id,
                        FDCAN_HandleTypeDef *can_handle, int para_num, ...)
{
    gm6020_data_t *data;
    const gm6020_pid_config_t *pid_config;
    float rotational_inertia_kg_m2;
    float K_t_Nm_A;
    float friction_torque;

    if (motor == NULL || motor->motor_data == NULL || can_handle == NULL) {
        return;
    }
    data = motor->motor_data;
    pid_config = data->pid_config;
    rotational_inertia_kg_m2 = data->rotational_inertia_kg_m2;
    friction_torque = data->friction_torque;
    K_t_Nm_A = data->K_t_Nm_A;
    if (pid_config == NULL) {
        return;
    }
    memset(data, 0, sizeof(*data));
    data->pid_config = pid_config;
    data->rotational_inertia_kg_m2 = rotational_inertia_kg_m2;
    data->friction_torque = friction_torque;
    data->K_t_Nm_A = K_t_Nm_A;
    motor->motor_id = motor_id;
    motor->motor_can_handle = can_handle;
    /* Feedback IDs are 0x205...0x208 for physical motors 1...4. */
    if (motor_id < 0x205U || motor_id > 0x208U) {
        return;
    }
    data->control_slot = (uint8_t)(motor_id - 0x204U);

    /* Parameters belong to this motor instance, not to the GM6020 type. */
    data->position_pid = pid_config->position_pid;
    data->velocity_pid = pid_config->velocity_pid;
    pid_reset(&data->position_pid);
    pid_reset(&data->velocity_pid);
    (void)para_num;
}

// 解析6020反馈帧数据，传入实例化电机与反馈帧
// 数据存储在该实例对应的私有数据中
void gm6020_feedback_calculate(const struct motor_device *motor,
                                      const uint8_t frame[8])
{
    gm6020_data_t *data;
    uint16_t encoder;
    int32_t delta_encoder;

    if (motor == NULL || motor->motor_data == NULL || frame == NULL) {
        return;
    }

    data = motor->motor_data;

    encoder = (uint16_t)(((uint16_t)frame[0] << 8) | frame[1]);

    if (data->encoder_initialized == 0U) {
        data->last_encoder = encoder;
        data->position_continuous_rad =
            (float)encoder * TWO_PI / GM6020_ENCODER;
        data->encoder_initialized = 1U;
    } else {
        delta_encoder = (int32_t)encoder - (int32_t)data->last_encoder;

        if (delta_encoder > GM6020_ENCODER / 2) {
            delta_encoder -= GM6020_ENCODER;
        } else if (delta_encoder < - GM6020_ENCODER / 2) {
            delta_encoder += GM6020_ENCODER;
        }

        data->position_continuous_rad +=
            (float)delta_encoder * TWO_PI / GM6020_ENCODER;

        data->last_encoder = encoder;
    }

    data->encoder = encoder;
    data->speed_rpm =
        (int16_t)(((uint16_t)frame[2] << 8) | frame[3]);
    data->current =
        (int16_t)(((uint16_t)frame[4] << 8) | frame[5]);
    data->temperature = frame[6];

    data->position_rad = data->position_continuous_rad;
    data->velocity_rpm = (float)data->speed_rpm;
}

// 打包6020电机数据组帧发送
void gm6020_send_ctrl_cmd(struct motor_device *motor)
{
    if (motor == NULL || motor->motor_data == NULL) {
        return;
    }
    /* A GM6020 control command is always a complete shared 0x1FF group frame. */
    gm6020_send_group(motor->motor_can_handle);
}

// 使能6020
void gm6020_enable(struct motor_device *motor)
{
    gm6020_data_t *data;

    if (motor == NULL || motor->motor_data == NULL) {
        return;
    }
    if (!motor_is_online(motor)) {
        return;
    }

    data = motor->motor_data;
    data->target_position_rad = data->position_rad;
    data->target_velocity_rpm = 0.0f;
    data->target_acceleration_rad_s2 = 0.0f;
    data->trace.active = 0U;

    pid_reset(&data->position_pid);
    pid_reset(&data->velocity_pid);

    data->last_update_tick = 0U;
    data->enabled = 1U;
}

// 失能6020
void gm6020_disable(struct motor_device *motor)
{
    gm6020_data_t *data;

    if (motor == NULL || motor->motor_data == NULL) {
        return;
    }

    data = motor->motor_data;
    data->enabled = 0U;
    data->output_current_code = 0.0f;
    data->target_position_rad = data->position_rad;
    data->target_velocity_rpm = 0.0f;
    data->target_acceleration_rad_s2 = 0.0f;
    data->trace.active = 0U;

    pid_reset(&data->position_pid);
    pid_reset(&data->velocity_pid);

    data->last_update_tick = 0U;

    gm6020_send_ctrl_cmd(motor);
}

// 不断更新gm6020应该下发的值
void gm6020_update(struct motor_device *motor)
{
    gm6020_data_t *data;
    float position_error;
    float position_pid_out_rpm;
    float velocity_pid_out_rpm;
    float output_current_code;
    float all_torque;
    float acc_torque;
    float grativity_torque;
    float friction_torque;
    float ff_current;
    float dt;
    uint32_t now;

    if (motor == NULL || motor->motor_data == NULL) {
        return;
    }

    data = motor->motor_data;

    if (data->enabled == 0U || motor->last_rx_tick == 0U || (HAL_GetTick() - motor->last_rx_tick) > MOTOR_OFFLINE_MS) {
        data->output_current_code = 0.0f;
        data->target_position_rad = data->position_rad;
        data->target_velocity_rpm = 0.0f;
        data->target_acceleration_rad_s2 = 0.0f;
        data->trace.active = 0U;

        pid_reset(&data->position_pid);
        pid_reset(&data->velocity_pid);

        data->last_update_tick = 0U;
        return;
    }

    now = HAL_GetTick();

    if (data->last_update_tick == 0U) {
        dt = MOTOR_CONTROL_S;
    } else {
        dt = (float)(now - data->last_update_tick) * 0.001f;
        dt = clamp_float(dt, 0.0001f, 0.020f);
    }

    data->last_update_tick = now;

    position_error = wrap_to_pi(data->target_position_rad - data->position_rad);

    position_pid_out_rpm = pid_update(&data->position_pid, position_error, 0.0f, dt);

    velocity_pid_out_rpm = data->target_velocity_rpm + position_pid_out_rpm;

    velocity_pid_out_rpm = clamp_float(velocity_pid_out_rpm, -GM6020_SPEED_LIMIT_RPM, GM6020_SPEED_LIMIT_RPM);

    output_current_code = pid_update(&data->velocity_pid, velocity_pid_out_rpm, data->velocity_rpm, dt);

    acc_torque =  data->target_acceleration_rad_s2 * data->rotational_inertia_kg_m2;

    grativity_torque = 0;

    friction_torque = data->friction_torque;

    all_torque = acc_torque + grativity_torque + friction_torque;

    ff_current = all_torque / data->K_t_Nm_A;

    output_current_code = output_current_code + ff_current * 16384 / 3;;

    data->output_current_code = output_current_code;

    if (data->output_current_code > 16384)
    {data->output_current_code = 16384;}
    else if (data->output_current_code < -16384)
    {data->output_current_code = -16384;}
}

// 设定轨迹
static void gm6020_trace_set_target(const struct motor_device *motor,
                                    float target_position_rad,
                                    float target_velocity_rpm,
                                    float target_acceleration_rad_s2)
{
    gm6020_data_t *data;

    if (motor == NULL || motor->motor_data == NULL) {
        return;
    }
    data = motor->motor_data;
    data->target_position_rad = target_position_rad;
    data->target_velocity_rpm = target_velocity_rpm;
    data->target_acceleration_rad_s2 = target_acceleration_rad_s2;
}

// 得到轨迹需要的时间
static float gm6020_trace_resolve_duration(float delta_position_rad, float requested_duration_s)
{
    float distance_rad;
    float maximum_velocity_rad_s;
    float minimum_velocity_duration_s;
    float minimum_acceleration_duration_s;
    float actual_duration_s;

    distance_rad = abs_float(delta_position_rad);
    maximum_velocity_rad_s = GM6020_SPEED_LIMIT_RPM * RPM_TO_RAD_S;
    minimum_velocity_duration_s = MOTOR_QUINTIC_PEAK_VELOCITY_FACTOR * distance_rad / maximum_velocity_rad_s;
    minimum_acceleration_duration_s = motor_sqrt_float(MOTOR_QUINTIC_PEAK_ACCELERATION_FACTOR * distance_rad / GM6020_TRACE_MAX_ACCELERATION_RAD_S2);

    actual_duration_s = requested_duration_s;
    if (actual_duration_s < minimum_velocity_duration_s) {
        actual_duration_s = minimum_velocity_duration_s;
    }
    if (actual_duration_s < minimum_acceleration_duration_s) {
        actual_duration_s = minimum_acceleration_duration_s;
    }
    if (actual_duration_s < MOTOR_CONTROL_S) {
        actual_duration_s = MOTOR_CONTROL_S;
    }

    return actual_duration_s;
}

// 初始化生成轨迹参数
void gm6020_set_trace(const struct motor_device *motor,
                             float target_position_rad,
                             float duration_s)
{
    gm6020_data_t *data;
    float delta_position_rad;

    if (motor == NULL || motor->motor_data == NULL) {
        return;
    }

    data = motor->motor_data;
    if (!motor_is_online(motor) || data->enabled == 0U || duration_s <= 0.0f) {
        return;
    }

    data->trace.start_position_rad = data->position_rad;
    delta_position_rad = wrap_to_pi(target_position_rad - data->trace.start_position_rad);
    data->trace.delta_position_rad = delta_position_rad;
    data->trace.end_position_rad = data->trace.start_position_rad + delta_position_rad;
    data->trace.duration_s = gm6020_trace_resolve_duration(delta_position_rad, duration_s);
    data->trace.position_ref_rad = data->trace.start_position_rad;
    data->trace.velocity_ref_rad_s = 0.0f;
    data->trace.acceleration_ref_rad_s2 = 0.0f;
    data->trace.start_tick = HAL_GetTick();

    if (delta_position_rad > - TWO_PI / GM6020_ENCODER && delta_position_rad < TWO_PI / GM6020_ENCODER) {
        data->trace.active = 0U;
        gm6020_trace_set_target(motor, data->trace.end_position_rad, 0.0f,0.0f);
        return;
    }

    data->trace.active = 1U;
    gm6020_trace_set_target(motor, data->trace.start_position_rad, 0.0f,0.0f);
}

// 依据strace进度更新strace
void gm6020_trace_update(struct motor_device *motor)
{
    gm6020_data_t *data;
    motor_trace_t *trace;
    float elapsed_s;
    float normalized_time;
    float normalized_time_2;
    float normalized_time_3;
    float normalized_time_4;
    float normalized_time_5;
    float position_scale;
    float velocity_scale;
    float acceleration_scale;

    if (motor == NULL || motor->motor_data == NULL) {
        return;
    }

    data = motor->motor_data;
    trace = &data->trace;
    if (trace->active == 0U) {
        return;
    }

    if (data->enabled == 0U || !motor_is_online(motor)) {
        trace->active = 0U;
        data->target_position_rad = data->position_rad;
        data->target_velocity_rpm = 0.0f;
        data->target_acceleration_rad_s2 = 0.0f;
        return;
    }

    elapsed_s = (float)(HAL_GetTick() - trace->start_tick) * 0.001f;
    if (elapsed_s >= trace->duration_s) {
        trace->position_ref_rad = trace->end_position_rad;
        trace->velocity_ref_rad_s = 0.0f;
        trace->acceleration_ref_rad_s2 = 0.0f;
        trace->active = 0U;
        gm6020_trace_set_target(motor, trace->end_position_rad, 0.0f,0.0f);
        return;
    }

    normalized_time = elapsed_s / trace->duration_s;
    normalized_time = clamp_float(normalized_time, 0.0f, 1.0f);
    normalized_time_2 = normalized_time * normalized_time;
    normalized_time_3 = normalized_time_2 * normalized_time;
    normalized_time_4 = normalized_time_3 * normalized_time;
    normalized_time_5 = normalized_time_4 * normalized_time;

    position_scale = 10.0f * normalized_time_3 - 15.0f * normalized_time_4 + 6.0f * normalized_time_5;
    velocity_scale = 30.0f * normalized_time_2 - 60.0f * normalized_time_3 + 30.0f * normalized_time_4;
    acceleration_scale = 60.0f * normalized_time - 180.0f * normalized_time_2 + 120.0f * normalized_time_3;

    trace->position_ref_rad = trace->start_position_rad + trace->delta_position_rad * position_scale;
    trace->velocity_ref_rad_s = trace->delta_position_rad / trace->duration_s * velocity_scale;
    trace->acceleration_ref_rad_s2 = trace->delta_position_rad / (trace->duration_s * trace->duration_s) * acceleration_scale;

    gm6020_trace_set_target(motor, trace->position_ref_rad, trace->velocity_ref_rad_s * RAD_S_TO_RPM,trace->acceleration_ref_rad_s2);
}

// 设置目标位置，参数顺序为目标位置，速度前馈
void gm6020_set_target(const struct motor_device *motor, int para_num, ...)
{
    gm6020_data_t *data;
    float target_position_rad;
    float target_velocity_rpm;
    float target_acceleration_rad_s2;
    va_list arguments;

    if (motor == NULL || motor->motor_data == NULL) {
        return;
    }
    data = motor->motor_data;
    target_position_rad = data->target_position_rad;
    target_velocity_rpm = data->target_velocity_rpm;
    target_acceleration_rad_s2 = data->target_acceleration_rad_s2;
    va_start(arguments, para_num);
    if (para_num >= 1)
    {
        target_position_rad = (float)va_arg(arguments, double);
        target_velocity_rpm = 0.0f;
        target_acceleration_rad_s2 = 0.0f;
    }
    if (para_num >= 2)
    {
        target_velocity_rpm = (float)va_arg(arguments, double);
        target_acceleration_rad_s2 = 0.0f;
    }

    if (para_num >= 3) { target_acceleration_rad_s2 = (float)va_arg(arguments, double); }
    va_end(arguments);
    data->trace.active = 0U;
    gm6020_trace_set_target(motor, target_position_rad, target_velocity_rpm,target_acceleration_rad_s2);
}

//获取电机的某个状态值，存入value中
void gm6020_get_status(const struct motor_device *motor, const char *which,
                              void *value)
{
    gm6020_data_t *data;
    if (motor == NULL || motor->motor_data == NULL || which == NULL || value == NULL) { return; }
    data = motor->motor_data;
    if (strcmp(which, "POS") == 0) { *(float *)value = data->position_rad; }
    else if (strcmp(which, "VEL") == 0) { *(float *)value = data->velocity_rpm; }
    else if (strcmp(which, "TEMP") == 0) { *(uint8_t *)value = data->temperature; }
    else if (strcmp(which, "TARGET_POS") == 0) {*(float *)value = data->target_position_rad;}
    else if (strcmp(which, "TARGET_VEL") == 0) {*(float *)value = data->target_velocity_rpm;}
    else if (strcmp(which, "TARGET_ACC") == 0) {*(float *)value = data->target_acceleration_rad_s2;}
    else if (strcmp(which, "ENC") == 0) { *(uint16_t *)value = data->encoder; }
}


// 运行时把对应的pid参数修改为value
void gm6020_set_para(const struct motor_device *motor, const char *which,
                            const void *value)
{
    gm6020_data_t *data;
    if (motor == NULL || motor->motor_data == NULL || which == NULL || value == NULL) { return; }
    data = motor->motor_data;
    if (strcmp(which, "POS_KP") == 0) { data->position_pid.kp = *(const float *)value; }
    else if (strcmp(which, "POS_KI") == 0) { data->position_pid.ki = *(const float *)value; }
    else if (strcmp(which, "POS_KD") == 0) { data->position_pid.kd = *(const float *)value; }
    else if (strcmp(which, "VEL_KP") == 0) { data->velocity_pid.kp = *(const float *)value; }
    else if (strcmp(which, "VEL_KI") == 0) { data->velocity_pid.ki = *(const float *)value; }
    else if (strcmp(which, "VEL_KD") == 0) { data->velocity_pid.kd = *(const float *)value; }
}

/* ------------------------------ M3508 ---------------------------------- */
// 初始化3508对象 传入要实例化电机对象，反馈id，can句柄
// 作用为清空3508初始状态，绑定can和id，确定组帧槽位，并加载pid参数
void m3508_init(struct motor_device *motor, uint32_t motor_id,
                       FDCAN_HandleTypeDef *can_handle, int para_num, ...)
{
    m3508_data_t *data;
    const m3508_pid_config_t *pid_config;
    float rotational_inertia_kg_m2;
    float K_t_Nm_A;
    float friction_torque;
    if (motor == NULL || motor->motor_data == NULL || can_handle == NULL) { return; }
    data = motor->motor_data;
    pid_config = data->pid_config;
    rotational_inertia_kg_m2 = data->rotational_inertia_kg_m2;
    friction_torque = data->friction_torque;
    K_t_Nm_A = data->K_t_Nm_A;
    if (pid_config == NULL) { return; }
    memset(data, 0, sizeof(*data));
    data->pid_config = pid_config;
    data->rotational_inertia_kg_m2 = rotational_inertia_kg_m2;
    data->friction_torque = friction_torque;
    data->K_t_Nm_A = K_t_Nm_A;
    motor->motor_id = motor_id;
    motor->motor_can_handle = can_handle;
    /* C620 IDs 1...4 use feedback IDs 0x201...0x204 and control group 0x200. */
    if (motor_id < 0x201U || motor_id > 0x204U) { return; }
    data->control_slot = (uint8_t)(motor_id - 0x200U);
    data->position_pid = pid_config->position_pid;
    data->velocity_pid = pid_config->velocity_pid;
    pid_reset(&data->position_pid);
    pid_reset(&data->velocity_pid);
    (void)para_num;
}

// 解析3508反馈帧数据，传入实例化电机与反馈帧
// 数据存储在该实例对应的私有数据中
void m3508_feedback_calculate(const struct motor_device *motor,
                                     const uint8_t frame[8])
{
    m3508_data_t *data;
    uint16_t encoder;
    int32_t delta_encoder;
    if (motor == NULL || motor->motor_data == NULL || frame == NULL) { return; }
    data = motor->motor_data;
    encoder = (uint16_t)(((uint16_t)frame[0] << 8) | frame[1]);
    if (data->encoder_initialized == 0U) {
        data->last_encoder = encoder;
        data->position_continuous_rad = (float)encoder * TWO_PI / M3508_ENCODER;
        data->encoder_initialized = 1U;
    } else {
        delta_encoder = (int32_t)encoder - (int32_t)data->last_encoder;
        if (delta_encoder > M3508_ENCODER / 2) { delta_encoder -= M3508_ENCODER; }
        else if (delta_encoder < -M3508_ENCODER / 2) { delta_encoder += M3508_ENCODER; }
        data->position_continuous_rad += (float)delta_encoder * TWO_PI / M3508_ENCODER;
        data->last_encoder = encoder;
    }
    data->encoder = encoder;
    data->speed_rpm = (int16_t)(((uint16_t)frame[2] << 8) | frame[3]);
    data->current = (int16_t)(((uint16_t)frame[4] << 8) | frame[5]);
    data->temperature = frame[6];
    data->error = frame[7];
    data->position_rad = data->position_continuous_rad;
    data->velocity_rpm = (float)data->speed_rpm;
    data->torque_current = (float)data->current * M3508_MAX_TORQUE_CURRENT_A / M3508_OUTPUT_LIMIT;
}

// 打包3508电机数据组帧发送
void m3508_send_ctrl_cmd(struct motor_device *motor)
{
    if (motor == NULL || motor->motor_data == NULL) { return; }
    m3508_send_group(motor->motor_can_handle);
}

// 使能3508
void m3508_enable(struct motor_device *motor)
{
    m3508_data_t *data;
    if (motor == NULL || motor->motor_data == NULL || !motor_is_online(motor)) { return; }
    data = motor->motor_data;
    data->target_position_rad = data->position_rad;
    data->target_velocity_rpm = 0.0f;
    data->target_acceleration_rad_s2 = 0.0f;
    data->trace.active = 0U;
    pid_reset(&data->position_pid);
    pid_reset(&data->velocity_pid);
    data->last_update_tick = 0U;
    data->enabled = 1U;
}

// 失能3508
void m3508_disable(struct motor_device *motor)
{
    m3508_data_t *data;
    if (motor == NULL || motor->motor_data == NULL) { return; }
    data = motor->motor_data;
    data->enabled = 0U;
    data->output_current_code = 0.0f;
    data->target_position_rad = data->position_rad;
    data->target_velocity_rpm = 0.0f;
    data->target_acceleration_rad_s2 = 0.0f;
    data->trace.active = 0U;
    pid_reset(&data->position_pid);
    pid_reset(&data->velocity_pid);
    data->last_update_tick = 0U;
    m3508_send_ctrl_cmd(motor);
}

// 不断更新m3508应该下发的值
void m3508_update(struct motor_device *motor)
{
    m3508_data_t *data;
    float position_error;
    float position_pid_out_rpm;
    float velocity_pid_out_rpm;
    float output_current_code;
    float all_torque;
    float acc_torque;
    float grativity_torque;
    float friction_torque;
    float ff_current;
    float dt;
    uint32_t now;
    if (motor == NULL || motor->motor_data == NULL) { return; }
    data = motor->motor_data;
    if (data->enabled == 0U || data->error != 0U || motor->last_rx_tick == 0U || (HAL_GetTick() - motor->last_rx_tick) > MOTOR_OFFLINE_MS) {
        data->output_current_code = 0.0f;
        data->target_position_rad = data->position_rad;
        data->target_velocity_rpm = 0.0f;
        data->target_acceleration_rad_s2 = 0.0f;
        data->trace.active = 0U;
        pid_reset(&data->position_pid);
        pid_reset(&data->velocity_pid);
        data->last_update_tick = 0U;
        return;
    }
    now = HAL_GetTick();
    if (data->last_update_tick == 0U) { dt = MOTOR_CONTROL_S; }
    else {
        dt = (float)(now - data->last_update_tick) * 0.001f;
        dt = clamp_float(dt, 0.0001f, 0.020f);
    }
    data->last_update_tick = now;
    position_error = wrap_to_pi(data->target_position_rad - data->position_rad);
    position_pid_out_rpm = pid_update(&data->position_pid, position_error, 0.0f, dt);
    velocity_pid_out_rpm = data->target_velocity_rpm + position_pid_out_rpm;
    velocity_pid_out_rpm = clamp_float(velocity_pid_out_rpm, -M3508_SPEED_LIMIT_RPM, M3508_SPEED_LIMIT_RPM);
    output_current_code = pid_update(&data->velocity_pid, velocity_pid_out_rpm, data->velocity_rpm, dt);
    acc_torque = data->target_acceleration_rad_s2 * data->rotational_inertia_kg_m2;
    grativity_torque = 0.0f;
    friction_torque = data->friction_torque;
    all_torque = acc_torque + grativity_torque + friction_torque;
    ff_current = all_torque / data->K_t_Nm_A;
    output_current_code += ff_current * M3508_OUTPUT_LIMIT / M3508_MAX_TORQUE_CURRENT_A;
    data->output_current_code = clamp_float(output_current_code, -M3508_OUTPUT_LIMIT, M3508_OUTPUT_LIMIT);
}

// 设定轨迹
static void m3508_trace_set_target(const struct motor_device *motor,
                                   float target_position_rad,
                                   float target_velocity_rpm,
                                   float target_acceleration_rad_s2)
{
    m3508_data_t *data;
    if (motor == NULL || motor->motor_data == NULL) { return; }
    data = motor->motor_data;
    data->target_position_rad = target_position_rad;
    data->target_velocity_rpm = target_velocity_rpm;
    data->target_acceleration_rad_s2 = target_acceleration_rad_s2;
}

// 得到轨迹需要的时间
static float m3508_trace_resolve_duration(float delta_position_rad, float requested_duration_s)
{
    float distance_rad;
    float maximum_velocity_rad_s;
    float minimum_velocity_duration_s;
    float minimum_acceleration_duration_s;
    float actual_duration_s;
    distance_rad = abs_float(delta_position_rad);
    maximum_velocity_rad_s = M3508_SPEED_LIMIT_RPM * RPM_TO_RAD_S;
    minimum_velocity_duration_s = MOTOR_QUINTIC_PEAK_VELOCITY_FACTOR * distance_rad / maximum_velocity_rad_s;
    minimum_acceleration_duration_s = motor_sqrt_float(MOTOR_QUINTIC_PEAK_ACCELERATION_FACTOR * distance_rad / M3508_TRACE_MAX_ACCELERATION_RAD_S2);
    actual_duration_s = requested_duration_s;
    if (actual_duration_s < minimum_velocity_duration_s) { actual_duration_s = minimum_velocity_duration_s; }
    if (actual_duration_s < minimum_acceleration_duration_s) { actual_duration_s = minimum_acceleration_duration_s; }
    if (actual_duration_s < MOTOR_CONTROL_S) { actual_duration_s = MOTOR_CONTROL_S; }
    return actual_duration_s;
}

// 初始化生成轨迹参数
void m3508_set_trace(const struct motor_device *motor,
                            float target_position_rad,
                            float duration_s)
{
    m3508_data_t *data;
    float delta_position_rad;
    if (motor == NULL || motor->motor_data == NULL) { return; }
    data = motor->motor_data;
    if (!motor_is_online(motor) || data->enabled == 0U || duration_s <= 0.0f) { return; }
    data->trace.start_position_rad = data->position_rad;
    delta_position_rad = wrap_to_pi(target_position_rad - data->trace.start_position_rad);
    data->trace.delta_position_rad = delta_position_rad;
    data->trace.end_position_rad = data->trace.start_position_rad + delta_position_rad;
    data->trace.duration_s = m3508_trace_resolve_duration(delta_position_rad, duration_s);
    data->trace.position_ref_rad = data->trace.start_position_rad;
    data->trace.velocity_ref_rad_s = 0.0f;
    data->trace.acceleration_ref_rad_s2 = 0.0f;
    data->trace.start_tick = HAL_GetTick();
    if (delta_position_rad > - TWO_PI / M3508_ENCODER && delta_position_rad < TWO_PI / M3508_ENCODER) {
        data->trace.active = 0U;
        m3508_trace_set_target(motor, data->trace.end_position_rad, 0.0f, 0.0f);
        return;
    }
    data->trace.active = 1U;
    m3508_trace_set_target(motor, data->trace.start_position_rad, 0.0f, 0.0f);
}

// 依据strace进度更新strace
void m3508_trace_update(struct motor_device *motor)
{
    m3508_data_t *data;
    motor_trace_t *trace;
    float elapsed_s;
    float normalized_time;
    float normalized_time_2;
    float normalized_time_3;
    float normalized_time_4;
    float normalized_time_5;
    float position_scale;
    float velocity_scale;
    float acceleration_scale;
    if (motor == NULL || motor->motor_data == NULL) { return; }
    data = motor->motor_data;
    trace = &data->trace;
    if (trace->active == 0U) { return; }
    if (data->enabled == 0U || !motor_is_online(motor)) {
        trace->active = 0U;
        data->target_position_rad = data->position_rad;
        data->target_velocity_rpm = 0.0f;
        data->target_acceleration_rad_s2 = 0.0f;
        return;
    }
    elapsed_s = (float)(HAL_GetTick() - trace->start_tick) * 0.001f;
    if (elapsed_s >= trace->duration_s) {
        trace->position_ref_rad = trace->end_position_rad;
        trace->velocity_ref_rad_s = 0.0f;
        trace->acceleration_ref_rad_s2 = 0.0f;
        trace->active = 0U;
        m3508_trace_set_target(motor, trace->end_position_rad, 0.0f, 0.0f);
        return;
    }
    normalized_time = clamp_float(elapsed_s / trace->duration_s, 0.0f, 1.0f);
    normalized_time_2 = normalized_time * normalized_time;
    normalized_time_3 = normalized_time_2 * normalized_time;
    normalized_time_4 = normalized_time_3 * normalized_time;
    normalized_time_5 = normalized_time_4 * normalized_time;
    position_scale = 10.0f * normalized_time_3 - 15.0f * normalized_time_4 + 6.0f * normalized_time_5;
    velocity_scale = 30.0f * normalized_time_2 - 60.0f * normalized_time_3 + 30.0f * normalized_time_4;
    acceleration_scale = 60.0f * normalized_time - 180.0f * normalized_time_2 + 120.0f * normalized_time_3;
    trace->position_ref_rad = trace->start_position_rad + trace->delta_position_rad * position_scale;
    trace->velocity_ref_rad_s = trace->delta_position_rad / trace->duration_s * velocity_scale;
    trace->acceleration_ref_rad_s2 = trace->delta_position_rad / (trace->duration_s * trace->duration_s) * acceleration_scale;
    m3508_trace_set_target(motor, trace->position_ref_rad, trace->velocity_ref_rad_s * RAD_S_TO_RPM, trace->acceleration_ref_rad_s2);
}

// 设置目标位置，参数顺序为目标位置，速度前馈，加速度前馈
void m3508_set_target(const struct motor_device *motor, int para_num, ...)
{
    m3508_data_t *data;
    float target_position_rad;
    float target_velocity_rpm;
    float target_acceleration_rad_s2;
    va_list arguments;
    if (motor == NULL || motor->motor_data == NULL) { return; }
    data = motor->motor_data;
    target_position_rad = data->target_position_rad;
    target_velocity_rpm = data->target_velocity_rpm;
    target_acceleration_rad_s2 = data->target_acceleration_rad_s2;
    va_start(arguments, para_num);
    if (para_num >= 1) {
        target_position_rad = (float)va_arg(arguments, double);
        target_velocity_rpm = 0.0f;
        target_acceleration_rad_s2 = 0.0f;
    }
    if (para_num >= 2) {
        target_velocity_rpm = (float)va_arg(arguments, double);
        target_acceleration_rad_s2 = 0.0f;
    }
    if (para_num >= 3) { target_acceleration_rad_s2 = (float)va_arg(arguments, double); }
    va_end(arguments);
    data->trace.active = 0U;
    m3508_trace_set_target(motor, target_position_rad, target_velocity_rpm, target_acceleration_rad_s2);
}

// 获取电机的某个状态值，存入value中
void m3508_get_status(const struct motor_device *motor, const char *which,
                             void *value)
{
    m3508_data_t *data;
    if (motor == NULL || motor->motor_data == NULL || which == NULL || value == NULL) { return; }
    data = motor->motor_data;
    if (strcmp(which, "POS") == 0) { *(float *)value = data->position_rad; }
    else if (strcmp(which, "VEL") == 0) { *(float *)value = data->velocity_rpm; }
    else if (strcmp(which, "CURRENT") == 0) { *(float *)value = data->torque_current; }
    else if (strcmp(which, "TEMP") == 0) { *(uint8_t *)value = data->temperature; }
    else if (strcmp(which, "ERR") == 0) { *(uint8_t *)value = data->error; }
    else if (strcmp(which, "TARGET_POS") == 0) { *(float *)value = data->target_position_rad; }
    else if (strcmp(which, "TARGET_VEL") == 0) { *(float *)value = data->target_velocity_rpm; }
    else if (strcmp(which, "TARGET_ACC") == 0) { *(float *)value = data->target_acceleration_rad_s2; }
    else if (strcmp(which, "ENC") == 0) { *(uint16_t *)value = data->encoder; }
}

// 运行时把对应的pid参数修改为value
void m3508_set_para(const struct motor_device *motor, const char *which,
                           const void *value)
{
    m3508_data_t *data;
    if (motor == NULL || motor->motor_data == NULL || which == NULL || value == NULL) { return; }
    data = motor->motor_data;
    if (strcmp(which, "POS_KP") == 0) { data->position_pid.kp = *(const float *)value; }
    else if (strcmp(which, "POS_KI") == 0) { data->position_pid.ki = *(const float *)value; }
    else if (strcmp(which, "POS_KD") == 0) { data->position_pid.kd = *(const float *)value; }
    else if (strcmp(which, "VEL_KP") == 0) { data->velocity_pid.kp = *(const float *)value; }
    else if (strcmp(which, "VEL_KI") == 0) { data->velocity_pid.ki = *(const float *)value; }
    else if (strcmp(which, "VEL_KD") == 0) { data->velocity_pid.kd = *(const float *)value; }
}

/* ------------------------------ MG4005E ---------------------------------- */

static void mg4005e_send_command(struct motor_device *motor, uint8_t command,
                                 const uint8_t payload[8])
{
    uint8_t frame[8] = {0};

    if (motor == NULL || payload == NULL || motor->motor_can_handle == NULL) {
        return;
    }
    memcpy(frame, payload, sizeof(frame));
    frame[0] = command;
    motor_send_standard(motor->motor_can_handle, motor->motor_id, frame);
}

void mg4005e_init(struct motor_device *motor, uint32_t motor_id,
                  FDCAN_HandleTypeDef *can_handle, int para_num, ...)
{
    mg4005e_data_t *data;
    uint32_t command_id;

    if (motor == NULL || motor->motor_data == NULL || can_handle == NULL) {
        return;
    }

    if (motor_id >= 1U && motor_id <= 32U) {
        command_id = MG4005E_CAN_BASE_ID + motor_id;
    } else if (motor_id > MG4005E_CAN_BASE_ID &&
               motor_id <= MG4005E_CAN_BASE_ID + 32U) {
        command_id = motor_id;
    } else {
        return;
    }

    data = motor->motor_data;
    memset(data, 0, sizeof(*data));
    data->device_id = (uint8_t)(command_id - MG4005E_CAN_BASE_ID);
    motor->motor_id = command_id;
    motor->motor_can_handle = can_handle;
    motor->last_rx_tick = 0U;
    data->target_velocity_dps = 0.0f;
    data->position_rad = 0.0f;
    data->velocity_dps = 0.0f;
    data->torque_current_a = 0.0f;
    data->bus_voltage_v = 0.0f;
    data->bus_current_a = 0.0f;
    data->enabled = 0U;
    data->enable_requested = 0U;
    (void)para_num;
}

void mg4005e_feedback_calculate(const struct motor_device *motor,
                                const uint8_t frame[8])
{
    mg4005e_data_t *data;
    uint16_t encoder;
    int32_t delta_encoder;
    int16_t iq_code;
    int16_t speed_dps;
    int16_t voltage_code;
    int16_t current_code;

    if (motor == NULL || motor->motor_data == NULL || frame == NULL) {
        return;
    }

    data = motor->motor_data;
    switch (frame[0]) {
    case 0x9AU:
    case 0x9BU:
        voltage_code = (int16_t)((uint16_t)frame[2] |
                                 ((uint16_t)frame[3] << 8));
        current_code = (int16_t)((uint16_t)frame[4] |
                                 ((uint16_t)frame[5] << 8));
        data->temperature = (int8_t)frame[1];
        data->bus_voltage_v = (float)voltage_code * 0.01f;
        data->bus_current_a = (float)current_code * 0.01f;
        data->motor_state = frame[6];
        data->error = frame[7];
        if (data->motor_state == 0x10U || data->error != 0U) {
            data->enabled = 0U;
        } else {
            data->enabled = data->enable_requested;
        }
        break;

    case 0x9CU:
    case 0xA2U:
    case 0xA0U:
    case 0xA1U:
    case 0xA3U:
    case 0xA4U:
    case 0xA5U:
    case 0xA6U:
    case 0xA7U:
    case 0xA8U:
        data->temperature = (int8_t)frame[1];
        iq_code = (int16_t)((uint16_t)frame[2] |
                            ((uint16_t)frame[3] << 8));
        speed_dps = (int16_t)((uint16_t)frame[4] |
                              ((uint16_t)frame[5] << 8));
        encoder = (uint16_t)((uint16_t)frame[6] |
                             ((uint16_t)frame[7] << 8));

        if (data->encoder_initialized == 0U) {
            data->last_encoder = encoder;
            data->position_continuous_rad =
                (float)encoder * TWO_PI / MG4005E_ENCODER;
            data->encoder_initialized = 1U;
        } else {
            delta_encoder = (int32_t)encoder - (int32_t)data->last_encoder;
            if (delta_encoder > 32768) {
                delta_encoder -= 65536;
            } else if (delta_encoder < -32768) {
                delta_encoder += 65536;
            }
            data->position_continuous_rad +=
                (float)delta_encoder * TWO_PI / MG4005E_ENCODER;
            data->last_encoder = encoder;
        }

        data->encoder = encoder;
        data->iq_code = iq_code;
        data->speed_dps = speed_dps;
        data->position_rad = data->position_continuous_rad;
        data->velocity_dps = (float)speed_dps;
        data->torque_current_a = (float)iq_code * 66.0f / 4096.0f;
        break;

    case 0x80U:
        data->enabled = 0U;
        data->motor_state = 0x10U;
        break;

    case 0x88U:
        data->enabled = 1U;
        data->motor_state = 0x00U;
        break;

    default:
        break;
    }
}

void mg4005e_send_ctrl_cmd(struct motor_device *motor)
{
    mg4005e_data_t *data;
    int32_t speed_control;
    float speed_scaled;
    uint16_t torque_limit;
    uint8_t frame[8] = {0};

    if (motor == NULL || motor->motor_data == NULL) {
        return;
    }
    data = motor->motor_data;
    if (data->enable_requested == 0U || data->enabled == 0U) {
        return;
    }

    speed_scaled = clamp_float(data->target_velocity_dps,
                               -MG4005E_SPEED_LIMIT_DPS,
                               MG4005E_SPEED_LIMIT_DPS) * 100.0f;
    speed_control = (speed_scaled >= 0.0f) ?
                    (int32_t)(speed_scaled + 0.5f) :
                    (int32_t)(speed_scaled - 0.5f);
    torque_limit = (uint16_t)MG4005E_TORQUE_LIMIT_CODE;
    frame[2] = (uint8_t)torque_limit;
    frame[3] = (uint8_t)(torque_limit >> 8);
    frame[4] = (uint8_t)(uint32_t)speed_control;
    frame[5] = (uint8_t)((uint32_t)speed_control >> 8);
    frame[6] = (uint8_t)((uint32_t)speed_control >> 16);
    frame[7] = (uint8_t)((uint32_t)speed_control >> 24);
    mg4005e_send_command(motor, 0xA2U, frame);
}

void mg4005e_enable(struct motor_device *motor)
{
    mg4005e_data_t *data;
    uint8_t frame[8] = {0};

    if (motor == NULL || motor->motor_data == NULL) {
        return;
    }
    data = motor->motor_data;
    data->enable_requested = 1U;
    data->enabled = 0U;
    data->motor_state = 0x00U;
    data->last_enable_cmd_tick = HAL_GetTick();
    mg4005e_send_command(motor, 0x88U, frame);
}

void mg4005e_disable(struct motor_device *motor)
{
    mg4005e_data_t *data;
    uint8_t frame[8] = {0};

    if (motor == NULL || motor->motor_data == NULL) {
        return;
    }
    data = motor->motor_data;
    data->enable_requested = 0U;
    data->enabled = 0U;
    data->target_velocity_dps = 0.0f;
    data->motor_state = 0x10U;
    mg4005e_send_command(motor, 0x80U, frame);
}

void mg4005e_update(struct motor_device *motor)
{
    mg4005e_data_t *data;
    uint8_t frame[8] = {0};
    uint32_t now;

    if (motor == NULL || motor->motor_data == NULL) {
        return;
    }
    data = motor->motor_data;
    if (data->enable_requested == 0U) {
        return;
    }

    now = HAL_GetTick();
    if (!motor_is_online(motor)) {
        data->enabled = 0U;
        if ((uint32_t)(now - data->last_enable_cmd_tick) >= 100U) {
            data->last_enable_cmd_tick = now;
            mg4005e_send_command(motor, 0x88U, frame);
        }
        if ((uint32_t)(now - data->last_state_request_tick) >= 50U) {
            data->last_state_request_tick = now;
            mg4005e_send_command(motor, 0x9AU, frame);
        }
        return;
    }
    if (data->error != 0U) {
        if ((uint32_t)(now - data->last_state_request_tick) >= 50U) {
            data->last_state_request_tick = now;
            mg4005e_send_command(motor, 0x9BU, frame);
        }
        return;
    }
    if (data->enabled == 0U) {
        if ((uint32_t)(now - data->last_enable_cmd_tick) >= 20U) {
            data->last_enable_cmd_tick = now;
            mg4005e_send_command(motor, 0x88U, frame);
        }
    } else {
        mg4005e_send_ctrl_cmd(motor);
    }

    if ((uint32_t)(now - data->last_state_request_tick) >= 50U) {
        data->last_state_request_tick = now;
        mg4005e_send_command(motor, 0x9AU, frame);
    }
}

void mg4005e_set_trace(const struct motor_device *motor,
                       float target_position_rad, float duration_s)
{
    (void)motor;
    (void)target_position_rad;
    (void)duration_s;
    return;
}

void mg4005e_trace_update(struct motor_device *motor)
{
    (void)motor;
    return;
}

void mg4005e_set_target(const struct motor_device *motor, int para_num, ...)
{
    mg4005e_data_t *data;
    float first_target;
    va_list arguments;

    if (motor == NULL || motor->motor_data == NULL || para_num <= 0) {
        return;
    }
    data = motor->motor_data;
    va_start(arguments, para_num);
    first_target = (float)va_arg(arguments, double);
    if (para_num >= 2) {
        /* Keep compatibility with the common position, velocity signature. */
        data->target_velocity_dps = (float)va_arg(arguments, double);
    } else {
        data->target_velocity_dps = first_target;
    }
    va_end(arguments);
    data->target_velocity_dps = clamp_float(data->target_velocity_dps,
                                            -MG4005E_SPEED_LIMIT_DPS,
                                            MG4005E_SPEED_LIMIT_DPS);
}

void mg4005e_get_status(const struct motor_device *motor, const char *which,
                        void *value)
{
    mg4005e_data_t *data;

    if (motor == NULL || motor->motor_data == NULL || which == NULL ||
        value == NULL) {
        return;
    }
    data = motor->motor_data;
    if (strcmp(which, "POS") == 0) { *(float *)value = data->position_rad; }
    else if (strcmp(which, "VEL") == 0) { *(float *)value = data->velocity_dps; }
    else if (strcmp(which, "VEL_DPS") == 0) { *(float *)value = data->velocity_dps; }
    else if (strcmp(which, "CURRENT") == 0) { *(float *)value = data->torque_current_a; }
    else if (strcmp(which, "TEMP") == 0) { *(int8_t *)value = data->temperature; }
    else if (strcmp(which, "VOLTAGE") == 0) { *(float *)value = data->bus_voltage_v; }
    else if (strcmp(which, "BUS_CURRENT") == 0) { *(float *)value = data->bus_current_a; }
    else if (strcmp(which, "ERR") == 0) { *(uint8_t *)value = data->error; }
    else if (strcmp(which, "STATE") == 0) { *(uint8_t *)value = data->motor_state; }
    else if (strcmp(which, "ID") == 0) { *(uint8_t *)value = data->device_id; }
    else if (strcmp(which, "ENC") == 0) { *(uint16_t *)value = data->encoder; }
    else if (strcmp(which, "TARGET_VEL") == 0) { *(float *)value = data->target_velocity_dps; }
    else if (strcmp(which, "TARGET_VEL_DPS") == 0) { *(float *)value = data->target_velocity_dps; }
    else if (strcmp(which, "TARGET_POS") == 0) { *(float *)value = data->position_rad; }
    else if (strcmp(which, "TARGET_ACC") == 0) { *(float *)value = 0.0f; }
}

void mg4005e_set_para(const struct motor_device *motor, const char *which,
                      const void *value)
{
    /* PI and current-loop parameters are configured in the MG driver. */
    (void)motor;
    (void)which;
    (void)value;
    return;
}

/* ------------------------------ DM3507 ---------------------------------- */

static void dm3507_send_special(struct motor_device *motor, uint8_t command)
{
    uint8_t frame[8] = {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, command};
    dm3507_data_t *data;
    if (motor == NULL || motor->motor_data == NULL) { return; }
    data = motor->motor_data;
    motor_send_standard(motor->motor_can_handle, data->command_id, frame);
}

void dm3507_init(struct motor_device *motor, uint32_t motor_id, FDCAN_HandleTypeDef *can_handle, int para_num, ...)
{
    dm3507_data_t *data;
    const dm3507_pid_config_t *pid_config;
    float rotational_inertia_kg_m2;
    float friction_torque;
    uint32_t master_id;
    uint32_t command_id;
    if (motor == NULL || motor->motor_data == NULL || can_handle == NULL) { return; }
    data = motor->motor_data;
    pid_config = data->pid_config;
    rotational_inertia_kg_m2 = data->rotational_inertia_kg_m2;
    friction_torque = data->friction_torque;
    master_id = data->master_id;
    command_id = data->command_id;
    if (pid_config == NULL || master_id > 0x7FFU || command_id > 0x7FFU) { return; }
    memset(data, 0, sizeof(*data));
    data->pid_config = pid_config;
    data->master_id = master_id;
    data->command_id = command_id;
    data->rotational_inertia_kg_m2 = rotational_inertia_kg_m2;
    data->friction_torque = friction_torque;
    data->p_max = DM3507_P_MAX;
    data->v_max = DM3507_V_MAX;
    data->t_max = DM3507_T_MAX;
    data->position_pid = pid_config->position_pid;
    data->velocity_pid = pid_config->velocity_pid;
    motor->motor_id = data->master_id;
    motor->motor_can_handle = can_handle;
    pid_reset(&data->position_pid);
    pid_reset(&data->velocity_pid);
    (void)motor_id;
    (void)para_num;
}

void dm3507_feedback_calculate(const struct motor_device *motor, const uint8_t frame[8])
{
    dm3507_data_t *data;
    uint16_t position;
    uint16_t velocity;
    uint16_t torque;
    if (motor == NULL || motor->motor_data == NULL || frame == NULL) { return; }
    data = motor->motor_data;
    data->can_id = frame[0] & 0x0FU;
    data->error = frame[0] >> 4;
    data->enabled = (data->error == 0x1U) ? 1U : 0U;
    position = (uint16_t)(((uint16_t)frame[1] << 8) | frame[2]);
    velocity = (uint16_t)(((uint16_t)frame[3] << 4) | (frame[4] >> 4));
    torque = (uint16_t)(((uint16_t)(frame[4] & 0x0FU) << 8) | frame[5]);
    data->position_rad = uint_to_float(position, -data->p_max, data->p_max, 16U);
    data->velocity_rad_s = uint_to_float(velocity, -data->v_max, data->v_max, 12U);
    data->torque_nm = uint_to_float(torque, -data->t_max, data->t_max, 12U);
    data->mos_temperature = frame[6];
    data->rotor_temperature = frame[7];
}

void dm3507_send_ctrl_cmd(struct motor_device *motor)
{
    dm3507_data_t *data;
    uint16_t position;
    uint16_t velocity;
    uint16_t torque;
    uint8_t frame[8];
    if (motor == NULL || motor->motor_data == NULL) { return; }
    data = motor->motor_data;
    if (data->enable_requested == 0U || data->enabled == 0U) { return; }
    position = float_to_uint(0.0f, -data->p_max, data->p_max, 16U);
    velocity = float_to_uint(0.0f, -data->v_max, data->v_max, 12U);
    torque = float_to_uint(clamp_float(data->output_torque_nm, -3.0f, 3.0f), -data->t_max, data->t_max, 12U);
    frame[0] = (uint8_t)(position >> 8);
    frame[1] = (uint8_t)position;
    frame[2] = (uint8_t)(velocity >> 4);
    frame[3] = (uint8_t)(velocity << 4);
    frame[4] = 0U;
    frame[5] = 0U;
    frame[6] = (uint8_t)(torque >> 8);
    frame[7] = (uint8_t)torque;
    motor_send_standard(motor->motor_can_handle, data->command_id, frame);
}

void dm3507_enable(struct motor_device *motor)
{
    dm3507_data_t *data;
    if (motor == NULL || motor->motor_data == NULL) { return; }
    data = motor->motor_data;
    data->enable_requested = 1U;
    data->enabled = 0U;
    data->hold_position_pending = motor_is_online(motor) ? 0U : 1U;
    if (data->hold_position_pending == 0U) { data->target_position_rad = data->position_rad; }
    data->target_velocity_rad_s = 0.0f;
    data->target_acceleration_rad_s2 = 0.0f;
    data->output_torque_nm = 0.0f;
    data->trace.active = 0U;
    pid_reset(&data->position_pid);
    pid_reset(&data->velocity_pid);
    data->last_update_tick = 0U;
    dm3507_send_special(motor, 0xFBU);
    dm3507_send_special(motor, 0xFCU);
    data->last_clear_cmd_tick = HAL_GetTick();
    data->last_enable_cmd_tick = data->last_clear_cmd_tick;
}

void dm3507_disable(struct motor_device *motor)
{
    dm3507_data_t *data;
    if (motor == NULL || motor->motor_data == NULL) { return; }
    data = motor->motor_data;
    data->enable_requested = 0U;
    data->enabled = 0U;
    data->hold_position_pending = 0U;
    data->target_position_rad = data->position_rad;
    data->target_velocity_rad_s = 0.0f;
    data->target_acceleration_rad_s2 = 0.0f;
    data->output_torque_nm = 0.0f;
    data->trace.active = 0U;
    pid_reset(&data->position_pid);
    pid_reset(&data->velocity_pid);
    data->last_update_tick = 0U;
    dm3507_send_special(motor, 0xFDU);
}

void dm3507_update(struct motor_device *motor)
{
    dm3507_data_t *data;
    float position_error;
    float position_pid_out_rad_s;
    float velocity_target_rad_s;
    float output_torque_nm;
    float acc_torque_nm;
    float gravity_torque_nm;
    float friction_torque_nm;
    float feedforward_torque_nm;
    float dt;
    uint32_t now;
    if (motor == NULL || motor->motor_data == NULL) { return; }
    data = motor->motor_data;
    if (data->enable_requested == 0U) { return; }
    if (motor->last_rx_tick == 0U || (HAL_GetTick() - motor->last_rx_tick) > MOTOR_OFFLINE_MS) {
        data->enabled = 0U;
        data->output_torque_nm = 0.0f;
        data->trace.active = 0U;
        pid_reset(&data->position_pid);
        pid_reset(&data->velocity_pid);
        data->last_update_tick = 0U;
        return;
    }
    now = HAL_GetTick();
    if (data->hold_position_pending != 0U) {
        // 防跳变
        data->target_position_rad = data->position_rad;
        data->target_velocity_rad_s = 0.0f;
        data->target_acceleration_rad_s2 = 0.0f;
        data->output_torque_nm = 0.0f;
        pid_reset(&data->position_pid);
        pid_reset(&data->velocity_pid);
        data->last_update_tick = 0U;
        data->hold_position_pending = 0U;
        return;
    }
    if (data->error >= 0x2U && data->error <= 0xEU) {
        data->output_torque_nm = 0.0f;
        data->trace.active = 0U;
        if ((uint32_t)(now - data->last_clear_cmd_tick) >= 50U) { dm3507_send_special(motor, 0xFBU); data->last_clear_cmd_tick = now; }
        return;
    }
    if (data->error != 0x1U) {
        data->output_torque_nm = 0.0f;
        data->trace.active = 0U;
        if ((uint32_t)(now - data->last_enable_cmd_tick) >= 20U) { dm3507_send_special(motor, 0xFCU); data->last_enable_cmd_tick = now; }
        return;
    }
    if (data->last_update_tick == 0U) { dt = MOTOR_CONTROL_S; }
    else { dt = clamp_float((float)(now - data->last_update_tick) * 0.001f, 0.0001f, 0.020f); }
    data->last_update_tick = now;
    position_error = wrap_to_pi(data->target_position_rad - data->position_rad);
    position_pid_out_rad_s = pid_update(&data->position_pid, position_error, 0.0f, dt);
    velocity_target_rad_s = data->target_velocity_rad_s + position_pid_out_rad_s;
    velocity_target_rad_s = clamp_float(velocity_target_rad_s, -48.17f, 48.17f);
    output_torque_nm = pid_update(&data->velocity_pid, velocity_target_rad_s, data->velocity_rad_s, dt);
    acc_torque_nm = data->target_acceleration_rad_s2 * data->rotational_inertia_kg_m2;
    gravity_torque_nm = 0.0f;
    friction_torque_nm = data->friction_torque;
    feedforward_torque_nm = acc_torque_nm + gravity_torque_nm + friction_torque_nm;
    output_torque_nm += feedforward_torque_nm;
    data->output_torque_nm = clamp_float(output_torque_nm, -3.0f, 3.0f);
    dm3507_send_ctrl_cmd(motor);
}

static void dm3507_trace_set_target(const struct motor_device *motor, float target_position_rad, float target_velocity_rad_s, float target_acceleration_rad_s2)
{
    dm3507_data_t *data;
    if (motor == NULL || motor->motor_data == NULL) { return; }
    data = motor->motor_data;
    data->target_position_rad = target_position_rad;
    data->target_velocity_rad_s = clamp_float(target_velocity_rad_s, -48.17f, 48.17f);
    data->target_acceleration_rad_s2 = target_acceleration_rad_s2;
}

static float dm3507_trace_resolve_duration(float delta_position_rad, float requested_duration_s)
{
    float distance_rad = abs_float(delta_position_rad);
    float minimum_velocity_duration_s = MOTOR_QUINTIC_PEAK_VELOCITY_FACTOR * distance_rad / 48.17f;
    float minimum_acceleration_duration_s = motor_sqrt_float(MOTOR_QUINTIC_PEAK_ACCELERATION_FACTOR * distance_rad / DM3507_TRACE_MAX_ACCELERATION_RAD_S2);
    float actual_duration_s = requested_duration_s;
    if (actual_duration_s < minimum_velocity_duration_s) { actual_duration_s = minimum_velocity_duration_s; }
    if (actual_duration_s < minimum_acceleration_duration_s) { actual_duration_s = minimum_acceleration_duration_s; }
    if (actual_duration_s < MOTOR_CONTROL_S) { actual_duration_s = MOTOR_CONTROL_S; }
    return actual_duration_s;
}

void dm3507_set_trace(const struct motor_device *motor, float target_position_rad, float duration_s)
{
    dm3507_data_t *data;
    float delta_position_rad;
    float position_epsilon_rad;
    if (motor == NULL || motor->motor_data == NULL) { return; }
    data = motor->motor_data;
    if (!motor_is_online(motor) || data->enabled == 0U || duration_s <= 0.0f) { return; }
    data->trace.start_position_rad = data->position_rad;
    delta_position_rad = wrap_to_pi(target_position_rad - data->trace.start_position_rad);
    data->trace.delta_position_rad = delta_position_rad;
    data->trace.end_position_rad = data->trace.start_position_rad + delta_position_rad;
    data->trace.duration_s = dm3507_trace_resolve_duration(delta_position_rad, duration_s);
    data->trace.position_ref_rad = data->trace.start_position_rad;
    data->trace.velocity_ref_rad_s = 0.0f;
    data->trace.acceleration_ref_rad_s2 = 0.0f;
    data->trace.start_tick = HAL_GetTick();
    position_epsilon_rad = 2.0f * data->p_max / 65535.0f;
    if (delta_position_rad > -position_epsilon_rad && delta_position_rad < position_epsilon_rad) {
        data->trace.active = 0U;
        dm3507_trace_set_target(motor, data->trace.end_position_rad, 0.0f, 0.0f);
        return;
    }
    data->trace.active = 1U;
    dm3507_trace_set_target(motor, data->trace.start_position_rad, 0.0f, 0.0f);
}

void dm3507_trace_update(struct motor_device *motor)
{
    dm3507_data_t *data;
    motor_trace_t *trace;
    float elapsed_s;
    float normalized_time;
    float normalized_time_2;
    float normalized_time_3;
    float normalized_time_4;
    float normalized_time_5;
    float position_scale;
    float velocity_scale;
    float acceleration_scale;
    if (motor == NULL || motor->motor_data == NULL) { return; }
    data = motor->motor_data;
    trace = &data->trace;
    if (trace->active == 0U) { return; }
    if (data->enabled == 0U || !motor_is_online(motor)) {
        trace->active = 0U;
        dm3507_trace_set_target(motor, data->position_rad, 0.0f, 0.0f);
        return;
    }
    elapsed_s = (float)(HAL_GetTick() - trace->start_tick) * 0.001f;
    if (elapsed_s >= trace->duration_s) {
        trace->position_ref_rad = trace->end_position_rad;
        trace->velocity_ref_rad_s = 0.0f;
        trace->acceleration_ref_rad_s2 = 0.0f;
        trace->active = 0U;
        dm3507_trace_set_target(motor, trace->end_position_rad, 0.0f, 0.0f);
        return;
    }
    normalized_time = clamp_float(elapsed_s / trace->duration_s, 0.0f, 1.0f);
    normalized_time_2 = normalized_time * normalized_time;
    normalized_time_3 = normalized_time_2 * normalized_time;
    normalized_time_4 = normalized_time_3 * normalized_time;
    normalized_time_5 = normalized_time_4 * normalized_time;
    position_scale = 10.0f * normalized_time_3 - 15.0f * normalized_time_4 + 6.0f * normalized_time_5;
    velocity_scale = 30.0f * normalized_time_2 - 60.0f * normalized_time_3 + 30.0f * normalized_time_4;
    acceleration_scale = 60.0f * normalized_time - 180.0f * normalized_time_2 + 120.0f * normalized_time_3;
    trace->position_ref_rad = trace->start_position_rad + trace->delta_position_rad * position_scale;
    trace->velocity_ref_rad_s = trace->delta_position_rad / trace->duration_s * velocity_scale;
    trace->acceleration_ref_rad_s2 = trace->delta_position_rad / (trace->duration_s * trace->duration_s) * acceleration_scale;
    dm3507_trace_set_target(motor, trace->position_ref_rad, trace->velocity_ref_rad_s, trace->acceleration_ref_rad_s2);
}

void dm3507_set_target(const struct motor_device *motor, int para_num, ...)
{
    dm3507_data_t *data;
    float target_position_rad;
    float target_velocity_rad_s;
    float target_acceleration_rad_s2;
    va_list arguments;
    if (motor == NULL || motor->motor_data == NULL) { return; }
    data = motor->motor_data;
    target_position_rad = data->target_position_rad;
    target_velocity_rad_s = data->target_velocity_rad_s;
    target_acceleration_rad_s2 = data->target_acceleration_rad_s2;
    va_start(arguments, para_num);
    if (para_num >= 1) { target_position_rad = (float)va_arg(arguments, double); target_velocity_rad_s = 0.0f; target_acceleration_rad_s2 = 0.0f; }
    if (para_num >= 2) { target_velocity_rad_s = (float)va_arg(arguments, double); target_acceleration_rad_s2 = 0.0f; }
    if (para_num >= 3) { target_acceleration_rad_s2 = (float)va_arg(arguments, double); }
    va_end(arguments);
    data->trace.active = 0U;
    dm3507_trace_set_target(motor, target_position_rad, target_velocity_rad_s, target_acceleration_rad_s2);
}

void dm3507_get_status(const struct motor_device *motor, const char *which, void *value)
{
    dm3507_data_t *data;
    if (motor == NULL || motor->motor_data == NULL || which == NULL || value == NULL) { return; }
    data = motor->motor_data;
    if (strcmp(which, "POS") == 0) { *(float *)value = data->position_rad; }
    else if (strcmp(which, "VEL") == 0) { *(float *)value = data->velocity_rad_s; }
    else if (strcmp(which, "TORQUE") == 0) { *(float *)value = data->torque_nm; }
    else if (strcmp(which, "TEMP") == 0) { *(uint8_t *)value = data->rotor_temperature; }
    else if (strcmp(which, "MOS_TEMP") == 0) { *(uint8_t *)value = data->mos_temperature; }
    else if (strcmp(which, "ERR") == 0) { *(uint8_t *)value = data->error; }
    else if (strcmp(which, "ID") == 0) { *(uint8_t *)value = data->can_id; }
    else if (strcmp(which, "TARGET_POS") == 0) { *(float *)value = data->target_position_rad; }
    else if (strcmp(which, "TARGET_VEL") == 0) { *(float *)value = data->target_velocity_rad_s; }
    else if (strcmp(which, "TARGET_ACC") == 0) { *(float *)value = data->target_acceleration_rad_s2; }
}

void dm3507_set_para(const struct motor_device *motor, const char *which, const void *value)
{
    dm3507_data_t *data;
    if (motor == NULL || motor->motor_data == NULL || which == NULL || value == NULL) { return; }
    data = motor->motor_data;
    if (strcmp(which, "POS_KP") == 0) { data->position_pid.kp = *(const float *)value; }
    else if (strcmp(which, "POS_KI") == 0) { data->position_pid.ki = *(const float *)value; }
    else if (strcmp(which, "POS_KD") == 0) { data->position_pid.kd = *(const float *)value; }
    else if (strcmp(which, "VEL_KP") == 0) { data->velocity_pid.kp = *(const float *)value; }
    else if (strcmp(which, "VEL_KI") == 0) { data->velocity_pid.ki = *(const float *)value; }
    else if (strcmp(which, "VEL_KD") == 0) { data->velocity_pid.kd = *(const float *)value; }
}

/* ------------------------------ DM4310 ---------------------------------- */

static void dm4310_send_special(struct motor_device *motor, uint8_t command)
{
    uint8_t frame[8] = {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, command};
    dm4310_data_t *data;
    if (motor == NULL || motor->motor_data == NULL) { return; }
    data = motor->motor_data;
    motor_send_standard(motor->motor_can_handle, data->command_id, frame);
}

void dm4310_init(struct motor_device *motor, uint32_t motor_id, FDCAN_HandleTypeDef *can_handle, int para_num, ...)
{
    dm4310_data_t *data;
    const dm4310_pid_config_t *pid_config;
    float rotational_inertia_kg_m2;
    float friction_torque;
    uint32_t master_id;
    uint32_t command_id;
    if (motor == NULL || motor->motor_data == NULL || can_handle == NULL) { return; }
    data = motor->motor_data;
    pid_config = data->pid_config;
    rotational_inertia_kg_m2 = data->rotational_inertia_kg_m2;
    friction_torque = data->friction_torque;
    master_id = data->master_id;
    command_id = data->command_id;
    if (pid_config == NULL || master_id > 0x7FFU || command_id > 0x7FFU) { return; }
    memset(data, 0, sizeof(*data));
    data->pid_config = pid_config;
    data->master_id = master_id;
    data->command_id = command_id;
    data->rotational_inertia_kg_m2 = rotational_inertia_kg_m2;
    data->friction_torque = friction_torque;
    data->p_max = DM4310_P_MAX;
    data->v_max = DM4310_V_MAX;
    data->t_max = DM4310_T_MAX;
    data->position_pid = pid_config->position_pid;
    data->velocity_pid = pid_config->velocity_pid;
    motor->motor_id = data->master_id;
    motor->motor_can_handle = can_handle;
    pid_reset(&data->position_pid);
    pid_reset(&data->velocity_pid);
    (void)motor_id;
    (void)para_num;
}

void dm4310_feedback_calculate(const struct motor_device *motor, const uint8_t frame[8])
{
    dm4310_data_t *data;
    uint16_t position;
    uint16_t velocity;
    uint16_t torque;
    if (motor == NULL || motor->motor_data == NULL || frame == NULL) { return; }
    data = motor->motor_data;
    data->can_id = frame[0] & 0x0FU;
    data->error = frame[0] >> 4;
    data->enabled = (data->error == 0x1U) ? 1U : 0U;
    position = (uint16_t)(((uint16_t)frame[1] << 8) | frame[2]);
    velocity = (uint16_t)(((uint16_t)frame[3] << 4) | (frame[4] >> 4));
    torque = (uint16_t)(((uint16_t)(frame[4] & 0x0FU) << 8) | frame[5]);
    data->position_rad = uint_to_float(position, -data->p_max, data->p_max, 16U);
    data->velocity_rad_s = uint_to_float(velocity, -data->v_max, data->v_max, 12U);
    data->torque_nm = uint_to_float(torque, -data->t_max, data->t_max, 12U);
    data->mos_temperature = frame[6];
    data->rotor_temperature = frame[7];
}

void dm4310_send_ctrl_cmd(struct motor_device *motor)
{
    dm4310_data_t *data;
    uint16_t position;
    uint16_t velocity;
    uint16_t torque;
    uint8_t frame[8];
    if (motor == NULL || motor->motor_data == NULL) { return; }
    data = motor->motor_data;
    if (data->enable_requested == 0U || data->enabled == 0U) { return; }
    position = float_to_uint(0.0f, -data->p_max, data->p_max, 16U);
    velocity = float_to_uint(0.0f, -data->v_max, data->v_max, 12U);
    torque = float_to_uint(clamp_float(data->output_torque_nm, -10.0f, 10.0f), -data->t_max, data->t_max, 12U);
    frame[0] = (uint8_t)(position >> 8);
    frame[1] = (uint8_t)position;
    frame[2] = (uint8_t)(velocity >> 4);
    frame[3] = (uint8_t)(velocity << 4);
    frame[4] = 0U;
    frame[5] = 0U;
    frame[6] = (uint8_t)(torque >> 8);
    frame[7] = (uint8_t)torque;
    motor_send_standard(motor->motor_can_handle, data->command_id, frame);
}

void dm4310_enable(struct motor_device *motor)
{
    dm4310_data_t *data;
    if (motor == NULL || motor->motor_data == NULL) { return; }
    data = motor->motor_data;
    data->enable_requested = 1U;
    data->enabled = 0U;
    data->hold_position_pending = motor_is_online(motor) ? 0U : 1U;
    if (data->hold_position_pending == 0U) { data->target_position_rad = data->position_rad; }
    data->target_velocity_rad_s = 0.0f;
    data->target_acceleration_rad_s2 = 0.0f;
    data->output_torque_nm = 0.0f;
    data->trace.active = 0U;
    pid_reset(&data->position_pid);
    pid_reset(&data->velocity_pid);
    data->last_update_tick = 0U;
    dm4310_send_special(motor, 0xFBU);
    dm4310_send_special(motor, 0xFCU);
    data->last_clear_cmd_tick = HAL_GetTick();
    data->last_enable_cmd_tick = data->last_clear_cmd_tick;
}

void dm4310_disable(struct motor_device *motor)
{
    dm4310_data_t *data;
    if (motor == NULL || motor->motor_data == NULL) { return; }
    data = motor->motor_data;
    data->enable_requested = 0U;
    data->enabled = 0U;
    data->hold_position_pending = 0U;
    data->target_position_rad = data->position_rad;
    data->target_velocity_rad_s = 0.0f;
    data->target_acceleration_rad_s2 = 0.0f;
    data->output_torque_nm = 0.0f;
    data->trace.active = 0U;
    pid_reset(&data->position_pid);
    pid_reset(&data->velocity_pid);
    data->last_update_tick = 0U;
    dm4310_send_special(motor, 0xFDU);
}

void dm4310_update(struct motor_device *motor)
{
    dm4310_data_t *data;
    float position_error;
    float position_pid_out_rad_s;
    float velocity_target_rad_s;
    float output_torque_nm;
    float acc_torque_nm;
    float gravity_torque_nm;
    float friction_torque_nm;
    float feedforward_torque_nm;
    float dt;
    uint32_t now;
    if (motor == NULL || motor->motor_data == NULL) { return; }
    data = motor->motor_data;
    if (data->enable_requested == 0U) { return; }
    if (motor->last_rx_tick == 0U || (HAL_GetTick() - motor->last_rx_tick) > MOTOR_OFFLINE_MS) {
        data->enabled = 0U;
        data->output_torque_nm = 0.0f;
        data->trace.active = 0U;
        pid_reset(&data->position_pid);
        pid_reset(&data->velocity_pid);
        data->last_update_tick = 0U;
        return;
    }
    now = HAL_GetTick();
    if (data->hold_position_pending != 0U) {
        // 防跳变
        data->target_position_rad = data->position_rad;
        data->target_velocity_rad_s = 0.0f;
        data->target_acceleration_rad_s2 = 0.0f;
        data->output_torque_nm = 0.0f;
        pid_reset(&data->position_pid);
        pid_reset(&data->velocity_pid);
        data->last_update_tick = 0U;
        data->hold_position_pending = 0U;
        return;
    }
    if (data->error >= 0x8U && data->error <= 0xEU) {
        data->output_torque_nm = 0.0f;
        data->trace.active = 0U;
        if ((uint32_t)(now - data->last_clear_cmd_tick) >= 50U) { dm4310_send_special(motor, 0xFBU); data->last_clear_cmd_tick = now; }
        return;
    }
    if (data->error != 0x1U) {
        data->output_torque_nm = 0.0f;
        data->trace.active = 0U;
        if ((uint32_t)(now - data->last_enable_cmd_tick) >= 20U) { dm4310_send_special(motor, 0xFCU); data->last_enable_cmd_tick = now; }
        return;
    }
    if (data->last_update_tick == 0U) { dt = MOTOR_CONTROL_S; }
    else { dt = clamp_float((float)(now - data->last_update_tick) * 0.001f, 0.0001f, 0.020f); }
    data->last_update_tick = now;
    position_error = wrap_to_pi(data->target_position_rad - data->position_rad);
    position_pid_out_rad_s = pid_update(&data->position_pid, position_error, 0.0f, dt);
    velocity_target_rad_s = data->target_velocity_rad_s + position_pid_out_rad_s;
    velocity_target_rad_s = clamp_float(velocity_target_rad_s, -20.94f, 20.94f);
    output_torque_nm = pid_update(&data->velocity_pid, velocity_target_rad_s, data->velocity_rad_s, dt);
    acc_torque_nm = data->target_acceleration_rad_s2 * data->rotational_inertia_kg_m2;
    gravity_torque_nm = 0.0f;
    friction_torque_nm = data->friction_torque;
    feedforward_torque_nm = acc_torque_nm + gravity_torque_nm + friction_torque_nm;
    output_torque_nm += feedforward_torque_nm;
    data->output_torque_nm = clamp_float(output_torque_nm, -10.0f, 10.0f);
    dm4310_send_ctrl_cmd(motor);
}

static void dm4310_trace_set_target(const struct motor_device *motor, float target_position_rad, float target_velocity_rad_s, float target_acceleration_rad_s2)
{
    dm4310_data_t *data;
    if (motor == NULL || motor->motor_data == NULL) { return; }
    data = motor->motor_data;
    data->target_position_rad = target_position_rad;
    data->target_velocity_rad_s = clamp_float(target_velocity_rad_s, -20.94f, 20.94f);
    data->target_acceleration_rad_s2 = target_acceleration_rad_s2;
}

static float dm4310_trace_resolve_duration(float delta_position_rad, float requested_duration_s)
{
    float distance_rad = abs_float(delta_position_rad);
    float minimum_velocity_duration_s = MOTOR_QUINTIC_PEAK_VELOCITY_FACTOR * distance_rad / 20.94f;
    float minimum_acceleration_duration_s = motor_sqrt_float(MOTOR_QUINTIC_PEAK_ACCELERATION_FACTOR * distance_rad / DM4310_TRACE_MAX_ACCELERATION_RAD_S2);
    float actual_duration_s = requested_duration_s;
    if (actual_duration_s < minimum_velocity_duration_s) { actual_duration_s = minimum_velocity_duration_s; }
    if (actual_duration_s < minimum_acceleration_duration_s) { actual_duration_s = minimum_acceleration_duration_s; }
    if (actual_duration_s < MOTOR_CONTROL_S) { actual_duration_s = MOTOR_CONTROL_S; }
    return actual_duration_s;
}

void dm4310_set_trace(const struct motor_device *motor, float target_position_rad, float duration_s)
{
    dm4310_data_t *data;
    float delta_position_rad;
    float position_epsilon_rad;
    if (motor == NULL || motor->motor_data == NULL) { return; }
    data = motor->motor_data;
    if (!motor_is_online(motor) || data->enabled == 0U || duration_s <= 0.0f) { return; }
    data->trace.start_position_rad = data->position_rad;
    delta_position_rad = wrap_to_pi(target_position_rad - data->trace.start_position_rad);
    data->trace.delta_position_rad = delta_position_rad;
    data->trace.end_position_rad = data->trace.start_position_rad + delta_position_rad;
    data->trace.duration_s = dm4310_trace_resolve_duration(delta_position_rad, duration_s);
    data->trace.position_ref_rad = data->trace.start_position_rad;
    data->trace.velocity_ref_rad_s = 0.0f;
    data->trace.acceleration_ref_rad_s2 = 0.0f;
    data->trace.start_tick = HAL_GetTick();
    position_epsilon_rad = 2.0f * data->p_max / 65535.0f;
    if (delta_position_rad > -position_epsilon_rad && delta_position_rad < position_epsilon_rad) {
        data->trace.active = 0U;
        dm4310_trace_set_target(motor, data->trace.end_position_rad, 0.0f, 0.0f);
        return;
    }
    data->trace.active = 1U;
    dm4310_trace_set_target(motor, data->trace.start_position_rad, 0.0f, 0.0f);
}

void dm4310_trace_update(struct motor_device *motor)
{
    dm4310_data_t *data;
    motor_trace_t *trace;
    float elapsed_s;
    float normalized_time;
    float normalized_time_2;
    float normalized_time_3;
    float normalized_time_4;
    float normalized_time_5;
    float position_scale;
    float velocity_scale;
    float acceleration_scale;
    if (motor == NULL || motor->motor_data == NULL) { return; }
    data = motor->motor_data;
    trace = &data->trace;
    if (trace->active == 0U) { return; }
    if (data->enabled == 0U || !motor_is_online(motor)) {
        trace->active = 0U;
        dm4310_trace_set_target(motor, data->position_rad, 0.0f, 0.0f);
        return;
    }
    elapsed_s = (float)(HAL_GetTick() - trace->start_tick) * 0.001f;
    if (elapsed_s >= trace->duration_s) {
        trace->position_ref_rad = trace->end_position_rad;
        trace->velocity_ref_rad_s = 0.0f;
        trace->acceleration_ref_rad_s2 = 0.0f;
        trace->active = 0U;
        dm4310_trace_set_target(motor, trace->end_position_rad, 0.0f, 0.0f);
        return;
    }
    normalized_time = clamp_float(elapsed_s / trace->duration_s, 0.0f, 1.0f);
    normalized_time_2 = normalized_time * normalized_time;
    normalized_time_3 = normalized_time_2 * normalized_time;
    normalized_time_4 = normalized_time_3 * normalized_time;
    normalized_time_5 = normalized_time_4 * normalized_time;
    position_scale = 10.0f * normalized_time_3 - 15.0f * normalized_time_4 + 6.0f * normalized_time_5;
    velocity_scale = 30.0f * normalized_time_2 - 60.0f * normalized_time_3 + 30.0f * normalized_time_4;
    acceleration_scale = 60.0f * normalized_time - 180.0f * normalized_time_2 + 120.0f * normalized_time_3;
    trace->position_ref_rad = trace->start_position_rad + trace->delta_position_rad * position_scale;
    trace->velocity_ref_rad_s = trace->delta_position_rad / trace->duration_s * velocity_scale;
    trace->acceleration_ref_rad_s2 = trace->delta_position_rad / (trace->duration_s * trace->duration_s) * acceleration_scale;
    dm4310_trace_set_target(motor, trace->position_ref_rad, trace->velocity_ref_rad_s, trace->acceleration_ref_rad_s2);
}

void dm4310_set_target(const struct motor_device *motor, int para_num, ...)
{
    dm4310_data_t *data;
    float target_position_rad;
    float target_velocity_rad_s;
    float target_acceleration_rad_s2;
    va_list arguments;
    if (motor == NULL || motor->motor_data == NULL) { return; }
    data = motor->motor_data;
    target_position_rad = data->target_position_rad;
    target_velocity_rad_s = data->target_velocity_rad_s;
    target_acceleration_rad_s2 = data->target_acceleration_rad_s2;
    va_start(arguments, para_num);
    if (para_num >= 1) { target_position_rad = (float)va_arg(arguments, double); target_velocity_rad_s = 0.0f; target_acceleration_rad_s2 = 0.0f; }
    if (para_num >= 2) { target_velocity_rad_s = (float)va_arg(arguments, double); target_acceleration_rad_s2 = 0.0f; }
    if (para_num >= 3) { target_acceleration_rad_s2 = (float)va_arg(arguments, double); }
    va_end(arguments);
    data->trace.active = 0U;
    dm4310_trace_set_target(motor, target_position_rad, target_velocity_rad_s, target_acceleration_rad_s2);
}

void dm4310_get_status(const struct motor_device *motor, const char *which, void *value)
{
    dm4310_data_t *data;
    if (motor == NULL || motor->motor_data == NULL || which == NULL || value == NULL) { return; }
    data = motor->motor_data;
    if (strcmp(which, "POS") == 0) { *(float *)value = data->position_rad; }
    else if (strcmp(which, "VEL") == 0) { *(float *)value = data->velocity_rad_s; }
    else if (strcmp(which, "TORQUE") == 0) { *(float *)value = data->torque_nm; }
    else if (strcmp(which, "TEMP") == 0) { *(uint8_t *)value = data->rotor_temperature; }
    else if (strcmp(which, "MOS_TEMP") == 0) { *(uint8_t *)value = data->mos_temperature; }
    else if (strcmp(which, "ERR") == 0) { *(uint8_t *)value = data->error; }
    else if (strcmp(which, "ID") == 0) { *(uint8_t *)value = data->can_id; }
    else if (strcmp(which, "TARGET_POS") == 0) { *(float *)value = data->target_position_rad; }
    else if (strcmp(which, "TARGET_VEL") == 0) { *(float *)value = data->target_velocity_rad_s; }
    else if (strcmp(which, "TARGET_ACC") == 0) { *(float *)value = data->target_acceleration_rad_s2; }
}

void dm4310_set_para(const struct motor_device *motor, const char *which, const void *value)
{
    dm4310_data_t *data;
    if (motor == NULL || motor->motor_data == NULL || which == NULL || value == NULL) { return; }
    data = motor->motor_data;
    if (strcmp(which, "POS_KP") == 0) { data->position_pid.kp = *(const float *)value; }
    else if (strcmp(which, "POS_KI") == 0) { data->position_pid.ki = *(const float *)value; }
    else if (strcmp(which, "POS_KD") == 0) { data->position_pid.kd = *(const float *)value; }
    else if (strcmp(which, "VEL_KP") == 0) { data->velocity_pid.kp = *(const float *)value; }
    else if (strcmp(which, "VEL_KI") == 0) { data->velocity_pid.ki = *(const float *)value; }
    else if (strcmp(which, "VEL_KD") == 0) { data->velocity_pid.kd = *(const float *)value; }
}

/* ------------------------------ DM8009P ---------------------------------- */

static void dm8009p_send_special(struct motor_device *motor, uint8_t command)
{
    uint8_t frame[8] = {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, command};
    dm8009p_data_t *data;
    if (motor == NULL || motor->motor_data == NULL) { return; }
    data = motor->motor_data;
    motor_send_standard(motor->motor_can_handle, data->command_id, frame);
}

void dm8009p_init(struct motor_device *motor, uint32_t motor_id, FDCAN_HandleTypeDef *can_handle, int para_num, ...)
{
    dm8009p_data_t *data;
    const dm8009p_pid_config_t *pid_config;
    float rotational_inertia_kg_m2;
    float friction_torque;
    uint32_t master_id;
    uint32_t command_id;
    if (motor == NULL || motor->motor_data == NULL || can_handle == NULL) { return; }
    data = motor->motor_data;
    pid_config = data->pid_config;
    rotational_inertia_kg_m2 = data->rotational_inertia_kg_m2;
    friction_torque = data->friction_torque;
    master_id = data->master_id;
    command_id = data->command_id;
    if (pid_config == NULL || master_id > 0x7FFU || command_id > 0x7FFU) { return; }
    memset(data, 0, sizeof(*data));
    data->pid_config = pid_config;
    data->master_id = master_id;
    data->command_id = command_id;
    data->rotational_inertia_kg_m2 = rotational_inertia_kg_m2;
    data->friction_torque = friction_torque;
    data->p_max = DM8009P_P_MAX;
    data->v_max = DM8009P_V_MAX;
    data->t_max = DM8009P_T_MAX;
    data->position_pid = pid_config->position_pid;
    data->velocity_pid = pid_config->velocity_pid;
    motor->motor_id = data->master_id;
    motor->motor_can_handle = can_handle;
    pid_reset(&data->position_pid);
    pid_reset(&data->velocity_pid);
    (void)motor_id;
    (void)para_num;
}

void dm8009p_feedback_calculate(const struct motor_device *motor, const uint8_t frame[8])
{
    dm8009p_data_t *data;
    uint16_t position;
    uint16_t velocity;
    uint16_t torque;
    if (motor == NULL || motor->motor_data == NULL || frame == NULL) { return; }
    data = motor->motor_data;
    data->can_id = frame[0] & 0x0FU;
    data->error = frame[0] >> 4;
    data->enabled = (data->error == 0x1U) ? 1U : 0U;
    position = (uint16_t)(((uint16_t)frame[1] << 8) | frame[2]);
    velocity = (uint16_t)(((uint16_t)frame[3] << 4) | (frame[4] >> 4));
    torque = (uint16_t)(((uint16_t)(frame[4] & 0x0FU) << 8) | frame[5]);
    data->position_rad = uint_to_float(position, -data->p_max, data->p_max, 16U);
    data->velocity_rad_s = uint_to_float(velocity, -data->v_max, data->v_max, 12U);
    data->torque_nm = uint_to_float(torque, -data->t_max, data->t_max, 12U);
    data->mos_temperature = frame[6];
    data->rotor_temperature = frame[7];
}

void dm8009p_send_ctrl_cmd(struct motor_device *motor)
{
    dm8009p_data_t *data;
    uint16_t position;
    uint16_t velocity;
    uint16_t torque;
    uint8_t frame[8];
    if (motor == NULL || motor->motor_data == NULL) { return; }
    data = motor->motor_data;
    if (data->enable_requested == 0U || data->enabled == 0U) { return; }
    position = float_to_uint(0.0f, -data->p_max, data->p_max, 16U);
    velocity = float_to_uint(0.0f, -data->v_max, data->v_max, 12U);
    torque = float_to_uint(clamp_float(data->output_torque_nm, -DM8009P_TORQUE_LIMIT_NM, DM8009P_TORQUE_LIMIT_NM), -data->t_max, data->t_max, 12U);
    frame[0] = (uint8_t)(position >> 8);
    frame[1] = (uint8_t)position;
    frame[2] = (uint8_t)(velocity >> 4);
    frame[3] = (uint8_t)(velocity << 4);
    frame[4] = 0U;
    frame[5] = 0U;
    frame[6] = (uint8_t)(torque >> 8);
    frame[7] = (uint8_t)torque;
    motor_send_standard(motor->motor_can_handle, data->command_id, frame);
}

void dm8009p_enable(struct motor_device *motor)
{
    dm8009p_data_t *data;
    if (motor == NULL || motor->motor_data == NULL) { return; }
    data = motor->motor_data;
    data->enable_requested = 1U;
    data->enabled = 0U;
    data->hold_position_pending = motor_is_online(motor) ? 0U : 1U;
    if (data->hold_position_pending == 0U) { data->target_position_rad = data->position_rad; }
    data->target_velocity_rad_s = 0.0f;
    data->target_acceleration_rad_s2 = 0.0f;
    data->output_torque_nm = 0.0f;
    data->trace.active = 0U;
    pid_reset(&data->position_pid);
    pid_reset(&data->velocity_pid);
    data->last_update_tick = 0U;
    dm8009p_send_special(motor, 0xFBU);
    dm8009p_send_special(motor, 0xFCU);
    data->last_clear_cmd_tick = HAL_GetTick();
    data->last_enable_cmd_tick = data->last_clear_cmd_tick;
}

void dm8009p_disable(struct motor_device *motor)
{
    dm8009p_data_t *data;
    if (motor == NULL || motor->motor_data == NULL) { return; }
    data = motor->motor_data;
    data->enable_requested = 0U;
    data->enabled = 0U;
    data->hold_position_pending = 0U;
    data->target_position_rad = data->position_rad;
    data->target_velocity_rad_s = 0.0f;
    data->target_acceleration_rad_s2 = 0.0f;
    data->output_torque_nm = 0.0f;
    data->trace.active = 0U;
    pid_reset(&data->position_pid);
    pid_reset(&data->velocity_pid);
    data->last_update_tick = 0U;
    dm8009p_send_special(motor, 0xFDU);
}

void dm8009p_update(struct motor_device *motor)
{
    dm8009p_data_t *data;
    float position_error;
    float position_pid_out_rad_s;
    float velocity_target_rad_s;
    float output_torque_nm;
    float acc_torque_nm;
    float gravity_torque_nm;
    float friction_torque_nm;
    float feedforward_torque_nm;
    float dt;
    uint32_t now;
    if (motor == NULL || motor->motor_data == NULL) { return; }
    data = motor->motor_data;
    if (data->enable_requested == 0U) { return; }
    if (motor->last_rx_tick == 0U || (HAL_GetTick() - motor->last_rx_tick) > MOTOR_OFFLINE_MS) {
        data->enabled = 0U;
        data->output_torque_nm = 0.0f;
        data->trace.active = 0U;
        pid_reset(&data->position_pid);
        pid_reset(&data->velocity_pid);
        data->last_update_tick = 0U;
        return;
    }
    now = HAL_GetTick();
    if (data->hold_position_pending != 0U) {
        // 防跳变
        data->target_position_rad = data->position_rad;
        data->target_velocity_rad_s = 0.0f;
        data->target_acceleration_rad_s2 = 0.0f;
        data->output_torque_nm = 0.0f;
        pid_reset(&data->position_pid);
        pid_reset(&data->velocity_pid);
        data->last_update_tick = 0U;
        data->hold_position_pending = 0U;
        return;
    }
    if (data->error >= 0x8U && data->error <= 0xEU) {
        data->output_torque_nm = 0.0f;
        data->trace.active = 0U;
        if ((uint32_t)(now - data->last_clear_cmd_tick) >= 50U) { dm8009p_send_special(motor, 0xFBU); data->last_clear_cmd_tick = now; }
        return;
    }
    if (data->error != 0x1U) {
        data->output_torque_nm = 0.0f;
        data->trace.active = 0U;
        if ((uint32_t)(now - data->last_enable_cmd_tick) >= 20U) { dm8009p_send_special(motor, 0xFCU); data->last_enable_cmd_tick = now; }
        return;
    }
    if (data->last_update_tick == 0U) { dt = MOTOR_CONTROL_S; }
    else { dt = clamp_float((float)(now - data->last_update_tick) * 0.001f, 0.0001f, 0.020f); }
    data->last_update_tick = now;
    position_error = wrap_to_pi(data->target_position_rad - data->position_rad);
    position_pid_out_rad_s = pid_update(&data->position_pid, position_error, 0.0f, dt);
    velocity_target_rad_s = data->target_velocity_rad_s + position_pid_out_rad_s;
    velocity_target_rad_s = clamp_float(velocity_target_rad_s, -DM8009P_V_MAX, DM8009P_V_MAX);
    output_torque_nm = pid_update(&data->velocity_pid, velocity_target_rad_s, data->velocity_rad_s, dt);
    acc_torque_nm = data->target_acceleration_rad_s2 * data->rotational_inertia_kg_m2;
    gravity_torque_nm = 0.0f;
    friction_torque_nm = data->friction_torque;
    feedforward_torque_nm = acc_torque_nm + gravity_torque_nm + friction_torque_nm;
    output_torque_nm += feedforward_torque_nm;
    data->output_torque_nm = clamp_float(output_torque_nm, -DM8009P_TORQUE_LIMIT_NM, DM8009P_TORQUE_LIMIT_NM);
    dm8009p_send_ctrl_cmd(motor);
}

static void dm8009p_trace_set_target(const struct motor_device *motor, float target_position_rad, float target_velocity_rad_s, float target_acceleration_rad_s2)
{
    dm8009p_data_t *data;
    if (motor == NULL || motor->motor_data == NULL) { return; }
    data = motor->motor_data;
    data->target_position_rad = target_position_rad;
    data->target_velocity_rad_s = clamp_float(target_velocity_rad_s, -DM8009P_V_MAX, DM8009P_V_MAX);
    data->target_acceleration_rad_s2 = target_acceleration_rad_s2;
}

static float dm8009p_trace_resolve_duration(float delta_position_rad, float requested_duration_s)
{
    float distance_rad = abs_float(delta_position_rad);
    float minimum_velocity_duration_s = MOTOR_QUINTIC_PEAK_VELOCITY_FACTOR * distance_rad / DM8009P_V_MAX;
    float minimum_acceleration_duration_s = motor_sqrt_float(MOTOR_QUINTIC_PEAK_ACCELERATION_FACTOR * distance_rad / DM8009P_TRACE_MAX_ACCELERATION_RAD_S2);
    float actual_duration_s = requested_duration_s;
    if (actual_duration_s < minimum_velocity_duration_s) { actual_duration_s = minimum_velocity_duration_s; }
    if (actual_duration_s < minimum_acceleration_duration_s) { actual_duration_s = minimum_acceleration_duration_s; }
    if (actual_duration_s < MOTOR_CONTROL_S) { actual_duration_s = MOTOR_CONTROL_S; }
    return actual_duration_s;
}

void dm8009p_set_trace(const struct motor_device *motor, float target_position_rad, float duration_s)
{
    dm8009p_data_t *data;
    float delta_position_rad;
    float position_epsilon_rad;
    if (motor == NULL || motor->motor_data == NULL) { return; }
    data = motor->motor_data;
    if (!motor_is_online(motor) || data->enabled == 0U || duration_s <= 0.0f) { return; }
    data->trace.start_position_rad = data->position_rad;
    delta_position_rad = wrap_to_pi(target_position_rad - data->trace.start_position_rad);
    data->trace.delta_position_rad = delta_position_rad;
    data->trace.end_position_rad = data->trace.start_position_rad + delta_position_rad;
    data->trace.duration_s = dm8009p_trace_resolve_duration(delta_position_rad, duration_s);
    data->trace.position_ref_rad = data->trace.start_position_rad;
    data->trace.velocity_ref_rad_s = 0.0f;
    data->trace.acceleration_ref_rad_s2 = 0.0f;
    data->trace.start_tick = HAL_GetTick();
    position_epsilon_rad = 2.0f * data->p_max / 65535.0f;
    if (delta_position_rad > -position_epsilon_rad && delta_position_rad < position_epsilon_rad) {
        data->trace.active = 0U;
        dm8009p_trace_set_target(motor, data->trace.end_position_rad, 0.0f, 0.0f);
        return;
    }
    data->trace.active = 1U;
    dm8009p_trace_set_target(motor, data->trace.start_position_rad, 0.0f, 0.0f);
}

void dm8009p_trace_update(struct motor_device *motor)
{
    dm8009p_data_t *data;
    motor_trace_t *trace;
    float elapsed_s;
    float normalized_time;
    float normalized_time_2;
    float normalized_time_3;
    float normalized_time_4;
    float normalized_time_5;
    float position_scale;
    float velocity_scale;
    float acceleration_scale;
    if (motor == NULL || motor->motor_data == NULL) { return; }
    data = motor->motor_data;
    trace = &data->trace;
    if (trace->active == 0U) { return; }
    if (data->enabled == 0U || !motor_is_online(motor)) {
        trace->active = 0U;
        dm8009p_trace_set_target(motor, data->position_rad, 0.0f, 0.0f);
        return;
    }
    elapsed_s = (float)(HAL_GetTick() - trace->start_tick) * 0.001f;
    if (elapsed_s >= trace->duration_s) {
        trace->position_ref_rad = trace->end_position_rad;
        trace->velocity_ref_rad_s = 0.0f;
        trace->acceleration_ref_rad_s2 = 0.0f;
        trace->active = 0U;
        dm8009p_trace_set_target(motor, trace->end_position_rad, 0.0f, 0.0f);
        return;
    }
    normalized_time = clamp_float(elapsed_s / trace->duration_s, 0.0f, 1.0f);
    normalized_time_2 = normalized_time * normalized_time;
    normalized_time_3 = normalized_time_2 * normalized_time;
    normalized_time_4 = normalized_time_3 * normalized_time;
    normalized_time_5 = normalized_time_4 * normalized_time;
    position_scale = 10.0f * normalized_time_3 - 15.0f * normalized_time_4 + 6.0f * normalized_time_5;
    velocity_scale = 30.0f * normalized_time_2 - 60.0f * normalized_time_3 + 30.0f * normalized_time_4;
    acceleration_scale = 60.0f * normalized_time - 180.0f * normalized_time_2 + 120.0f * normalized_time_3;
    trace->position_ref_rad = trace->start_position_rad + trace->delta_position_rad * position_scale;
    trace->velocity_ref_rad_s = trace->delta_position_rad / trace->duration_s * velocity_scale;
    trace->acceleration_ref_rad_s2 = trace->delta_position_rad / (trace->duration_s * trace->duration_s) * acceleration_scale;
    dm8009p_trace_set_target(motor, trace->position_ref_rad, trace->velocity_ref_rad_s, trace->acceleration_ref_rad_s2);
}

void dm8009p_set_target(const struct motor_device *motor, int para_num, ...)
{
    dm8009p_data_t *data;
    float target_position_rad;
    float target_velocity_rad_s;
    float target_acceleration_rad_s2;
    va_list arguments;
    if (motor == NULL || motor->motor_data == NULL) { return; }
    data = motor->motor_data;
    target_position_rad = data->target_position_rad;
    target_velocity_rad_s = data->target_velocity_rad_s;
    target_acceleration_rad_s2 = data->target_acceleration_rad_s2;
    va_start(arguments, para_num);
    if (para_num >= 1) { target_position_rad = (float)va_arg(arguments, double); target_velocity_rad_s = 0.0f; target_acceleration_rad_s2 = 0.0f; }
    if (para_num >= 2) { target_velocity_rad_s = (float)va_arg(arguments, double); target_acceleration_rad_s2 = 0.0f; }
    if (para_num >= 3) { target_acceleration_rad_s2 = (float)va_arg(arguments, double); }
    va_end(arguments);
    data->trace.active = 0U;
    dm8009p_trace_set_target(motor, target_position_rad, target_velocity_rad_s, target_acceleration_rad_s2);
}

void dm8009p_get_status(const struct motor_device *motor, const char *which, void *value)
{
    dm8009p_data_t *data;
    if (motor == NULL || motor->motor_data == NULL || which == NULL || value == NULL) { return; }
    data = motor->motor_data;
    if (strcmp(which, "POS") == 0) { *(float *)value = data->position_rad; }
    else if (strcmp(which, "VEL") == 0) { *(float *)value = data->velocity_rad_s; }
    else if (strcmp(which, "TORQUE") == 0) { *(float *)value = data->torque_nm; }
    else if (strcmp(which, "TEMP") == 0) { *(uint8_t *)value = data->rotor_temperature; }
    else if (strcmp(which, "MOS_TEMP") == 0) { *(uint8_t *)value = data->mos_temperature; }
    else if (strcmp(which, "ERR") == 0) { *(uint8_t *)value = data->error; }
    else if (strcmp(which, "ID") == 0) { *(uint8_t *)value = data->can_id; }
    else if (strcmp(which, "TARGET_POS") == 0) { *(float *)value = data->target_position_rad; }
    else if (strcmp(which, "TARGET_VEL") == 0) { *(float *)value = data->target_velocity_rad_s; }
    else if (strcmp(which, "TARGET_ACC") == 0) { *(float *)value = data->target_acceleration_rad_s2; }
}

void dm8009p_set_para(const struct motor_device *motor, const char *which, const void *value)
{
    dm8009p_data_t *data;
    if (motor == NULL || motor->motor_data == NULL || which == NULL || value == NULL) { return; }
    data = motor->motor_data;
    if (strcmp(which, "POS_KP") == 0) { data->position_pid.kp = *(const float *)value; }
    else if (strcmp(which, "POS_KI") == 0) { data->position_pid.ki = *(const float *)value; }
    else if (strcmp(which, "POS_KD") == 0) { data->position_pid.kd = *(const float *)value; }
    else if (strcmp(which, "VEL_KP") == 0) { data->velocity_pid.kp = *(const float *)value; }
    else if (strcmp(which, "VEL_KI") == 0) { data->velocity_pid.ki = *(const float *)value; }
    else if (strcmp(which, "VEL_KD") == 0) { data->velocity_pid.kd = *(const float *)value; }
}

/* --------------------------- 信号发送与接收部分 --------------------------- */
/* Build every byte of 0x1FF from the registered GM6020s on this CAN bus.
 * This prevents one motor's update from zeroing the other three control slots. */
// 将挂载在传入can句柄上的电机的控制量打包下发
static void gm6020_send_group(FDCAN_HandleTypeDef *can_handle)
{
    uint8_t frame[8] = {0};
    uint32_t index;

    if (can_handle == NULL) {
        return;
    }
    for (index = 0U; index < Motor_Get_Count(); ++index) {
        struct motor_device *motor = motor_instance_get(index);
        gm6020_data_t *data;
        int16_t output;
        uint8_t offset;

        if (motor->motor_can_handle != can_handle || motor->send_ctrl_cmd != gm6020_send_ctrl_cmd || motor->motor_data == NULL) {
            continue;
        }
        data = motor->motor_data;
        if (data->control_slot == 0U || data->control_slot > 4U) {
            continue;
        }
        output = (data->enabled != 0U) ? (int16_t)data->output_current_code : 0;
        offset = (uint8_t)((data->control_slot - 1U) * 2U);
        frame[offset] = (uint8_t)((uint16_t)output >> 8);
        frame[offset + 1U] = (uint8_t)output;
    }
    motor_send_standard(can_handle, GM6020_GROUP_ID, frame);
}

// 将挂载在传入can句柄上的3508电机控制量打包下发
static void m3508_send_group(FDCAN_HandleTypeDef *can_handle)
{
    uint8_t frame[8] = {0};
    uint32_t index;
    if (can_handle == NULL) { return; }
    for (index = 0U; index < Motor_Get_Count(); ++index) {
        struct motor_device *motor = motor_instance_get(index);
        m3508_data_t *data;
        int16_t output;
        uint8_t offset;
        if (motor->motor_can_handle != can_handle || motor->send_ctrl_cmd != m3508_send_ctrl_cmd || motor->motor_data == NULL) { continue; }
        data = motor->motor_data;
        if (data->control_slot == 0U || data->control_slot > 4U) { continue; }
        output = (data->enabled != 0U && data->error == 0U && motor_is_online(motor)) ? (int16_t)data->output_current_code : 0;
        offset = (uint8_t)((data->control_slot - 1U) * 2U);
        frame[offset] = (uint8_t)((uint16_t)output >> 8);
        frame[offset + 1U] = (uint8_t)output;
    }
    motor_send_standard(can_handle, M3508_GROUP_ID, frame);
}

//得到电机句柄
struct motor_device *motor_get_device(const char *name)
{
    uint32_t index;
    if (name == NULL) { return NULL; }
    for (index = 0U; index < Motor_Get_Count(); ++index) {
        struct motor_device *motor = motor_instance_get(index);

        if (strcmp(name, motor->motor_name) == 0) {
            return motor;
        }
    }
    return NULL;
}

// 得到注册电机数量
uint32_t Motor_Get_Count(void)
{
    return motor_instance_count();
}

bool motor_is_online(const struct motor_device *motor)
{
    if (motor == NULL || motor->last_rx_tick == 0U) {
        return false;
    }
    return (HAL_GetTick() - motor->last_rx_tick) <= MOTOR_OFFLINE_MS;
}

void Motor_All_Trace_Update(void)
{
    uint32_t index;

    for (index = 0U; index < Motor_Get_Count(); ++index) {
        struct motor_device *motor = motor_instance_get(index);

        if (motor->trace_update != NULL) {
            motor->trace_update(motor);
        }
    }
}

void Motor_All_Update(void)
{
    uint32_t index;
    for (index = 0U; index < Motor_Get_Count(); ++index) {
        struct motor_device *motor = motor_instance_get(index);

        motor->update(motor);
    }
    Motor_Send_All_Control();
}

void Motor_Send_All_Control(void)
{
    FDCAN_HandleTypeDef *gm6020_sent_handles[4] = {0};
    FDCAN_HandleTypeDef *m3508_sent_handles[4] = {0};
    uint32_t gm6020_sent_count = 0U;
    uint32_t m3508_sent_count = 0U;
    uint32_t index;
    for (index = 0U; index < Motor_Get_Count(); ++index) {
        struct motor_device *motor = motor_instance_get(index);
        uint32_t sent_index;
        if (motor->motor_can_handle == NULL) { continue; }
        if (motor->send_ctrl_cmd == gm6020_send_ctrl_cmd) {
            for (sent_index = 0U; sent_index < gm6020_sent_count; ++sent_index) {
                if (gm6020_sent_handles[sent_index] == motor->motor_can_handle) { break; }
            }
            if (sent_index == gm6020_sent_count && gm6020_sent_count < (sizeof(gm6020_sent_handles) / sizeof(gm6020_sent_handles[0]))) {
                gm6020_sent_handles[gm6020_sent_count++] = motor->motor_can_handle;
                gm6020_send_group(motor->motor_can_handle);
            }
        } else if (motor->send_ctrl_cmd == m3508_send_ctrl_cmd) {
            for (sent_index = 0U; sent_index < m3508_sent_count; ++sent_index) {
                if (m3508_sent_handles[sent_index] == motor->motor_can_handle) { break; }
            }
            if (sent_index == m3508_sent_count && m3508_sent_count < (sizeof(m3508_sent_handles) / sizeof(m3508_sent_handles[0]))) {
                m3508_sent_handles[m3508_sent_count++] = motor->motor_can_handle;
                m3508_send_group(motor->motor_can_handle);
            }
        }
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *handle, uint32_t interrupts)
{
    FDCAN_RxHeaderTypeDef header;
    uint8_t frame[8];
    uint32_t index;



    if (handle == &hfdcan1) {
        ++g_can1_irq_count;
    }

    if ((interrupts & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U) { return; }
    while (HAL_FDCAN_GetRxFifoFillLevel(handle, FDCAN_RX_FIFO0) > 0U) {
        if (HAL_FDCAN_GetRxMessage(handle, FDCAN_RX_FIFO0, &header, frame) != HAL_OK) { break; }
        if (handle == &hfdcan1) {
            ++g_can1_frame_count;
            g_can1_last_id = header.Identifier;
            g_can1_last_dlc = header.DataLength;
        }
        if (header.IdType != FDCAN_STANDARD_ID || header.DataLength != FDCAN_DLC_BYTES_8) { continue; }
        for (index = 0U; index < Motor_Get_Count(); ++index) {
            struct motor_device *motor = motor_instance_get(index);
            if (motor->motor_can_handle == handle && motor->motor_id == header.Identifier) {
                motor->feedback_calculate(motor, frame);
                motor->last_rx_tick = HAL_GetTick();
                break;
            }
        }
    }
}
