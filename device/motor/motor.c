/**
 * @file motor.c
 * @brief GM6020, M3508 and DM4310 static motor-device implementations.
 */

#include "motor.h"

#include <stdarg.h>
#include <string.h>

#include "../../bsp/can/can.h"
#include "../../dsp/pid/pid.h"
#include "../../dsp/math.h"

#define MOTOR_CONTROL_DT_DEFAULT_S       0.001f
#define MOTOR_OFFLINE_TIMEOUT_MS         100U

#define GM6020_CONTROL_GROUP_ID          0x1FEU
#define GM6020_ENCODER_RESOLUTION        8192.0f
#define GM6020_OUTPUT_LIMIT              16384.0f
#define GM6020_SPEED_LIMIT_RPM           320.0f

#define GM6020_TRACE_POSITION_EPSILON_RAD (TWO_PI / GM6020_ENCODER_RESOLUTION)

#define MOTOR_QUINTIC_PEAK_VELOCITY_FACTOR     1.875f
#define MOTOR_QUINTIC_PEAK_ACCELERATION_FACTOR 5.773502691896258f
#define GM6020_TRACE_MAX_ACCELERATION_RAD_S2    300.0f

#define M3508_CONTROL_GROUP_ID           0x200U
#define M3508_ENCODER_RESOLUTION         8192.0f
#define M3508_OUTPUT_LIMIT               16384.0f
#define M3508_SPEED_LIMIT_RPM            9085.0f
#define M3508_MAX_TORQUE_CURRENT_A       20.0f
#define M3508_TRACE_POSITION_EPSILON_RAD (TWO_PI / M3508_ENCODER_RESOLUTION)
#define M3508_TRACE_MAX_ACCELERATION_RAD_S2 300.0f

#define DM4310_COMMAND_ID                 CAN_J4310_PITCH_ID
#define DM4310_P_MAX                      12.5f
#define DM4310_V_MAX                      3.0f
#define DM4310_T_MAX                      10.0f
#define DM4310_ERR_DISABLED                0x0U
#define DM4310_ERR_ENABLED                 0x1U
#define DM4310_ERR_FAULT_MIN               0x8U
#define DM4310_ERR_FAULT_MAX               0xEU
#define DM4310_CLEAR_RETRY_MS              50U
#define DM4310_ENABLE_RETRY_MS             20U
#define DM4310_TRACE_MAX_ACCELERATION_RAD_S2 30.0f

volatile uint32_t g_can1_irq_count = 0U;
volatile uint32_t g_can1_frame_count = 0U;
volatile uint32_t g_can1_last_id = 0U;
volatile uint32_t g_can1_last_dlc = 0U;
volatile uint32_t g_can1_tx_ok_count = 0U;
volatile uint32_t g_can1_tx_fail_count = 0U;

typedef struct {
    pid_t position_pid;
    pid_t velocity_pid;
} gm6020_pid_config_t;

typedef struct {
    pid_t position_pid;
    pid_t velocity_pid;
} m3508_pid_config_t;

typedef struct {
    float start_position_rad;
    float end_position_rad;
    float delta_position_rad;
    float duration_s;
    float position_ref_rad;
    float velocity_ref_rad_s;
    float acceleration_ref_rad_s2;
    uint32_t start_tick;
    uint8_t active;
} motor_trace_t;

/* ------------------------------ 电机结构体封装 ---------------------------------- */
typedef struct {
    const gm6020_pid_config_t *pid_config;
    pid_t position_pid;
    pid_t velocity_pid;
    float target_position_rad;
    float target_velocity_rpm;  // 速度前馈
    float target_acceleration_rad_s2; // 加速度前馈
    float rotational_inertia_kg_m2;
    float friction_torque;
    float K_t_Nm_A;
    motor_trace_t trace;
    float position_rad;
    float velocity_rpm;
    float torque_current;
    float output_current_code;
    uint16_t encoder;
    uint16_t last_encoder;
    int16_t speed_rpm;
    int16_t current;
    uint8_t temperature;
    uint8_t encoder_initialized;
    uint8_t enabled;
    uint32_t last_update_tick;
    uint8_t control_slot;               /* GM6020 ID: 1..4 in group 0x1FF. */
    float position_continuous_rad;
} gm6020_data_t;

typedef struct {
    const m3508_pid_config_t *pid_config;
    pid_t position_pid;
    pid_t velocity_pid;
    float target_position_rad;
    float target_velocity_rpm;
    float target_acceleration_rad_s2;
    float rotational_inertia_kg_m2;
    float friction_torque;
    float K_t_Nm_A;
    motor_trace_t trace;
    float position_rad;
    float velocity_rpm;
    float torque_current;
    float output_current_code;
    uint16_t encoder;
    uint16_t last_encoder;
    int16_t speed_rpm;
    int16_t current;
    uint8_t temperature;
    uint8_t error;
    uint8_t encoder_initialized;
    uint8_t enabled;
    uint32_t last_update_tick;
    uint8_t control_slot;
    float position_continuous_rad;
} m3508_data_t;

typedef struct {
    float target_position_rad;
    float target_velocity_rad_s;
    float target_torque_nm;
    motor_trace_t trace;
    float kp;
    float kd;
    float position_rad;
    float velocity_rad_s;
    float torque_nm;
    float p_max;
    float v_max;
    float t_max;
    uint8_t error;
    uint8_t mos_temperature;
    uint8_t rotor_temperature;
    uint8_t enable_requested; /* Software request, latched until explicit disable. */
    uint8_t enabled;          /* Confirmed only by a feedback frame reporting enabled. */
    uint32_t last_clear_cmd_tick;
    uint32_t last_enable_cmd_tick;
} dm4310_data_t;

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

static float gm6020_wrap_to_pi(float angle_rad)
{
    while (angle_rad > PI) {
        angle_rad -= TWO_PI;
    }
    while (angle_rad < -PI) {
        angle_rad += TWO_PI;
    }
    return angle_rad;
}

static float m3508_wrap_to_pi(float angle_rad)
{
    while (angle_rad > PI) { angle_rad -= TWO_PI; }
    while (angle_rad < -PI) { angle_rad += TWO_PI; }
    return angle_rad;
}

// dm4310协议要求浮点数转化为uint16
static uint16_t float_to_uint(float value, float minimum, float maximum, uint16_t bits)
{
    float scaled;
    uint32_t maximum_integer = (1UL << bits) - 1UL;

    value = clamp_float(value, minimum, maximum);
    scaled = (value - minimum) * (float)maximum_integer / (maximum - minimum);
    return (uint16_t)scaled;
}

static void gm6020_send_group(FDCAN_HandleTypeDef *can_handle);
static void m3508_send_group(FDCAN_HandleTypeDef *can_handle);

/* ------------------------------ GM6020 ---------------------------------- */
// 初始化6020对象 传入要实例化电机对象，反馈id，can句柄
// 作用为清空6020初始状态，绑定can和id，确定组帧槽位，并加载pid参数
static void gm6020_init(struct motor_device *motor, uint32_t motor_id,
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
static void gm6020_feedback_calculate(const struct motor_device *motor,
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
            (float)encoder * TWO_PI / GM6020_ENCODER_RESOLUTION;
        data->encoder_initialized = 1U;
    } else {
        delta_encoder = (int32_t)encoder - (int32_t)data->last_encoder;

        if (delta_encoder > 4096) {
            delta_encoder -= 8192;
        } else if (delta_encoder < -4096) {
            delta_encoder += 8192;
        }

        data->position_continuous_rad +=
            (float)delta_encoder * TWO_PI / GM6020_ENCODER_RESOLUTION;

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
static void gm6020_send_ctrl_cmd(struct motor_device *motor)
{
    if (motor == NULL || motor->motor_data == NULL) {
        return;
    }
    /* A GM6020 control command is always a complete shared 0x1FF group frame. */
    gm6020_send_group(motor->motor_can_handle);
}

// 使能6020
static void gm6020_enable(struct motor_device *motor)
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
static void gm6020_disable(struct motor_device *motor)
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
static void gm6020_update(struct motor_device *motor)
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

    if (data->enabled == 0U || motor->last_rx_tick == 0U || (HAL_GetTick() - motor->last_rx_tick) > MOTOR_OFFLINE_TIMEOUT_MS) {
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
        dt = MOTOR_CONTROL_DT_DEFAULT_S;
    } else {
        dt = (float)(now - data->last_update_tick) * 0.001f;
        dt = clamp_float(dt, 0.0001f, 0.020f);
    }

    data->last_update_tick = now;

    position_error = gm6020_wrap_to_pi(data->target_position_rad - data->position_rad);

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
    if (actual_duration_s < MOTOR_CONTROL_DT_DEFAULT_S) {
        actual_duration_s = MOTOR_CONTROL_DT_DEFAULT_S;
    }

    return actual_duration_s;
}

// 初始化生成轨迹参数
static void gm6020_set_trace(const struct motor_device *motor,
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
    delta_position_rad = gm6020_wrap_to_pi(target_position_rad - data->trace.start_position_rad);
    data->trace.delta_position_rad = delta_position_rad;
    data->trace.end_position_rad = data->trace.start_position_rad + delta_position_rad;
    data->trace.duration_s = gm6020_trace_resolve_duration(delta_position_rad, duration_s);
    data->trace.position_ref_rad = data->trace.start_position_rad;
    data->trace.velocity_ref_rad_s = 0.0f;
    data->trace.acceleration_ref_rad_s2 = 0.0f;
    data->trace.start_tick = HAL_GetTick();

    if (delta_position_rad > -GM6020_TRACE_POSITION_EPSILON_RAD && delta_position_rad < GM6020_TRACE_POSITION_EPSILON_RAD) {
        data->trace.active = 0U;
        gm6020_trace_set_target(motor, data->trace.end_position_rad, 0.0f,0.0f);
        return;
    }

    data->trace.active = 1U;
    gm6020_trace_set_target(motor, data->trace.start_position_rad, 0.0f,0.0f);
}

// 依据strace进度更新strace
static void gm6020_trace_update(struct motor_device *motor)
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
static void gm6020_set_target(const struct motor_device *motor, int para_num, ...)
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
static void gm6020_get_status(const struct motor_device *motor, const char *which,
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
static void gm6020_set_para(const struct motor_device *motor, const char *which,
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
static void m3508_init(struct motor_device *motor, uint32_t motor_id,
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
static void m3508_feedback_calculate(const struct motor_device *motor,
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
        data->position_continuous_rad = (float)encoder * TWO_PI / M3508_ENCODER_RESOLUTION;
        data->encoder_initialized = 1U;
    } else {
        delta_encoder = (int32_t)encoder - (int32_t)data->last_encoder;
        if (delta_encoder > 4096) { delta_encoder -= 8192; }
        else if (delta_encoder < -4096) { delta_encoder += 8192; }
        data->position_continuous_rad += (float)delta_encoder * TWO_PI / M3508_ENCODER_RESOLUTION;
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
static void m3508_send_ctrl_cmd(struct motor_device *motor)
{
    if (motor == NULL || motor->motor_data == NULL) { return; }
    m3508_send_group(motor->motor_can_handle);
}

// 使能3508
static void m3508_enable(struct motor_device *motor)
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
static void m3508_disable(struct motor_device *motor)
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
static void m3508_update(struct motor_device *motor)
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
    if (data->enabled == 0U || data->error != 0U || motor->last_rx_tick == 0U || (HAL_GetTick() - motor->last_rx_tick) > MOTOR_OFFLINE_TIMEOUT_MS) {
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
    if (data->last_update_tick == 0U) { dt = MOTOR_CONTROL_DT_DEFAULT_S; }
    else {
        dt = (float)(now - data->last_update_tick) * 0.001f;
        dt = clamp_float(dt, 0.0001f, 0.020f);
    }
    data->last_update_tick = now;
    position_error = m3508_wrap_to_pi(data->target_position_rad - data->position_rad);
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
    if (actual_duration_s < MOTOR_CONTROL_DT_DEFAULT_S) { actual_duration_s = MOTOR_CONTROL_DT_DEFAULT_S; }
    return actual_duration_s;
}

// 初始化生成轨迹参数
static void m3508_set_trace(const struct motor_device *motor,
                            float target_position_rad,
                            float duration_s)
{
    m3508_data_t *data;
    float delta_position_rad;
    if (motor == NULL || motor->motor_data == NULL) { return; }
    data = motor->motor_data;
    if (!motor_is_online(motor) || data->enabled == 0U || duration_s <= 0.0f) { return; }
    data->trace.start_position_rad = data->position_rad;
    delta_position_rad = m3508_wrap_to_pi(target_position_rad - data->trace.start_position_rad);
    data->trace.delta_position_rad = delta_position_rad;
    data->trace.end_position_rad = data->trace.start_position_rad + delta_position_rad;
    data->trace.duration_s = m3508_trace_resolve_duration(delta_position_rad, duration_s);
    data->trace.position_ref_rad = data->trace.start_position_rad;
    data->trace.velocity_ref_rad_s = 0.0f;
    data->trace.acceleration_ref_rad_s2 = 0.0f;
    data->trace.start_tick = HAL_GetTick();
    if (delta_position_rad > -M3508_TRACE_POSITION_EPSILON_RAD && delta_position_rad < M3508_TRACE_POSITION_EPSILON_RAD) {
        data->trace.active = 0U;
        m3508_trace_set_target(motor, data->trace.end_position_rad, 0.0f, 0.0f);
        return;
    }
    data->trace.active = 1U;
    m3508_trace_set_target(motor, data->trace.start_position_rad, 0.0f, 0.0f);
}

// 依据strace进度更新strace
static void m3508_trace_update(struct motor_device *motor)
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
static void m3508_set_target(const struct motor_device *motor, int para_num, ...)
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
static void m3508_get_status(const struct motor_device *motor, const char *which,
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
static void m3508_set_para(const struct motor_device *motor, const char *which,
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

/* ------------------------------ DM4310 ---------------------------------- */

static void dm4310_send_special(struct motor_device *motor, uint8_t command)
{
    uint8_t frame[8] = {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, command};
    dm4310_data_t *data;

    if (motor == NULL || motor->motor_data == NULL) { return; }
    data = motor->motor_data;
    motor_send_standard(motor->motor_can_handle, DM4310_COMMAND_ID, frame);
    (void)data;
}

static void dm4310_init(struct motor_device *motor, uint32_t motor_id,
                        FDCAN_HandleTypeDef *can_handle, int para_num, ...)
{
    dm4310_data_t *data;
    va_list arguments;

    if (motor == NULL || motor->motor_data == NULL || can_handle == NULL) { return; }
    data = motor->motor_data;
    memset(data, 0, sizeof(*data));
    motor->motor_id = motor_id;
    motor->motor_can_handle = can_handle;
    data->p_max = DM4310_P_MAX;
    data->v_max = DM4310_V_MAX;
    data->t_max = DM4310_T_MAX;
    data->kp = 20.0f;
    data->kd = 1.0f;

    va_start(arguments, para_num);
    if (para_num >= 1) { data->kp = (float)va_arg(arguments, double); }
    if (para_num >= 2) { data->kd = (float)va_arg(arguments, double); }
    if (para_num >= 3) { data->p_max = (float)va_arg(arguments, double); }
    if (para_num >= 4) { data->v_max = (float)va_arg(arguments, double); }
    if (para_num >= 5) { data->t_max = (float)va_arg(arguments, double); }
    va_end(arguments);
}

static void dm4310_feedback_calculate(const struct motor_device *motor,
                                      const uint8_t frame[8])
{
    dm4310_data_t *data;
    uint16_t position;
    uint16_t velocity;
    uint16_t torque;

    if (motor == NULL || motor->motor_data == NULL || frame == NULL) { return; }
    data = motor->motor_data;
    data->error = frame[0] >> 4;
    data->enabled = (data->error == DM4310_ERR_ENABLED) ? 1U : 0U;
    position = (uint16_t)(((uint16_t)frame[1] << 8) | frame[2]);
    velocity = (uint16_t)(((uint16_t)frame[3] << 4) | (frame[4] >> 4));
    torque = (uint16_t)(((uint16_t)(frame[4] & 0x0FU) << 8) | frame[5]);
    data->position_rad = ((float)position * 2.0f * data->p_max / 65535.0f) - data->p_max;
    data->velocity_rad_s = ((float)velocity * 2.0f * data->v_max / 4095.0f) - data->v_max;
    data->torque_nm = ((float)torque * 2.0f * data->t_max / 4095.0f) - data->t_max;
    data->mos_temperature = frame[6];
    data->rotor_temperature = frame[7];
}

static void dm4310_send_ctrl_cmd(struct motor_device *motor)
{
    dm4310_data_t *data;
    uint16_t position;
    uint16_t velocity;
    uint16_t kp;
    uint16_t kd;
    uint16_t torque;
    uint8_t frame[8];

    if (motor == NULL || motor->motor_data == NULL) { return; }
    data = motor->motor_data;
    if (data->enable_requested == 0U || data->enabled == 0U) { return; }

    position = float_to_uint(data->target_position_rad, -data->p_max, data->p_max, 16U);
    velocity = float_to_uint(data->target_velocity_rad_s, -data->v_max, data->v_max, 12U);
    kp = float_to_uint(data->kp, 0.0f, 500.0f, 12U);
    kd = float_to_uint(data->kd, 0.0f, 5.0f, 12U);
    torque = float_to_uint(data->target_torque_nm, -data->t_max, data->t_max, 12U);

    frame[0] = (uint8_t)(position >> 8);
    frame[1] = (uint8_t)position;
    frame[2] = (uint8_t)(velocity >> 4);
    frame[3] = (uint8_t)((velocity << 4) | (kp >> 8));
    frame[4] = (uint8_t)kp;
    frame[5] = (uint8_t)(kd >> 4);
    frame[6] = (uint8_t)((kd << 4) | (torque >> 8));
    frame[7] = (uint8_t)torque;
    motor_send_standard(motor->motor_can_handle, DM4310_COMMAND_ID, frame);
}

static void dm4310_enable(struct motor_device *motor)
{
    dm4310_data_t *data;
    if (motor == NULL || motor->motor_data == NULL) { return; }
    data = motor->motor_data;
    data->enable_requested = 1U;
    data->enabled = 0U;
    data->target_position_rad = data->position_rad;
    data->target_velocity_rad_s = 0.0f;
    data->target_torque_nm = 0.0f;
    data->trace.active = 0U;
    dm4310_send_special(motor, 0xFBU);
    dm4310_send_special(motor, 0xFCU);
    data->last_clear_cmd_tick = HAL_GetTick();
    data->last_enable_cmd_tick = data->last_clear_cmd_tick;
}

static void dm4310_disable(struct motor_device *motor)
{
    dm4310_data_t *data;
    if (motor == NULL || motor->motor_data == NULL) { return; }
    data = motor->motor_data;
    data->enable_requested = 0U;
    data->enabled = 0U;
    data->target_position_rad = data->position_rad;
    data->target_velocity_rad_s = 0.0f;
    data->target_torque_nm = 0.0f;
    data->trace.active = 0U;
    dm4310_send_special(motor, 0xFDU);
}

static void dm4310_update(struct motor_device *motor)
{
    dm4310_data_t *data;
    uint32_t now;
    if (motor == NULL || motor->motor_data == NULL) { return; }
    data = motor->motor_data;
    if (motor->last_rx_tick == 0U || (HAL_GetTick() - motor->last_rx_tick) > MOTOR_OFFLINE_TIMEOUT_MS) {
        data->target_position_rad = data->position_rad;
        data->target_velocity_rad_s = 0.0f;
        data->target_torque_nm = 0.0f;
        data->trace.active = 0U;
        return;
    }
    if (data->enable_requested == 0U) {
        data->trace.active = 0U;
        return;
    }

    now = HAL_GetTick();
    if (data->error >= DM4310_ERR_FAULT_MIN && data->error <= DM4310_ERR_FAULT_MAX) {
        data->trace.active = 0U;
        if ((uint32_t)(now - data->last_clear_cmd_tick) >= DM4310_CLEAR_RETRY_MS) {
            dm4310_send_special(motor, 0xFBU);
            data->last_clear_cmd_tick = now;
        }
        return;
    }
    if (data->error != DM4310_ERR_ENABLED) {
        data->trace.active = 0U;
        if ((uint32_t)(now - data->last_enable_cmd_tick) >= DM4310_ENABLE_RETRY_MS) {
            dm4310_send_special(motor, 0xFCU);
            data->last_enable_cmd_tick = now;
        }
        return;
    }
    dm4310_send_ctrl_cmd(motor);
}

static void dm4310_trace_set_target(const struct motor_device *motor,
                                    float target_position_rad,
                                    float target_velocity_rad_s)
{
    dm4310_data_t *data;
    if (motor == NULL || motor->motor_data == NULL) { return; }
    data = motor->motor_data;
    data->target_position_rad = target_position_rad;
    data->target_velocity_rad_s = clamp_float(target_velocity_rad_s, -data->v_max, data->v_max);
    data->target_torque_nm = 0.0f;
}

static float dm4310_trace_resolve_duration(const dm4310_data_t *data, float delta_position_rad, float requested_duration_s)
{
    float distance_rad;
    float minimum_velocity_duration_s;
    float minimum_acceleration_duration_s;
    float actual_duration_s;
    if (data == NULL || data->v_max <= 0.0f) { return requested_duration_s; }
    distance_rad = abs_float(delta_position_rad);
    minimum_velocity_duration_s = MOTOR_QUINTIC_PEAK_VELOCITY_FACTOR * distance_rad / data->v_max;
    minimum_acceleration_duration_s = motor_sqrt_float(MOTOR_QUINTIC_PEAK_ACCELERATION_FACTOR * distance_rad / DM4310_TRACE_MAX_ACCELERATION_RAD_S2);
    actual_duration_s = requested_duration_s;
    if (actual_duration_s < minimum_velocity_duration_s) { actual_duration_s = minimum_velocity_duration_s; }
    if (actual_duration_s < minimum_acceleration_duration_s) { actual_duration_s = minimum_acceleration_duration_s; }
    if (actual_duration_s < MOTOR_CONTROL_DT_DEFAULT_S) { actual_duration_s = MOTOR_CONTROL_DT_DEFAULT_S; }
    return actual_duration_s;
}

static void dm4310_set_trace(const struct motor_device *motor,
                             float target_position_rad,
                             float duration_s)
{
    dm4310_data_t *data;
    float position_epsilon_rad;
    float end_position_rad;
    float delta_position_rad;
    if (motor == NULL || motor->motor_data == NULL) { return; }
    data = motor->motor_data;
    if (!motor_is_online(motor) || data->enabled == 0U || data->p_max <= 0.0f || data->v_max <= 0.0f || duration_s <= 0.0f) { return; }
    data->trace.start_position_rad = data->position_rad;
    end_position_rad = clamp_float(target_position_rad, -data->p_max, data->p_max);
    delta_position_rad = end_position_rad - data->trace.start_position_rad;
    data->trace.end_position_rad = end_position_rad;
    data->trace.delta_position_rad = delta_position_rad;
    data->trace.duration_s = dm4310_trace_resolve_duration(data, delta_position_rad, duration_s);
    data->trace.position_ref_rad = data->trace.start_position_rad;
    data->trace.velocity_ref_rad_s = 0.0f;
    data->trace.acceleration_ref_rad_s2 = 0.0f;
    data->trace.start_tick = HAL_GetTick();
    position_epsilon_rad = 2.0f * data->p_max / 65535.0f;
    if (delta_position_rad > -position_epsilon_rad && delta_position_rad < position_epsilon_rad) {
        data->trace.active = 0U;
        dm4310_trace_set_target(motor, data->trace.end_position_rad, 0.0f);
        return;
    }
    data->trace.active = 1U;
    dm4310_trace_set_target(motor, data->trace.start_position_rad, 0.0f);
}

static void dm4310_trace_update(struct motor_device *motor)
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
        dm4310_trace_set_target(motor, data->position_rad, 0.0f);
        return;
    }
    elapsed_s = (float)(HAL_GetTick() - trace->start_tick) * 0.001f;
    if (elapsed_s >= trace->duration_s) {
        trace->position_ref_rad = trace->end_position_rad;
        trace->velocity_ref_rad_s = 0.0f;
        trace->acceleration_ref_rad_s2 = 0.0f;
        trace->active = 0U;
        dm4310_trace_set_target(motor, trace->end_position_rad, 0.0f);
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
    dm4310_trace_set_target(motor, trace->position_ref_rad, trace->velocity_ref_rad_s);
}

static void dm4310_set_target(const struct motor_device *motor, int para_num, ...)
{
    dm4310_data_t *data;
    float target_position_rad;
    float target_velocity_rad_s;
    float target_torque_nm;
    va_list arguments;
    if (motor == NULL || motor->motor_data == NULL) { return; }
    data = motor->motor_data;
    target_position_rad = data->target_position_rad;
    target_velocity_rad_s = data->target_velocity_rad_s;
    target_torque_nm = data->target_torque_nm;
    va_start(arguments, para_num);
    if (para_num >= 1) { target_position_rad = (float)va_arg(arguments, double); }
    if (para_num >= 2) { target_velocity_rad_s = (float)va_arg(arguments, double); }
    if (para_num >= 3) { target_torque_nm = (float)va_arg(arguments, double); }
    va_end(arguments);
    data->trace.active = 0U;
    data->target_position_rad = target_position_rad;
    data->target_velocity_rad_s = target_velocity_rad_s;
    data->target_torque_nm = target_torque_nm;
}

static void dm4310_get_status(const struct motor_device *motor, const char *which,
                              void *value)
{
    dm4310_data_t *data;
    if (motor == NULL || motor->motor_data == NULL || which == NULL || value == NULL) { return; }
    data = motor->motor_data;
    if (strcmp(which, "POS") == 0) { *(float *)value = data->position_rad; }
    else if (strcmp(which, "VEL") == 0) { *(float *)value = data->velocity_rad_s; }
    else if (strcmp(which, "TEMP") == 0) { *(uint8_t *)value = data->rotor_temperature; }
    else if (strcmp(which, "ERR") == 0) { *(uint8_t *)value = data->error; }
}

static void dm4310_set_para(const struct motor_device *motor, const char *which,
                            const void *value)
{
    dm4310_data_t *data;
    if (motor == NULL || motor->motor_data == NULL || which == NULL || value == NULL) { return; }
    data = motor->motor_data;
    if (strcmp(which, "KP") == 0) { data->kp = clamp_float(*(const float *)value, 0.0f, 500.0f); }
    else if (strcmp(which, "KD") == 0) { data->kd = clamp_float(*(const float *)value, 0.0f, 5.0f); }
}

/* --------------------------- Static instances --------------------------- */
// 6020pitch实例化
static const gm6020_pid_config_t gm6020_pitch_pid_config = {
    .position_pid = {
        .kp = 50.0f,
        .ki = 0.0f,
        .kd = 0.0f,
        .integral_limit = 0.0f,
        .output_limit = GM6020_SPEED_LIMIT_RPM,
        .derivative_filter_alpha = 0.5f,
        .deadband = 0.0f,
        .integral_separation_threshold = 0.0f,
        .variable_integration_threshold = 0.0f,
    },
    .velocity_pid = {
        .kp = 20.0f,
        .ki = 0.0f,
        .kd = 0.0f,
        .integral_limit = 3000.0f,
        .output_limit = GM6020_OUTPUT_LIMIT,
        .derivative_filter_alpha = 0.5f,
        .deadband = 0.0f,
        .integral_separation_threshold = 100.0f,
        .variable_integration_threshold = 50.0f,
    },
};

static gm6020_data_t gm6020_pitch_data = {
    .pid_config = &gm6020_pitch_pid_config,
    .rotational_inertia_kg_m2 = 0.0005,
    .friction_torque = 0.0f,
    .K_t_Nm_A = 0.741,

};

static struct motor_device gm6020_pitch = {
    .motor_name = "GM6020_PITCH",
    .motor_id = CAN_GM6020_PITCH_ID,
    .motor_data = &gm6020_pitch_data,
    .init = gm6020_init,
    .feedback_calculate = gm6020_feedback_calculate,
    .send_enable_cmd = gm6020_enable,
    .send_disable_cmd = gm6020_disable,
    .send_ctrl_cmd = gm6020_send_ctrl_cmd,
    .update = gm6020_update,
    .set_target = gm6020_set_target,
    .set_trace = gm6020_set_trace,
    .trace_update = gm6020_trace_update,
    .get_status = gm6020_get_status,
    .set_para = gm6020_set_para,
};

//6020yaw实例化
static const gm6020_pid_config_t gm6020_yaw_pid_config = {
    .position_pid = {
        .kp = 200.0f,
        .ki = 0.0f,
        .kd = 0.0f,
        .integral_limit = 10.0f,
        .output_limit = GM6020_SPEED_LIMIT_RPM,
        .derivative_filter_alpha = 0.2f,
        .deadband = 0.0f,
        .integral_separation_threshold = 0.0f,
        .variable_integration_threshold = 0.0f,
    },
    .velocity_pid = {
        .kp = 30.0f,
        .ki = 200.0f,
        .kd = 0.0f,
        .integral_limit = 40.0f,
        .output_limit = GM6020_OUTPUT_LIMIT,
        .derivative_filter_alpha = 0.5f,
        .deadband = 0.0f,
        .integral_separation_threshold = 100.0f,
        .variable_integration_threshold = 50.0f,
    },
};

static gm6020_data_t gm6020_yaw_data = {
    .pid_config = &gm6020_yaw_pid_config,
    .rotational_inertia_kg_m2 = 0.000f,
    .friction_torque = 0.0f,
    .K_t_Nm_A = 0.741,
};

static struct motor_device gm6020_yaw = {
    .motor_name = "GM6020_YAW",
    .motor_id = CAN_GM6020_YAW_ID,
    .motor_data = &gm6020_yaw_data,
    .init = gm6020_init,
    .feedback_calculate = gm6020_feedback_calculate,
    .send_enable_cmd = gm6020_enable,
    .send_disable_cmd = gm6020_disable,
    .send_ctrl_cmd = gm6020_send_ctrl_cmd,
    .update = gm6020_update,
    .set_target = gm6020_set_target,
    .set_trace = gm6020_set_trace,
    .trace_update = gm6020_trace_update,
    .get_status = gm6020_get_status,
    .set_para = gm6020_set_para,
};

// 3508_1实例化
static const m3508_pid_config_t m3508_2_pid_config = {
    .position_pid = {
        .kp = 0.0f,
        .ki = 0.0f,
        .kd = 0.0f,
        .integral_limit = 0.0f,
        .output_limit = M3508_SPEED_LIMIT_RPM,
        .derivative_filter_alpha = 0.5f,
        .deadband = 0.0f,
        .integral_separation_threshold = 0.0f,
        .variable_integration_threshold = 0.0f,
    },
    .velocity_pid = {
        .kp = 30.0f,
        .ki = 0.0f,
        .kd = 0.0f,
        .integral_limit = 500.0f,
        .output_limit = M3508_OUTPUT_LIMIT,
        .derivative_filter_alpha = 0.5f,
        .deadband = 0.0f,
        .integral_separation_threshold = 1000.0f,
        .variable_integration_threshold = 500.0f,
    },
};

static m3508_data_t m3508_2_data = {
    .pid_config = &m3508_2_pid_config,
    .rotational_inertia_kg_m2 = 0.0f,
    .friction_torque = 0.0f,
    .K_t_Nm_A = 0.02f,
};

static struct motor_device m3508_2 = {
    .motor_name = "M3508_2",
    .motor_id = CAN_M3508_2_ID,
    .motor_data = &m3508_2_data,
    .init = m3508_init,
    .feedback_calculate = m3508_feedback_calculate,
    .send_enable_cmd = m3508_enable,
    .send_disable_cmd = m3508_disable,
    .send_ctrl_cmd = m3508_send_ctrl_cmd,
    .update = m3508_update,
    .set_target = m3508_set_target,
    .set_trace = m3508_set_trace,
    .trace_update = m3508_trace_update,
    .get_status = m3508_get_status,
    .set_para = m3508_set_para,
};

static dm4310_data_t dm4310_pitch_data;

static struct motor_device dm4310_pitch = {
    .motor_name = "DM4310_PITCH",
    .motor_id = DM_4310_MASTER_ID,
    .motor_data = &dm4310_pitch_data,
    .init = dm4310_init,
    .feedback_calculate = dm4310_feedback_calculate,
    .send_enable_cmd = dm4310_enable,
    .send_disable_cmd = dm4310_disable,
    .send_ctrl_cmd = dm4310_send_ctrl_cmd,
    .update = dm4310_update,
    .set_target = dm4310_set_target,
    .set_trace = dm4310_set_trace,
    .trace_update = dm4310_trace_update,
    .get_status = dm4310_get_status,
    .set_para = dm4310_set_para,
};

static struct motor_device *const motor_list[] = {&gm6020_pitch, &dm4310_pitch, &gm6020_yaw, &m3508_2};
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
    for (index = 0U; index < (sizeof(motor_list) / sizeof(motor_list[0])); ++index) {
        struct motor_device *motor = motor_list[index];
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
    motor_send_standard(can_handle, GM6020_CONTROL_GROUP_ID, frame);
}

// 将挂载在传入can句柄上的3508电机控制量打包下发
static void m3508_send_group(FDCAN_HandleTypeDef *can_handle)
{
    uint8_t frame[8] = {0};
    uint32_t index;
    if (can_handle == NULL) { return; }
    for (index = 0U; index < (sizeof(motor_list) / sizeof(motor_list[0])); ++index) {
        struct motor_device *motor = motor_list[index];
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
    motor_send_standard(can_handle, M3508_CONTROL_GROUP_ID, frame);
}

//得到电机句柄
struct motor_device *motor_get_device(const char *name)
{
    uint32_t index;
    if (name == NULL) { return NULL; }
    for (index = 0U; index < (sizeof(motor_list) / sizeof(motor_list[0])); ++index) {
        if (strcmp(name, motor_list[index]->motor_name) == 0)
            { return motor_list[index]; }
    }
    return NULL;
}

// 得到注册电机数量
uint32_t Motor_Get_Count(void)
{
    return (uint32_t)(sizeof(motor_list) / sizeof(motor_list[0]));
}

bool motor_is_online(const struct motor_device *motor)
{
    if (motor == NULL || motor->last_rx_tick == 0U) {
        return false;
    }
    return (HAL_GetTick() - motor->last_rx_tick) <= MOTOR_OFFLINE_TIMEOUT_MS;
}

void Motor_System_PowerOn_Init(void)
{
    gm6020_pitch.init(&gm6020_pitch, CAN_GM6020_PITCH_ID, &hfdcan1, 0);
    gm6020_yaw.init(&gm6020_yaw, CAN_GM6020_YAW_ID, &hfdcan1, 0);
    m3508_2.init(&m3508_2, CAN_M3508_2_ID, &hfdcan1, 0);
    dm4310_pitch.init(&dm4310_pitch, DM_4310_MASTER_ID, &hfdcan1, 0);
    gm6020_pitch.send_disable_cmd(&gm6020_pitch);
    gm6020_yaw.send_disable_cmd(&gm6020_yaw);
    m3508_2.send_disable_cmd(&m3508_2);
    dm4310_pitch.send_disable_cmd(&dm4310_pitch);
}

void Motor_All_Trace_Update(void)
{
    uint32_t index;

    for (index = 0U; index < Motor_Get_Count(); ++index) {
        if (motor_list[index]->trace_update != NULL) {
            motor_list[index]->trace_update(motor_list[index]);
        }
    }
}

void Motor_All_Update(void)
{
    uint32_t index;
    for (index = 0U; index < Motor_Get_Count(); ++index) {
        motor_list[index]->update(motor_list[index]);
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
        struct motor_device *motor = motor_list[index];
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
            struct motor_device *motor = motor_list[index];
            if (motor->motor_can_handle == handle && motor->motor_id == header.Identifier) {
                motor->feedback_calculate(motor, frame);
                motor->last_rx_tick = HAL_GetTick();
                break;
            }
        }
    }
}
