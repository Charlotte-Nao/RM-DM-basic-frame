/**
 * @file motor.c
 * @brief GM6020 and DM4310 static motor-device implementations.
 */

#include "motor.h"

#include <stdarg.h>
#include <string.h>

#include "../../bsp/can/can.h"
#include "../../dsp/pid/pid.h"

#define MOTOR_CONTROL_DT_DEFAULT_S       0.001f
#define MOTOR_OFFLINE_TIMEOUT_MS         100U
#define MOTOR_TWO_PI                     6.28318530717958647692f

#define GM6020_CONTROL_GROUP_ID          0x1FFU
#define GM6020_ENCODER_RESOLUTION         8192.0f
#define GM6020_OUTPUT_LIMIT               25000.0f
#define GM6020_SPEED_LIMIT_RPM            320.0f

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

typedef struct {
    pid_t position_pid;
    pid_t velocity_pid;
} gm6020_pid_config_t;

typedef struct {
    const gm6020_pid_config_t *pid_config;
    pid_t position_pid;
    pid_t velocity_pid;
    float target_position_rad;
    float target_velocity_rpm;
    float position_rad;
    float velocity_rpm;
    float torque_current;
    float output_voltage;
    uint16_t encoder;
    int16_t speed_rpm;
    int16_t current;
    uint8_t temperature;
    uint8_t enabled;
    uint32_t last_update_tick;
    uint8_t control_slot;               /* GM6020 ID: 1..4 in group 0x1FF. */
} gm6020_data_t;

typedef struct {
    float target_position_rad;
    float target_velocity_rad_s;
    float target_torque_nm;
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

static void motor_send_standard(FDCAN_HandleTypeDef *handle, uint32_t identifier,
                                const uint8_t data[8])
{
    FDCAN_TxHeaderTypeDef header = {0};

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
    (void)HAL_FDCAN_AddMessageToTxFifoQ(handle, &header, (uint8_t *)data);
}

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

static uint16_t float_to_uint(float value, float minimum, float maximum, uint16_t bits)
{
    float scaled;
    uint32_t maximum_integer = (1UL << bits) - 1UL;

    value = clamp_float(value, minimum, maximum);
    scaled = (value - minimum) * (float)maximum_integer / (maximum - minimum);
    return (uint16_t)scaled;
}

static void gm6020_send_group(FDCAN_HandleTypeDef *can_handle);

/* ------------------------------ GM6020 ---------------------------------- */

static void gm6020_init(struct motor_device *motor, uint32_t motor_id,
                        FDCAN_HandleTypeDef *can_handle, int para_num, ...)
{
    gm6020_data_t *data;
    const gm6020_pid_config_t *pid_config;

    if (motor == NULL || motor->motor_data == NULL || can_handle == NULL) {
        return;
    }
    data = motor->motor_data;
    pid_config = data->pid_config;
    if (pid_config == NULL) {
        return;
    }
    memset(data, 0, sizeof(*data));
    data->pid_config = pid_config;
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

static void gm6020_feedback_calculate(const struct motor_device *motor,
                                      const uint8_t frame[8])
{
    gm6020_data_t *data;

    if (motor == NULL || motor->motor_data == NULL || frame == NULL) {
        return;
    }
    data = motor->motor_data;
    data->encoder = (uint16_t)(((uint16_t)frame[0] << 8) | frame[1]);
    data->speed_rpm = (int16_t)(((uint16_t)frame[2] << 8) | frame[3]);
    data->current = (int16_t)(((uint16_t)frame[4] << 8) | frame[5]);
    data->temperature = frame[6];
    data->position_rad = (float)data->encoder * MOTOR_TWO_PI / GM6020_ENCODER_RESOLUTION;
    data->velocity_rpm = (float)data->speed_rpm;
    data->torque_current = (float)data->current;
}

static void gm6020_send_ctrl_cmd(struct motor_device *motor)
{
    if (motor == NULL || motor->motor_data == NULL) {
        return;
    }
    /* A GM6020 control command is always a complete shared 0x1FF group frame. */
    gm6020_send_group(motor->motor_can_handle);
}

static void gm6020_enable(struct motor_device *motor)
{
    gm6020_data_t *data;
    if (motor == NULL || motor->motor_data == NULL) { return; }
    /* Position is unknown until the first feedback frame: never enable blindly. */
    if (!motor_is_online(motor)) { return; }
    data = motor->motor_data;
    /* Prevent a stored target from commanding a jump after re-enable. */
    data->target_position_rad = data->position_rad;
    data->target_velocity_rpm = 0.0f;
    pid_reset(&data->position_pid);
    pid_reset(&data->velocity_pid);
    data->last_update_tick = 0U;
    data->enabled = 1U;
}

static void gm6020_disable(struct motor_device *motor)
{
    gm6020_data_t *data;

    if (motor == NULL || motor->motor_data == NULL) {
        return;
    }
    data = motor->motor_data;
    data->enabled = 0U;
    data->output_voltage = 0.0f;
    data->target_position_rad = data->position_rad;
    data->target_velocity_rpm = 0.0f;
    pid_reset(&data->position_pid);
    pid_reset(&data->velocity_pid);
    data->last_update_tick = 0U;
    gm6020_send_ctrl_cmd(motor);
}

static void gm6020_update(struct motor_device *motor)
{
    gm6020_data_t *data;
    float position_error;
    float desired_velocity;
    float dt;
    uint32_t now;

    if (motor == NULL || motor->motor_data == NULL) {
        return;
    }
    data = motor->motor_data;
    if (data->enabled == 0U || motor->last_rx_tick == 0U ||
        (HAL_GetTick() - motor->last_rx_tick) > MOTOR_OFFLINE_TIMEOUT_MS) {
        data->output_voltage = 0.0f;
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
    position_error = data->target_position_rad - data->position_rad;
    while (position_error > 3.14159265358979323846f) { position_error -= MOTOR_TWO_PI; }
    while (position_error < -3.14159265358979323846f) { position_error += MOTOR_TWO_PI; }
    desired_velocity = pid_update(&data->position_pid, position_error, 0.0f, dt);
    desired_velocity += data->target_velocity_rpm;
    desired_velocity = clamp_float(desired_velocity,
                                   -GM6020_SPEED_LIMIT_RPM, GM6020_SPEED_LIMIT_RPM);
    data->output_voltage = pid_update(&data->velocity_pid, desired_velocity,
                                      data->velocity_rpm, dt);
}

static void gm6020_set_target(const struct motor_device *motor, int para_num, ...)
{
    gm6020_data_t *data;
    va_list arguments;

    if (motor == NULL || motor->motor_data == NULL) {
        return;
    }
    data = motor->motor_data;
    va_start(arguments, para_num);
    if (para_num >= 1) { data->target_position_rad = (float)va_arg(arguments, double); }
    if (para_num >= 2) { data->target_velocity_rpm = (float)va_arg(arguments, double); }
    va_end(arguments);
}

static void gm6020_get_status(const struct motor_device *motor, const char *which,
                              void *value)
{
    gm6020_data_t *data;
    if (motor == NULL || motor->motor_data == NULL || which == NULL || value == NULL) { return; }
    data = motor->motor_data;
    if (strcmp(which, "POS") == 0) { *(float *)value = data->position_rad; }
    else if (strcmp(which, "VEL") == 0) { *(float *)value = data->velocity_rpm; }
    else if (strcmp(which, "TEMP") == 0) { *(uint8_t *)value = data->temperature; }
    else if (strcmp(which, "TARGET") == 0) { *(float *)value = data->target_position_rad; }
}

static void gm6020_set_para(const struct motor_device *motor, const char *which,
                            const void *value)
{
    gm6020_data_t *data;
    if (motor == NULL || motor->motor_data == NULL || which == NULL || value == NULL) { return; }
    data = motor->motor_data;
    if (strcmp(which, "POS_KP") == 0) { data->position_pid.kp = *(const float *)value; }
    else if (strcmp(which, "VEL_KP") == 0) { data->velocity_pid.kp = *(const float *)value; }
    else if (strcmp(which, "VEL_KI") == 0) { data->velocity_pid.ki = *(const float *)value; }
    else if (strcmp(which, "VEL_KD") == 0) { data->velocity_pid.kd = *(const float *)value; }
}

/* ------------------------------ DM4310 ---------------------------------- */

static void dm4310_send_special(struct motor_device *motor, uint8_t command)
{
    uint8_t frame[8] = {0xFFU, 0xFFU, 0xFFU, 0xFFU,
                        0xFFU, 0xFFU, 0xFFU, command};
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
    dm4310_send_special(motor, 0xFDU);
}

static void dm4310_update(struct motor_device *motor)
{
    dm4310_data_t *data;
    uint32_t now;
    if (motor == NULL || motor->motor_data == NULL) { return; }
    data = motor->motor_data;
    if (motor->last_rx_tick == 0U ||
        (HAL_GetTick() - motor->last_rx_tick) > MOTOR_OFFLINE_TIMEOUT_MS) { return; }
    if (data->enable_requested == 0U) { return; }

    now = HAL_GetTick();
    if (data->error >= DM4310_ERR_FAULT_MIN && data->error <= DM4310_ERR_FAULT_MAX) {
        if ((uint32_t)(now - data->last_clear_cmd_tick) >= DM4310_CLEAR_RETRY_MS) {
            dm4310_send_special(motor, 0xFBU);
            data->last_clear_cmd_tick = now;
        }
        return;
    }
    if (data->error != DM4310_ERR_ENABLED) {
        if ((uint32_t)(now - data->last_enable_cmd_tick) >= DM4310_ENABLE_RETRY_MS) {
            dm4310_send_special(motor, 0xFCU);
            data->last_enable_cmd_tick = now;
        }
        return;
    }
    dm4310_send_ctrl_cmd(motor);
}

static void dm4310_set_target(const struct motor_device *motor, int para_num, ...)
{
    dm4310_data_t *data;
    va_list arguments;
    if (motor == NULL || motor->motor_data == NULL) { return; }
    data = motor->motor_data;
    va_start(arguments, para_num);
    if (para_num >= 1) { data->target_position_rad = (float)va_arg(arguments, double); }
    if (para_num >= 2) { data->target_velocity_rad_s = (float)va_arg(arguments, double); }
    if (para_num >= 3) { data->target_torque_nm = (float)va_arg(arguments, double); }
    va_end(arguments);
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

static const gm6020_pid_config_t gm6020_yaw_pid_config = {
    .position_pid = {
        .kp = 15.0f, .ki = 0.0f, .kd = 0.0f,
        .integral_limit = 0.0f, .output_limit = GM6020_SPEED_LIMIT_RPM,
        .derivative_filter_alpha = 0.2f, .deadband = 0.002f,
        .integral_separation_threshold = 0.0f,
        .variable_integration_threshold = 0.0f,
    },
    .velocity_pid = {
        .kp = 80.0f, .ki = 0.0f, .kd = 0.0f,
        .integral_limit = 3000.0f, .output_limit = GM6020_OUTPUT_LIMIT,
        .derivative_filter_alpha = 0.2f, .deadband = 1.0f,
        .integral_separation_threshold = 100.0f,
        .variable_integration_threshold = 50.0f,
    },
};

static gm6020_data_t gm6020_yaw_data = {
    .pid_config = &gm6020_yaw_pid_config,
};
static dm4310_data_t dm4310_pitch_data;

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
    .get_status = gm6020_get_status,
    .set_para = gm6020_set_para,
};

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
    .get_status = dm4310_get_status,
    .set_para = dm4310_set_para,
};

static struct motor_device *const motor_list[] = {&gm6020_yaw, &dm4310_pitch};

/* Build every byte of 0x1FF from the registered GM6020s on this CAN bus.
 * This prevents one motor's update from zeroing the other three control slots. */
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

        if (motor->motor_can_handle != can_handle ||
            motor->send_ctrl_cmd != gm6020_send_ctrl_cmd ||
            motor->motor_data == NULL) {
            continue;
        }
        data = motor->motor_data;
        if (data->control_slot == 0U || data->control_slot > 4U) {
            continue;
        }
        output = (data->enabled != 0U) ? (int16_t)data->output_voltage : 0;
        offset = (uint8_t)((data->control_slot - 1U) * 2U);
        frame[offset] = (uint8_t)((uint16_t)output >> 8);
        frame[offset + 1U] = (uint8_t)output;
    }
    motor_send_standard(can_handle, GM6020_CONTROL_GROUP_ID, frame);
}

struct motor_device *motor_get_device(const char *name)
{
    uint32_t index;
    if (name == NULL) { return NULL; }
    for (index = 0U; index < (sizeof(motor_list) / sizeof(motor_list[0])); ++index) {
        if (strcmp(name, motor_list[index]->motor_name) == 0) { return motor_list[index]; }
    }
    return NULL;
}

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
    gm6020_yaw.init(&gm6020_yaw, CAN_GM6020_YAW_ID, &hfdcan1, 0);
    dm4310_pitch.init(&dm4310_pitch, DM_4310_MASTER_ID, &hfdcan2, 0);
    gm6020_yaw.send_disable_cmd(&gm6020_yaw);
    dm4310_pitch.send_disable_cmd(&dm4310_pitch);
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
    FDCAN_HandleTypeDef *sent_handles[4] = {0};
    uint32_t sent_count = 0U;
    uint32_t index;

    /* One complete frame per used CAN bus, after every GM output is fresh. */
    for (index = 0U; index < Motor_Get_Count(); ++index) {
        struct motor_device *motor = motor_list[index];
        uint32_t sent_index;
        if (motor->send_ctrl_cmd != gm6020_send_ctrl_cmd ||
            motor->motor_can_handle == NULL) {
            continue;
        }
        for (sent_index = 0U; sent_index < sent_count; ++sent_index) {
            if (sent_handles[sent_index] == motor->motor_can_handle) {
                break;
            }
        }
        if (sent_index == sent_count && sent_count < (sizeof(sent_handles) / sizeof(sent_handles[0]))) {
            sent_handles[sent_count++] = motor->motor_can_handle;
            gm6020_send_group(motor->motor_can_handle);
        }
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *handle, uint32_t interrupts)
{
    FDCAN_RxHeaderTypeDef header;
    uint8_t frame[8];
    uint32_t index;

    if ((interrupts & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U) { return; }
    while (HAL_FDCAN_GetRxFifoFillLevel(handle, FDCAN_RX_FIFO0) > 0U) {
        if (HAL_FDCAN_GetRxMessage(handle, FDCAN_RX_FIFO0, &header, frame) != HAL_OK) { break; }
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
