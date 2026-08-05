#ifndef DM_MOTOR_INTERNAL_H
#define DM_MOTOR_INTERNAL_H

#include "motor.h"

#include "../../bsp/can/can.h"
#include "../../dsp/math.h"
#include "../../dsp/pid/pid.h"

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

#define DM3507_MASTER_ID                  0x00U
#define DM3507_COMMAND_ID                 0x01U
#define DM3507_P_MAX                      12.566f
#define DM3507_V_MAX                      100.0f
#define DM3507_T_MAX                      5.0f
#define DM3507_TORQUE_LIMIT_NM            3.0f
#define DM3507_ERR_ENABLED                0x1U
#define DM3507_ERR_FAULT_MIN              0x2U
#define DM3507_ERR_FAULT_MAX              0xEU
#define DM3507_CLEAR_RETRY_MS             50U
#define DM3507_ENABLE_RETRY_MS            20U
#define DM3507_TRACE_MAX_ACCELERATION_RAD_S2 100.0f

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

#if (DM3507_MASTER_ID == DM_4310_MASTER_ID) || \
    (DM3507_MASTER_ID == DM4310_COMMAND_ID) || \
    (DM3507_COMMAND_ID == DM_4310_MASTER_ID) || \
    (DM3507_COMMAND_ID == DM4310_COMMAND_ID)
#error "DM3507 and DM4310 CAN identifiers must be unique on FDCAN1"
#endif

typedef struct {
    pid_t position_pid;
    pid_t velocity_pid;
} gm6020_pid_config_t;

typedef struct {
    pid_t position_pid;
    pid_t velocity_pid;
} m3508_pid_config_t;

typedef struct {
    pid_t position_pid;
    pid_t velocity_pid;
} dm3507_pid_config_t;

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
    const dm3507_pid_config_t *pid_config;
    pid_t position_pid;
    pid_t velocity_pid;
    float target_position_rad;
    float target_velocity_rad_s;
    float target_acceleration_rad_s2;
    float rotational_inertia_kg_m2;
    float friction_torque;
    motor_trace_t trace;
    float position_rad;
    float velocity_rad_s;
    float torque_nm;
    float output_torque_nm;
    float p_max;
    float v_max;
    float t_max;
    uint8_t can_id;
    uint8_t error;
    uint8_t mos_temperature;
    uint8_t rotor_temperature;
    uint8_t enable_requested;
    uint8_t enabled;
    uint8_t hold_position_pending;
    uint32_t last_update_tick;
    uint32_t last_clear_cmd_tick;
    uint32_t last_enable_cmd_tick;
} dm3507_data_t;

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

#define MOTOR_DRIVER_DECLARATIONS(prefix) \
    void prefix##_init(struct motor_device *motor, uint32_t motor_id, \
                       FDCAN_HandleTypeDef *can_handle, int para_num, ...); \
    void prefix##_feedback_calculate(const struct motor_device *motor, \
                                     const uint8_t data[8]); \
    void prefix##_send_ctrl_cmd(struct motor_device *motor); \
    void prefix##_enable(struct motor_device *motor); \
    void prefix##_disable(struct motor_device *motor); \
    void prefix##_update(struct motor_device *motor); \
    void prefix##_set_target(const struct motor_device *motor, int para_num, ...); \
    void prefix##_set_trace(const struct motor_device *motor, \
                            float target_position_rad, float duration_s); \
    void prefix##_trace_update(struct motor_device *motor); \
    void prefix##_get_status(const struct motor_device *motor, \
                             const char *which, void *value); \
    void prefix##_set_para(const struct motor_device *motor, \
                           const char *which, const void *value)

MOTOR_DRIVER_DECLARATIONS(gm6020);
MOTOR_DRIVER_DECLARATIONS(m3508);
MOTOR_DRIVER_DECLARATIONS(dm3507);
MOTOR_DRIVER_DECLARATIONS(dm4310);

#undef MOTOR_DRIVER_DECLARATIONS

#endif /* DM_MOTOR_INTERNAL_H */
