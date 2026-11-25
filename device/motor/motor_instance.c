/**
 * @file motor_instance.c
 * @brief Hard-coded motor instances and their project-specific parameters.
 */

#include "motor_instance.h"
#include "motor_internal.h"

/* --------------------------- Static instances --------------------------- */
// Pitch电机：CAN2，控制ID为0x55，反馈Master ID为0x66。
// 串级初始参数等效为位置刚度25Nm/rad、速度阻尼2Nm/(rad/s)，后续实机调试。
static const dm4310_pid_config_t dm4310_pitch_pid_config = {
    .position_pid = {
        .kp = 12.5f, .ki = 0.0f, .kd = 0.0f,
        .output_limit = DM4310_V_MAX,
    },
    .velocity_pid = {
        .kp = 2.0f, .ki = 0.0f, .kd = 0.0f,
        .output_limit = DM4310_T_MAX,
    },
};

static dm4310_data_t dm4310_pitch_data = {
    .pid_config = &dm4310_pitch_pid_config,
    .master_id = CAN_DM4310_PITCH_FEEDBACK_ID,
    .command_id = CAN_DM4310_PITCH_COMMAND_ID,
    .rotational_inertia_kg_m2 = 0.0f,
    .friction_torque = 0.0f,
};

static struct motor_device dm4310_pitch = {
    .motor_name = "DM4310_PITCH",
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

// 左摩擦轮：CAN2，电机ID为1，对应反馈ID为0x201。
// 摩擦轮速度环初始参数，test_task中暂不使能。
static const m3508_pid_config_t m3508_shoot_l_pid_config = {
    .position_pid = {
        .kp = 0.0f, .ki = 0.0f, .kd = 0.0f,
        .output_limit = M3508_SPEED_LIMIT_RPM,
    },
    .velocity_pid = {
        .kp = 20.0f, .ki = 0.0f, .kd = 0.0f,
        .output_limit = M3508_OUTPUT_LIMIT,
    },
};

static m3508_data_t m3508_shoot_l_data = {
    .pid_config = &m3508_shoot_l_pid_config,
    .rotational_inertia_kg_m2 = 0.0f,
    .friction_torque = 0.0f,
    .K_t_Nm_A = 0.02f,
};

static struct motor_device m3508_shoot_l = {
    .motor_name = "M3508_SHOOT_L",
    .motor_id = CAN_M3508_SHOOT_L_ID,
    .motor_data = &m3508_shoot_l_data,
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

// 右摩擦轮：CAN2，电机ID为2，对应反馈ID为0x202。
static const m3508_pid_config_t m3508_shoot_r_pid_config = {
    .position_pid = {
        .kp = 0.0f, .ki = 0.0f, .kd = 0.0f,
        .output_limit = M3508_SPEED_LIMIT_RPM,
    },
    .velocity_pid = {
        .kp = 20.0f, .ki = 0.0f, .kd = 0.0f,
        .output_limit = M3508_OUTPUT_LIMIT,
    },
};

static m3508_data_t m3508_shoot_r_data = {
    .pid_config = &m3508_shoot_r_pid_config,
    .rotational_inertia_kg_m2 = 0.0f,
    .friction_torque = 0.0f,
    .K_t_Nm_A = 0.02f,
};

static struct motor_device m3508_shoot_r = {
    .motor_name = "M3508_SHOOT_R",
    .motor_id = CAN_M3508_SHOOT_R_ID,
    .motor_data = &m3508_shoot_r_data,
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

static struct motor_device *const motor_list[] = {&dm4310_pitch, &m3508_shoot_l, &m3508_shoot_r};

uint32_t motor_instance_count(void)
{
    return (uint32_t)(sizeof(motor_list) / sizeof(motor_list[0]));
}

struct motor_device *motor_instance_get(uint32_t index)
{
    if (index >= motor_instance_count()) {
        return NULL;
    }

    return motor_list[index];
}

void Motor_System_PowerOn_Init(void)
{
    dm4310_pitch.init(&dm4310_pitch, 0U, &hfdcan2, 0);
    m3508_shoot_l.init(&m3508_shoot_l, CAN_M3508_SHOOT_L_ID, &hfdcan2, 0);
    m3508_shoot_r.init(&m3508_shoot_r, CAN_M3508_SHOOT_R_ID, &hfdcan2, 0);

    dm4310_pitch.send_disable_cmd(&dm4310_pitch);
    m3508_shoot_l.send_disable_cmd(&m3508_shoot_l);
    m3508_shoot_r.send_disable_cmd(&m3508_shoot_r);
}
