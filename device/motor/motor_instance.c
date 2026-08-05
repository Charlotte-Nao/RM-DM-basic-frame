/**
 * @file motor_instance.c
 * @brief Hard-coded motor instances and their project-specific parameters.
 */

#include "motor_instance.h"
#include "motor_internal.h"

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
        .kp = 80.0f,
        .ki = 0.0f,
        .kd = 0.0f,
        .integral_limit = 0.0f,
        .output_limit = M3508_SPEED_LIMIT_RPM,
        .derivative_filter_alpha = 0.0f,
        .deadband = 0.0f,
        .integral_separation_threshold = 0.0f,
        .variable_integration_threshold = 0.0f,
    },
    .velocity_pid = {
        .kp = 15.0f,
        .ki = 40.0f,
        .kd = 0.0f,
        .integral_limit = 50.0f,
        .output_limit = M3508_OUTPUT_LIMIT,
        .derivative_filter_alpha = 0.0f,
        .deadband = 0.0f,
        .integral_separation_threshold = 1000.0f,
        .variable_integration_threshold = 500.0f,
    },
};

static m3508_data_t m3508_2_data = {
    .pid_config = &m3508_2_pid_config,
    .rotational_inertia_kg_m2 = 0.000f,
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

// DM3507示例实例化：MIT模式，CAN ID为0x001，反馈Master ID为0x000
static const dm3507_pid_config_t dm3507_1_pid_config = {
    .position_pid = {
        .kp = 20.0f, .ki = 0.0f, .kd = 0.0f,
        .integral_limit = 0.0f,
        .output_limit = 48.17f,
        .derivative_filter_alpha = 0.5f,
        .deadband = 0.002f,
        .integral_separation_threshold = 0.0f,
        .variable_integration_threshold = 0.0f,
    },
    .velocity_pid = {
        .kp = 0.10f, .ki = 0.0f, .kd = 0.0f,
        .integral_limit = 1.0f,
        .output_limit = DM3507_TORQUE_LIMIT_NM,
        .derivative_filter_alpha = 0.5f,
        .deadband = 0.05f,
        .integral_separation_threshold = 20.0f,
        .variable_integration_threshold = 10.0f,
    },
};

static dm3507_data_t dm3507_1_data = {
    .pid_config = &dm3507_1_pid_config,
    .master_id = 0x00U,
    .command_id = 0x01U,
    .rotational_inertia_kg_m2 = 0.0f,
    .friction_torque = 0.0f,
};

static struct motor_device dm3507_1 = {
    .motor_name = "DM3507_1",
    .motor_data = &dm3507_1_data,
    .init = dm3507_init,
    .feedback_calculate = dm3507_feedback_calculate,
    .send_enable_cmd = dm3507_enable,
    .send_disable_cmd = dm3507_disable,
    .send_ctrl_cmd = dm3507_send_ctrl_cmd,
    .update = dm3507_update,
    .set_target = dm3507_set_target,
    .set_trace = dm3507_set_trace,
    .trace_update = dm3507_trace_update,
    .get_status = dm3507_get_status,
    .set_para = dm3507_set_para,
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

static struct motor_device *const motor_list[] = {&gm6020_pitch, &dm4310_pitch, &gm6020_yaw, &m3508_2, &dm3507_1};

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
    gm6020_pitch.init(&gm6020_pitch, CAN_GM6020_PITCH_ID, &hfdcan1, 0);
    gm6020_yaw.init(&gm6020_yaw, CAN_GM6020_YAW_ID, &hfdcan1, 0);
    m3508_2.init(&m3508_2, CAN_M3508_2_ID, &hfdcan1, 0);
    dm4310_pitch.init(&dm4310_pitch, DM_4310_MASTER_ID, &hfdcan1, 0);
    dm3507_1.init(&dm3507_1, 0U, &hfdcan1, 0);

    gm6020_pitch.send_disable_cmd(&gm6020_pitch);
    gm6020_yaw.send_disable_cmd(&gm6020_yaw);
    m3508_2.send_disable_cmd(&m3508_2);
    dm4310_pitch.send_disable_cmd(&dm4310_pitch);
    dm3507_1.send_disable_cmd(&dm3507_1);
}
