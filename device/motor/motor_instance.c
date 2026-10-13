/**
 * @file motor_instance.c
 * @brief Hard-coded motor instances and their project-specific parameters.
 */

#include "motor_instance.h"
#include "motor_internal.h"

/* --------------------------- Static instances --------------------------- */
// 3508_L实例化
static const m3508_pid_config_t m3508_l_pid_config = {
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

static m3508_data_t m3508_l_data = {
    .pid_config = &m3508_l_pid_config,
    .rotational_inertia_kg_m2 = 0.000f,
    .friction_torque = 0.0f,
    .K_t_Nm_A = 0.02f,
};

static struct motor_device m3508_l = {
    .motor_name = "M3508_L",
    .motor_id = CAN_M3508_L_ID,
    .motor_data = &m3508_l_data,
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

// 3508_R实例化
static const m3508_pid_config_t m3508_r_pid_config = {
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

static m3508_data_t m3508_r_data = {
    .pid_config = &m3508_r_pid_config,
    .rotational_inertia_kg_m2 = 0.000f,
    .friction_torque = 0.0f,
    .K_t_Nm_A = 0.02f,
};

static struct motor_device m3508_r = {
    .motor_name = "M3508_R",
    .motor_id = CAN_M3508_R_ID,
    .motor_data = &m3508_r_data,
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

// DM8009P_LF实例化：MIT模式，CAN ID为0x001，反馈Master ID为0x011
static const dm8009p_pid_config_t dm8009p_lf_pid_config = {
    .position_pid = {
        .kp = 20.0f, .ki = 0.0f, .kd = 0.0f,
        .integral_limit = 0.0f,
        .output_limit = DM8009P_V_MAX,
        .derivative_filter_alpha = 0.5f,
        .deadband = 0.002f,
        .integral_separation_threshold = 0.0f,
        .variable_integration_threshold = 0.0f,
    },
    .velocity_pid = {
        .kp = 0.10f, .ki = 0.0f, .kd = 0.0f,
        .integral_limit = 1.0f,
        .output_limit = DM8009P_TORQUE_LIMIT_NM,
        .derivative_filter_alpha = 0.5f,
        .deadband = 0.05f,
        .integral_separation_threshold = 20.0f,
        .variable_integration_threshold = 10.0f,
    },
};

static dm8009p_data_t dm8009p_lf_data = {
    .pid_config = &dm8009p_lf_pid_config,
    .master_id = CAN_DM8009P_LF_FEEDBACK_ID,
    .command_id = CAN_DM8009P_LF_COMMAND_ID,
    .rotational_inertia_kg_m2 = 0.0f,
    .friction_torque = 0.0f,
};

static struct motor_device dm8009p_lf = {
    .motor_name = "DM8009P_LF",
    .motor_data = &dm8009p_lf_data,
    .init = dm8009p_init,
    .feedback_calculate = dm8009p_feedback_calculate,
    .send_enable_cmd = dm8009p_enable,
    .send_disable_cmd = dm8009p_disable,
    .send_ctrl_cmd = dm8009p_send_ctrl_cmd,
    .update = dm8009p_update,
    .set_target = dm8009p_set_target,
    .set_trace = dm8009p_set_trace,
    .trace_update = dm8009p_trace_update,
    .get_status = dm8009p_get_status,
    .set_para = dm8009p_set_para,
};

// DM_8009P_LB实例化：MIT模式，CAN ID为0x002，反馈Master ID为0x012
static const dm8009p_pid_config_t dm_8009p_lb_pid_config = {
    .position_pid = {
        .kp = 20.0f, .ki = 0.0f, .kd = 0.0f,
        .integral_limit = 0.0f,
        .output_limit = DM8009P_V_MAX,
        .derivative_filter_alpha = 0.5f,
        .deadband = 0.002f,
        .integral_separation_threshold = 0.0f,
        .variable_integration_threshold = 0.0f,
    },
    .velocity_pid = {
        .kp = 0.10f, .ki = 0.0f, .kd = 0.0f,
        .integral_limit = 1.0f,
        .output_limit = DM8009P_TORQUE_LIMIT_NM,
        .derivative_filter_alpha = 0.5f,
        .deadband = 0.05f,
        .integral_separation_threshold = 20.0f,
        .variable_integration_threshold = 10.0f,
    },
};

static dm8009p_data_t dm_8009p_lb_data = {
    .pid_config = &dm_8009p_lb_pid_config,
    .master_id = CAN_DM_8009P_LB_FEEDBACK_ID,
    .command_id = CAN_DM_8009P_LB_COMMAND_ID,
    .rotational_inertia_kg_m2 = 0.0f,
    .friction_torque = 0.0f,
};

static struct motor_device dm_8009p_lb = {
    .motor_name = "DM_8009P_LB",
    .motor_data = &dm_8009p_lb_data,
    .init = dm8009p_init,
    .feedback_calculate = dm8009p_feedback_calculate,
    .send_enable_cmd = dm8009p_enable,
    .send_disable_cmd = dm8009p_disable,
    .send_ctrl_cmd = dm8009p_send_ctrl_cmd,
    .update = dm8009p_update,
    .set_target = dm8009p_set_target,
    .set_trace = dm8009p_set_trace,
    .trace_update = dm8009p_trace_update,
    .get_status = dm8009p_get_status,
    .set_para = dm8009p_set_para,
};

// DM_8009P_RF实例化：MIT模式，CAN ID为0x003，反馈Master ID为0x013
static const dm8009p_pid_config_t dm_8009p_rf_pid_config = {
    .position_pid = {
        .kp = 20.0f, .ki = 0.0f, .kd = 0.0f,
        .integral_limit = 0.0f,
        .output_limit = DM8009P_V_MAX,
        .derivative_filter_alpha = 0.5f,
        .deadband = 0.002f,
        .integral_separation_threshold = 0.0f,
        .variable_integration_threshold = 0.0f,
    },
    .velocity_pid = {
        .kp = 0.10f, .ki = 0.0f, .kd = 0.0f,
        .integral_limit = 1.0f,
        .output_limit = DM8009P_TORQUE_LIMIT_NM,
        .derivative_filter_alpha = 0.5f,
        .deadband = 0.05f,
        .integral_separation_threshold = 20.0f,
        .variable_integration_threshold = 10.0f,
    },
};

static dm8009p_data_t dm_8009p_rf_data = {
    .pid_config = &dm_8009p_rf_pid_config,
    .master_id = CAN_DM_8009P_RF_FEEDBACK_ID,
    .command_id = CAN_DM_8009P_RF_COMMAND_ID,
    .rotational_inertia_kg_m2 = 0.0f,
    .friction_torque = 0.0f,
};

static struct motor_device dm_8009p_rf = {
    .motor_name = "DM_8009P_RF",
    .motor_data = &dm_8009p_rf_data,
    .init = dm8009p_init,
    .feedback_calculate = dm8009p_feedback_calculate,
    .send_enable_cmd = dm8009p_enable,
    .send_disable_cmd = dm8009p_disable,
    .send_ctrl_cmd = dm8009p_send_ctrl_cmd,
    .update = dm8009p_update,
    .set_target = dm8009p_set_target,
    .set_trace = dm8009p_set_trace,
    .trace_update = dm8009p_trace_update,
    .get_status = dm8009p_get_status,
    .set_para = dm8009p_set_para,
};

// DM_8009P_RB实例化：MIT模式，CAN ID为0x004，反馈Master ID为0x014
static const dm8009p_pid_config_t dm_8009p_rb_pid_config = {
    .position_pid = {
        .kp = 20.0f, .ki = 0.0f, .kd = 0.0f,
        .integral_limit = 0.0f,
        .output_limit = DM8009P_V_MAX,
        .derivative_filter_alpha = 0.5f,
        .deadband = 0.002f,
        .integral_separation_threshold = 0.0f,
        .variable_integration_threshold = 0.0f,
    },
    .velocity_pid = {
        .kp = 0.10f, .ki = 0.0f, .kd = 0.0f,
        .integral_limit = 1.0f,
        .output_limit = DM8009P_TORQUE_LIMIT_NM,
        .derivative_filter_alpha = 0.5f,
        .deadband = 0.05f,
        .integral_separation_threshold = 20.0f,
        .variable_integration_threshold = 10.0f,
    },
};

static dm8009p_data_t dm_8009p_rb_data = {
    .pid_config = &dm_8009p_rb_pid_config,
    .master_id = CAN_DM_8009P_RB_FEEDBACK_ID,
    .command_id = CAN_DM_8009P_RB_COMMAND_ID,
    .rotational_inertia_kg_m2 = 0.0f,
    .friction_torque = 0.0f,
};

static struct motor_device dm_8009p_rb = {
    .motor_name = "DM_8009P_RB",
    .motor_data = &dm_8009p_rb_data,
    .init = dm8009p_init,
    .feedback_calculate = dm8009p_feedback_calculate,
    .send_enable_cmd = dm8009p_enable,
    .send_disable_cmd = dm8009p_disable,
    .send_ctrl_cmd = dm8009p_send_ctrl_cmd,
    .update = dm8009p_update,
    .set_target = dm8009p_set_target,
    .set_trace = dm8009p_set_trace,
    .trace_update = dm8009p_trace_update,
    .get_status = dm8009p_get_status,
    .set_para = dm8009p_set_para,
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

// MG4005E_STIR实例化：速度模式，无PID参数以及轨迹等
static mg4005e_data_t mg4005e_stir_data = {0};

static struct motor_device mg4005e_stir = {
    .motor_name = "MG4005E_STIR",
    .motor_id = CAN_MG4005E_STIR_ID,
    .motor_data = &mg4005e_stir_data,
    .init = mg4005e_init,
    .feedback_calculate = mg4005e_feedback_calculate,
    .send_enable_cmd = mg4005e_enable,
    .send_disable_cmd = mg4005e_disable,
    .send_ctrl_cmd = mg4005e_send_ctrl_cmd,
    .update = mg4005e_update,
    .set_target = mg4005e_set_target,
    .set_trace = mg4005e_set_trace,
    .trace_update = mg4005e_trace_update,
    .get_status = mg4005e_get_status,
    .set_para = mg4005e_set_para,
};

static struct motor_device *const motor_list[] = {&m3508_l, &m3508_r, &dm8009p_lf, &dm_8009p_lb, &dm_8009p_rf, &dm_8009p_rb, &gm6020_yaw, &mg4005e_stir};

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
    m3508_l.init(&m3508_l, CAN_M3508_L_ID, &hfdcan1, 0);
    m3508_r.init(&m3508_r, CAN_M3508_R_ID, &hfdcan1, 0);
    dm8009p_lf.init(&dm8009p_lf, 0U, &hfdcan2, 0);
    dm_8009p_lb.init(&dm_8009p_lb, 0U, &hfdcan2, 0);
    dm_8009p_rf.init(&dm_8009p_rf, 0U, &hfdcan2, 0);
    dm_8009p_rb.init(&dm_8009p_rb, 0U, &hfdcan2, 0);
    gm6020_yaw.init(&gm6020_yaw, CAN_GM6020_YAW_ID, &hfdcan3, 0);
    mg4005e_stir.init(&mg4005e_stir, CAN_MG4005E_STIR_ID, &hfdcan3, 0);

    m3508_l.send_disable_cmd(&m3508_l);
    m3508_r.send_disable_cmd(&m3508_r);
    dm8009p_lf.send_disable_cmd(&dm8009p_lf);
    dm_8009p_lb.send_disable_cmd(&dm_8009p_lb);
    dm_8009p_rf.send_disable_cmd(&dm_8009p_rf);
    dm_8009p_rb.send_disable_cmd(&dm_8009p_rb);
    gm6020_yaw.send_disable_cmd(&gm6020_yaw);
    mg4005e_stir.send_disable_cmd(&mg4005e_stir);
}
