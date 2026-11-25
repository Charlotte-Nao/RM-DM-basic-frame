/**
 * @file can.h
 * @brief Classic CAN configuration for the DM-MC-Board02.
 */

#ifndef DM_CAN_H
#define DM_CAN_H

/* CAN1: 云台板与底盘板通信。 */
#define CAN_GIMBAL_YAW_ID                     0x012U
#define CAN_GIMBAL_COMMAND_ID                 0x025U

/* CAN2: Pitch电机和摩擦轮电机。 */
#define CAN_DM4310_PITCH_COMMAND_ID           0x055U
#define CAN_DM4310_PITCH_FEEDBACK_ID          0x066U
#define CAN_M3508_SHOOT_L_ID                  0x201U
#define CAN_M3508_SHOOT_R_ID                  0x202U

#ifdef __cplusplus
extern "C" {
#endif

void can_init(void);

#ifdef __cplusplus
}
#endif

#endif /* DM_CAN_H */
