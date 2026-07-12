/**
 * @file can.h
 * @brief Classic CAN configuration for the DM-MC-Board02.
 */

#ifndef DM_CAN_H
#define DM_CAN_H

/* Motor standard identifiers. */
#define DM_4310_MASTER_ID          0x000U
#define CAN_J4310_PITCH_ID         0x001U

#define CAN_M3508_CHASSIS_1_ID     0x201U
#define CAN_M3508_CHASSIS_2_ID     0x202U
#define CAN_M3508_CHASSIS_3_ID     0x203U
#define CAN_M3508_CHASSIS_4_ID     0x204U
#define CAN_M2006_TRIGGER_ID       0x205U
#define CAN_GM6020_YAW_ID          0x206U
#define CAN_M3508_SHOOT_R_ID       0x207U
#define CAN_M3508_SHOOT_L_ID       0x208U

#ifdef __cplusplus
extern "C" {
#endif

/** Configure filters, start FDCAN1/2/3 in Classic CAN mode, and enable FIFO0 RX notification. */
void can_init(void);

#ifdef __cplusplus
}
#endif

#endif /* DM_CAN_H */
